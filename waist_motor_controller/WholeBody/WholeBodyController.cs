using System.Diagnostics;
using System.Text.Json;
using WaistModel;
using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.WholeBody;
public enum GroupState { Observing, Arming, Holding, Executing, Stopping, Settling, Disarming, FaultLatched }
internal sealed class FeedbackNotReadyException(string message) : InvalidOperationException(message);

public sealed class WholeBodyController : IDisposable
{
    private const int SettledErrorRaw = 20;
    private static readonly int ParkingHoldErrorRaw = (int)Math.Ceiling(
        (Calibration.WholeBodyPlanningMarginDeg - Calibration.WholeBodyAlarmMarginDeg) * 65536 / 360);
    private const double ParkingRecoveryResponseSeconds = .16;
    private const int ParkingRecoveryStableMs = 200;
    private const double GearRatio = 161;
    readonly object gate=new(); readonly ICanTransport transport; readonly MotorRegistry registry; readonly MotorController legacy; readonly Robot robot;
    readonly CancellationTokenSource cancel=new(); readonly Thread thread;
    Calibration calibration; int[] ids=[]; short[] lastRaw=new short[4]; double[] hold=new double[4]; MotionPlan? plan;
    short[] previousRaw = [];
    double lastTargetInterval;
    long lastTargetSentAt;
    private CspTrace? recording;
    private int parkingAxis = -1;
    private int parkingLockAxis = -1;
    private long holdingStableSince;
    private long insidePlanningSince;
    private bool stoppedBeforeTarget;
    private bool settlingInParking;
    private bool targetReached;
    private readonly int[] peakMotorRpm = new int[4];
    private int peakTrackingErrorRaw;
    private int peakTrackingAxis;
    double elapsed,stopStart,stopElapsed; long armedAt,settledAt,settleStarted; bool owns; string message="被动观察";
    GroupState state; int lateCount; double maxInterval;
    MotorFeedback[]? coherentFeedback;
    int feedbackPollTick;
    bool exitRequested;
    long disableSentAt;
    (double CrossTrackMm, double AlongErrorMm, double AngleErrorDeg)? moveLDeviation;
    public event Action<string>? Log;
    public GroupState State { get {lock(gate)return state;} }
    public string Status { get {lock(gate)return $"{state} · {message} · 最大周期 {maxInterval:F1} ms";} }
    public bool Owns {get {lock(gate)return owns;} }
    internal bool InParkingZone { get { lock (gate) return owns && parkingLockAxis >= 0; } }
    internal bool CanRecoverParking { get { lock (gate) return IsParkingRecoveryReady(MotorFeedback.Now); } }
    private bool IsParkingRecoveryReady(long now) => owns && state == GroupState.Holding && parkingAxis >= 0 &&
        holdingStableSince != 0 && now - holdingStableSince >= ParkingRecoveryStableMs;
    internal (GroupState State, bool TargetReached, bool StoppedBeforeTarget) MotionStatus
    {
        get { lock (gate) return (state, targetReached, stoppedBeforeTarget); }
    }
    public string? MoveLTracking
    {
        get
        {
            lock (gate)
                return plan is not null && moveLDeviation is { } error
                    ? $"横偏 {error.CrossTrackMm:F2} mm · 沿线差 {error.AlongErrorMm:+0.00;-0.00;0.00} mm · 姿态差 {error.AngleErrorDeg:F2}°"
                    : null;
        }
    }
    public bool PollsFeedback { get { lock (gate) return owns && state != GroupState.FaultLatched; } }

    internal bool? RecordingComplete { get { lock (gate) return recording?.Complete; } }

    internal void StartRecording()
    {
        var capture = new CspTrace();
        lock (gate)
        {
            if (!transport.IsOpen) throw new InvalidOperationException("请先连接设备");
            if (recording is not null) throw new InvalidOperationException("请先保存上一次录制");
            if (state == GroupState.FaultLatched) throw new InvalidOperationException("请先退出 CSP 并确认设备状态");
            capture.Start(MotorFeedback.Now);
            recording = capture;
        }
    }

    internal CspTrace? TakeRecording()
    {
        lock (gate)
        {
            var capture = recording;
            capture?.Finish(MotorFeedback.Now, "manual_save");
            recording = null;
            return capture;
        }
    }

    public WholeBodyController(Robot model,Calibration c,ICanTransport bus,MotorRegistry motors,MotorController controller,bool startThread=true)
    {
        robot=model; calibration=Clone(c); transport=bus;registry=motors;legacy=controller;
        legacy.WholeBodyEmergency += EmergencyAndExit;
        thread=new Thread(Run){IsBackground=true,Name="WholeBodyCSP",Priority=ThreadPriority.AboveNormal};if(startThread)thread.Start();
    }
    static Calibration Clone(Calibration c){var copy=JsonSerializer.Deserialize<Calibration>(JsonSerializer.Serialize(c))!;copy.Signs=(int[])c.Signs.Clone();return copy;}
    public void ConnectionClosed()
    {
        lock(gate)
        {
            recording?.Finish(MotorFeedback.Now, "connection_closed");
            ReleaseControl();
            message = "连接已关闭，旧计划已撤销";
        }
    }
    public void SetCalibration(Calibration c) {lock(gate){if(owns && state!=GroupState.Holding)throw new InvalidOperationException("请等待运动结束");c.Validate(robot);calibration=Clone(c);plan=null;}}
    public static CanFrame CspFrame(int id,short raw)=>PdoBuilder.SingleAxis(id,new PdoCommand(true,true,false,3,raw),true);
    MotorFeedback[] Feedback(int age, bool preflight = false, bool requireCoherent = false)
    {
        if (!transport.IsOpen) throw new InvalidOperationException("CAN 已断开");
        var latest = ids.Select(id => transport.GetFeedback(id)
            ?? throw new FeedbackNotReadyException($"ID {id} 无反馈")).ToArray();
        // Sample the clock after reading the receive cache. A new packet can
        // arrive between entering this method and reading the last axis.
        long now = MotorFeedback.Now;
        parkingAxis = -1;
        for (int i = 0; i < latest.Length; i++)
        {
            var axis = latest[i];
            if (!axis.IsFirmware117 || axis.HasError || axis.FaultCode != 0)
                throw new InvalidOperationException($"ID {axis.DeviceId} 协议不符或驱动故障 0x{axis.FaultCode:X4}");
            long delay = now - axis.Timestamp;
            if (delay < 0 || delay > age)
                throw new FeedbackNotReadyException($"ID {axis.DeviceId} 反馈时间异常或陈旧（{delay} ms，限 {age} ms）");
            if (!preflight && state != GroupState.Arming && !CspReady(axis))
                throw new InvalidOperationException($"退出 CSP · {Describe(axis)}");
            double joint = calibration.ToJoint(i, axis.PositionRaw);
            var zone = calibration.WholeBodyZone(i, joint);
            if (zone == WholeBodyLimitZone.Alarm)
                throw new InvalidOperationException($"轴 {i + 1} / ID {axis.DeviceId} 到达限位报警线（标定内收 0.2°）");
            if (zone != WholeBodyLimitZone.Planning && owns && !preflight && Math.Abs((int)axis.VelocityRpm) <= 2 &&
                calibration.WholeBodyZone(i, calibration.ToJoint(i, lastRaw[i])) == WholeBodyLimitZone.Planning)
                zone = calibration.WholeBodyStationaryFeedbackZone(i, joint);
            if (zone != WholeBodyLimitZone.Planning && parkingAxis < 0) parkingAxis = i;
        }
        if (parkingAxis >= 0)
        {
            bool newlyLocked = owns && parkingLockAxis < 0;
            parkingLockAxis = parkingAxis;
            insidePlanningSince = 0;
            if (newlyLocked) Log?.Invoke($"全身运控 · 轴 {parkingAxis + 1} 进入停车区，锁定后续普通任务");
        }

        long skew = latest.Max(axis => axis.Timestamp) - latest.Min(axis => axis.Timestamp);
        if (skew <= 10)
            coherentFeedback = latest;
        else if (preflight || requireCoherent || coherentFeedback is null || now - coherentFeedback.Min(axis => axis.Timestamp) > age)
            throw new FeedbackNotReadyException($"四轴反馈尚未成组（偏斜 {skew} ms，限 10 ms）");

        var feedback = coherentFeedback!;
        calibration.Check(feedback.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray());
        return feedback;
    }

    static bool CspReady(MotorFeedback axis) => axis.Mode == 3 && axis.Enabled && axis.BrakeReleased;
    static string Describe(MotorFeedback axis) =>
        $"ID {axis.DeviceId} 模式={axis.Mode} 使能={axis.Enabled} 抱闸释放={axis.BrakeReleased} 故障=0x{axis.FaultCode:X4}";
    public void Arm(IReadOnlyDictionary<int,string> firmware)
    {
        lock(gate)
        {
            if(state==GroupState.FaultLatched)throw new InvalidOperationException("故障锁存，请先退出 CSP 后重新检查");
            if(owns)throw new InvalidOperationException("已占用四轴");
            calibration.Validate(robot);
            ids=registry.Motors.Select(m=>m.DeviceId).ToArray();
            if(registry.Motors.Any(m=>!m.UseFirmware117Pdo))throw new InvalidOperationException("请使用V1.17自定义PDO协议");
            var fb = Feedback(100, preflight: true);
            calibration.CheckWholeBodyStart(fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray());
            if(!legacy.TryAcquireWholeBody())throw new InvalidOperationException("请先结束关节页的运动或标定操作");
            owns=true;
            try
            {
                hold=fb.Select((x,i)=>calibration.ToJoint(i,x.PositionRaw)).ToArray();lastRaw=fb.Select(x=>x.PositionRaw).ToArray();
                previousRaw = lastRaw;
                lastTargetInterval = 0;
                int sent = transport.SendBatch(ids.Select((id, i) => CspFrame(id, lastRaw[i])).ToArray());
                if (sent != ids.Length) throw new InvalidOperationException($"首批保持帧仅发送 {sent}/{ids.Length}");
                lastTargetSentAt = MotorFeedback.Now;
                holdingStableSince = 0;
                parkingLockAxis = -1;
                insidePlanningSince = 0;
                state=GroupState.Arming;armedAt=MotorFeedback.Now;plan=null;elapsed=0;lateCount=0;maxInterval=0;message="等待四轴模式3、使能与抱闸反馈";
                feedbackPollTick = 0;
            }
            catch (Exception e) { Fail("CSP 接管发帧失败 · " + e.Message); throw; }
        }
    }
    public void Execute(MotionPlan next)
    {
        lock(gate)
        {
            if(state!=GroupState.Holding)throw new InvalidOperationException("四轴尚未处于稳定 CSP 保持状态");
            if(next.ModelHash!=robot.Hash||next.CalibrationHash!=calibration.Fingerprint)throw new InvalidOperationException("模型版本变化，请重新预览");
            var fb=Feedback(30, requireCoherent: true);
            var feedbackJoints = fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray();
            var start = next.Start;
            if (next.IsParkingRecovery)
            {
                if (!IsParkingRecoveryReady(MotorFeedback.Now) ||
                    !IsSettled(fb, lastRaw, allowParking: true) || !calibration.Encode(robot, start).SequenceEqual(lastRaw))
                    throw new InvalidOperationException("退让前需要四轴保持稳定、当前指令与起点一致");
                var end = next.End;
                for (int i = 0; i < fb.Length; i++)
                {
                    var zone = calibration.WholeBodyZone(i, feedbackJoints[i]);
                    if (zone == WholeBodyLimitZone.Planning) continue;
                    bool inward = zone == WholeBodyLimitZone.LowerParking ? end[i] > start[i] : end[i] < start[i];
                    var (low, high) = calibration.WholeBodyAlarmBounds(i);
                    double clearance = Math.Min(feedbackJoints[i] - low, high - feedbackJoints[i]);
                    double reserve = (4 * 360.0 / 65536) * Math.PI / 180 +
                        Math.Abs(MeasuredJointSpeed(i, fb[i])) * ParkingRecoveryResponseSeconds;
                    if (!inward || clearance <= reserve)
                        throw new InvalidOperationException($"轴 {i + 1} 退让方向或限位余量不满足要求");
                }
            }
            else
            {
                if (parkingLockAxis >= 0)
                    throw new InvalidOperationException($"轴 {parkingLockAxis + 1} 停车区锁定，回到规划区并停稳后再执行");
                calibration.CheckWholeBodyStart(start);
            }
            if (start.Where((x,i)=>Math.Abs(x-feedbackJoints[i])>0.01).Any() ||
                start.Where((x,i)=>Math.Abs(x-hold[i])>0.001).Any())
                throw new InvalidOperationException("起点变化，请从当前保持位置重新规划");
            if (next.ExpectedMoveLPose(0) is { } plannedStart &&
                (!DiskKinematics.Matches(plannedStart, feedbackJoints) ||
                 !DiskKinematics.Matches(plannedStart, hold)))
                throw new InvalidOperationException("实测或保持位姿已偏离 MoveL 起点，请等待稳定后重试");
            moveLDeviation = null;
            stoppedBeforeTarget = false;
            targetReached = false;
            Array.Clear(peakMotorRpm);
            peakTrackingErrorRaw = 0;
            peakTrackingAxis = 0;
            holdingStableSince = 0;
            plan=next;elapsed=0;state=GroupState.Executing;message=$"最长指令轨迹 {next.Duration:F2} s";
        }
    }
    public double[] PlanningStart() {lock(gate)return (double[])hold.Clone();}
    public void Stop()
    {
        lock (gate)
        {
            if (state != GroupState.Executing) return;
            state = GroupState.Stopping;
            stopStart = elapsed;
            stopElapsed = 0;
            stoppedBeforeTarget = true;
            message = "沿共同路径平滑减速";
        }
    }

    private bool IsSettled(MotorFeedback[] feedback, short[] target, bool allowParking = false)
    {
        for (int i = 0; i < feedback.Length; i++)
        {
            var axis = feedback[i];
            if (Math.Abs((int)axis.VelocityRpm) > 2) return false;
            int error = Math.Abs(axis.PositionRaw - target[i]);
            if (error <= SettledErrorRaw) continue;
            if (!allowParking || error > ParkingHoldErrorRaw) return false;
            var zone = calibration.WholeBodyZone(i, calibration.ToJoint(i, axis.PositionRaw));
            if (zone is not (WholeBodyLimitZone.LowerParking or WholeBodyLimitZone.UpperParking)) return false;
        }
        return true;
    }

    private double MeasuredJointSpeed(int axis, MotorFeedback feedback) =>
        calibration.ToJoint(axis, feedback.VelocityRpm) * 65536 / (60 * GearRatio);

    public void Emergency() {lock(gate){if(owns)Fail("软件急停：已撤销计划，尝试失能抱闸");}}

    private void EmergencyAndExit()
    {
        lock (gate)
        {
            if (!owns) return;
            try
            {
                BeginDisarming();
                message = "急停指令已发送，等待四轴静止、失能抱闸回读";
            }
            catch (Exception e)
            {
                Fail("急停 · " + e.Message);
            }
        }
    }

    public void StopAndExit()
    {
        lock (gate)
        {
            if (!owns || state == GroupState.Disarming) return;
            exitRequested = true;
            if (state == GroupState.FaultLatched)
            {
                try { BeginDisarming(); }
                catch (Exception e) { Fail(e.Message); }
            }
            else if (state == GroupState.Holding)
                BeginSettling(MotorFeedback.Now);
            else
                Stop();
        }
    }

    void BeginDisarming()
    {
        plan = null;
        if (!SendDisabled()) throw new InvalidOperationException("退出 CSP 时失能指令发送失败");
        disableSentAt = MotorFeedback.Now;
        feedbackPollTick = 0;
        state = GroupState.Disarming;
        message = "等待四轴回读失能、抱闸";
    }

    void ConfirmDisarmed(long now)
    {
        var feedback = ids.Select(transport.GetFeedback).ToArray();
        long readAt = MotorFeedback.Now;
        // Some drives keep reporting their last mode byte while disabled.
        // Confirm fresh drive status before releasing the host's CSP ownership.
        if (feedback.All(axis => axis is { IsFirmware117: true, Enabled: false, BrakeReleased: false }
            && Math.Abs((int)axis.VelocityRpm) <= 2
            && axis.Timestamp > disableSentAt && readAt - axis.Timestamp is >= 0 and <= 100))
        {
            ReleaseControl();
            message = "已失能抱闸并退出 CSP";
            recording?.Finish(readAt, "csp_disarmed");
            Log?.Invoke("全身运控 · " + message);
            return;
        }
        if (now - disableSentAt > 1000)
            throw new InvalidOperationException("退出 CSP 超时，未确认四轴静止、失能抱闸");
        PollFeedback();
    }

    void ReleaseControl()
    {
        plan = null;
        coherentFeedback = null;
        parkingAxis = -1;
        parkingLockAxis = -1;
        holdingStableSince = 0;
        insidePlanningSince = 0;
        stoppedBeforeTarget = false;
        targetReached = false;
        exitRequested = false;
        owns = false;
        legacy.ReleaseWholeBody();
        state = GroupState.Observing;
    }
    bool SendDisabled()
    {
        var frames = ids.Select(id => PdoBuilder.SingleAxis(id, PdoCommand.Disable, true)).ToArray();
        if (transport.SendBatch(frames) == frames.Length) return true;
        bool sent = true;
        foreach (var frame in frames) sent &= transport.Send(frame);
        return sent;
    }
    void Fail(string why)
    {
        targetReached = false;
        plan=null;exitRequested=false;state=GroupState.FaultLatched;message=why;
        if(owns&&!SendDisabled())message+="；失能发送失败，请使用硬件安全链";
        recording?.Finish(MotorFeedback.Now, "fault: " + message);
        Log?.Invoke("全身运控故障 · " + message);
    }

    internal void Tick(double dt,long now)
    {
        lock(gate)
        {
            recording?.CheckDuration(now);
            if(!owns||state==GroupState.FaultLatched)return;
            try
            {
                if (state == GroupState.Disarming) { ConfirmDisarmed(now); return; }
                maxInterval=Math.Max(maxInterval,dt*1000);
                if(dt>MotionPlan.CspMaxInterval||dt<=0||!double.IsFinite(dt))throw new InvalidOperationException("CSP 严重超期");
                lateCount=dt>0.008 ? lateCount+1:0;if(lateCount>=3)throw new InvalidOperationException("连续周期超期");
                var fb=Feedback(state==GroupState.Arming?100:30);
                if (recording is { Complete: false } capture)
                    capture.Add(new(MotorFeedback.Now, lastTargetSentAt, state, plan is null ? null : elapsed, dt,
                        lastRaw, previousRaw, lastTargetInterval, fb, calibration.Signs));
                if(state==GroupState.Arming)
                {
                    if(now-armedAt>1000)throw new InvalidOperationException("模式/使能确认超时 · " + string.Join("；", fb.Select(Describe)));
                    if (now - armedAt >= 150 && fb.All(f => CspReady(f) && f.Timestamp >= armedAt))
                    {
                        if (exitRequested) BeginSettling(now);
                        else { state = GroupState.Holding; message = "CSP 已确认；保持实测起点"; }
                    }
                }
                if (plan is not null)
                {
                    for (int i = 0; i < fb.Length; i++)
                    {
                        peakMotorRpm[i] = Math.Max(peakMotorRpm[i], Math.Abs((int)fb[i].VelocityRpm));
                        int error = Math.Abs(fb[i].PositionRaw - lastRaw[i]);
                        if (error <= peakTrackingErrorRaw) continue;
                        peakTrackingErrorRaw = error;
                        peakTrackingAxis = i;
                    }
                }
                if (state == GroupState.Holding)
                {
                    bool settled = IsSettled(fb, lastRaw);
                    bool stable = settled || (parkingAxis >= 0 && IsSettled(fb, lastRaw, allowParking: true));
                    if (!stable) holdingStableSince = 0;
                    else if (holdingStableSince == 0) holdingStableSince = now;
                    if (parkingLockAxis >= 0)
                    {
                        if (parkingAxis >= 0 || !settled) insidePlanningSince = 0;
                        else if (insidePlanningSince == 0) insidePlanningSince = now;
                        else if (now - insidePlanningSince >= 200)
                        {
                            parkingLockAxis = -1;
                            message = "已回到规划区并稳定，可重新规划";
                        }
                        if (parkingLockAxis >= 0)
                            message = parkingAxis >= 0 ? $"轴 {parkingAxis + 1} 位于停车区，" +
                                (IsParkingRecoveryReady(now) ? "可点击停车区退让" : "等待四轴停稳")
                                : "已回到规划区，等待反馈稳定";
                    }
                }
                if (plan?.ExpectedMoveLPose(elapsed) is not null)
                {
                    var actual = DiskKinematics.Forward(fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray());
                    moveLDeviation = plan.MoveLDeviation(elapsed, actual);
                }
                if (plan is not null)
                {
                    // A late tick may advance the common clock less than wall time.
                    // Discard the excess instead of catching up with a later position jump.
                    double advance = Math.Min(dt, plan.MaxCspAdvanceSeconds);
                    if (state == GroupState.Executing)
                    {
                        elapsed = Math.Min(plan.Duration, elapsed + advance);
                        hold = plan.At(elapsed);
                        if (elapsed >= plan.Duration) BeginSettling(now);
                    }
                    else if (state == GroupState.Stopping)
                    {
                        stopElapsed = Math.Min(MotionPlan.StopTime, stopElapsed + advance);
                        double u = stopElapsed / MotionPlan.StopTime;
                        elapsed = Math.Min(plan.Duration, stopStart + MotionPlan.StopTime * (u - u * u * u + 0.5 * u * u * u * u));
                        hold = plan.At(elapsed);
                        if (stopElapsed >= MotionPlan.StopTime || elapsed >= plan.Duration) BeginSettling(now);
                    }
                }
                var raw=calibration.Encode(robot,hold);
                if(raw.Where((r,i)=>Math.Abs(r-lastRaw[i])>MotionPlan.CspMaxStepRaw).Any())throw new InvalidOperationException("四轴目标步长超限");
                if(transport.SendBatch(ids.Select((id,i)=>CspFrame(id,raw[i])).ToArray())!=4)throw new InvalidOperationException("四轴部分发送失败");
                previousRaw = lastRaw;
                lastRaw=raw;
                lastTargetInterval = dt;
                lastTargetSentAt = MotorFeedback.Now;
                PollFeedback();
                if(state==GroupState.Settling)
                {
                    bool parked = parkingAxis >= 0;
                    bool good = IsSettled(fb, raw, allowParking: parked);
                    if (!good) settledAt = 0;
                    else if (settledAt == 0 || settlingInParking != parked) settledAt = now;
                    settlingInParking = parked;
                    if (settledAt != 0 && now - settledAt >= 200)
                    {
                        string result = "四轴已稳定 200 ms";
                        if (plan is { } finished)
                        {
                            bool completed = elapsed >= finished.Duration;
                            targetReached = completed && !parked && !finished.IsParkingRecovery;
                            stoppedBeforeTarget = !completed;
                            result = parked ? $"轴 {parkingAxis + 1} 已停稳在停车区，普通任务已锁定"
                                : !completed ? "已停稳，未到目标"
                                : finished.IsParkingRecovery ? "停车区退让完成" : "已到位";
                            if (Log is { } log)
                            {
                                var actual = DiskKinematics.Forward(fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray());
                                var peaks = peakMotorRpm.Select((rpm, i) => $"轴 {i + 1} {rpm / GearRatio:F2}");
                                string error = peakTrackingErrorRaw == 0 ? "0°"
                                    : $"轴 {peakTrackingAxis + 1} {Od.RawToAngle(peakTrackingErrorRaw):F3}°";
                                log($"全身运控 · {result} · 停稳高度 {actual.Z * 1000:F2} mm · 实测输出端峰值 rpm（{string.Join("，", peaks)}）"
                                    + $" · 跟踪峰值偏差 {error}");
                            }
                        }
                        if (exitRequested) BeginDisarming();
                        else
                        {
                            state = GroupState.Holding;
                            holdingStableSince = settledAt;
                            plan = null;
                            message = result;
                        }
                    }
                    else if(now-settleStarted>5000)throw new InvalidOperationException("四轴到位等待超时");
                }
            }
            catch(Exception e){Fail(e.Message);}
        }
    }
    void PollFeedback()
    {
        if ((feedbackPollTick++ & 1) == 0 && !transport.Send(PdoBuilder.Sync()))
            throw new InvalidOperationException("CSP 反馈查询发送失败");
    }
    void BeginSettling(long now)
    {
        state = GroupState.Settling;
        settledAt = 0;
        settlingInParking = false;
        settleStarted = now;
        message = plan is { } finished && elapsed >= finished.Duration
            ? "指令轨迹完成，等待四轴实际共同到位"
            : "停止指令完成，等待四轴停稳";
    }
    void Run()
    {
        long previous=Stopwatch.GetTimestamp(),deadline=previous;long period=Stopwatch.Frequency/200;
        while(!cancel.IsCancellationRequested)
        {
            deadline+=period;long now;
            while((now=Stopwatch.GetTimestamp())<deadline) {if(cancel.Token.WaitHandle.WaitOne(1))return;}
            Tick((now-previous)/(double)Stopwatch.Frequency,MotorFeedback.Now);previous=now;
            if(now-deadline>period)deadline=now;
        }
    }
    public void Dispose(){cancel.Cancel();if(thread.IsAlive)thread.Join();Emergency();legacy.WholeBodyEmergency-=EmergencyAndExit;cancel.Dispose();}
}
