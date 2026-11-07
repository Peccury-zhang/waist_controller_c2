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
    readonly object gate=new(); readonly ICanTransport transport; readonly MotorRegistry registry; readonly MotorController legacy; readonly Robot robot;
    readonly CancellationTokenSource cancel=new(); readonly Thread thread;
    Calibration calibration; int[] ids=[]; short[] lastRaw=new short[4]; double[] hold=new double[4]; MotionPlan? plan;
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
    public WholeBodyController(Robot model,Calibration c,ICanTransport bus,MotorRegistry motors,MotorController controller,bool startThread=true)
    {
        robot=model; calibration=Clone(c); transport=bus;registry=motors;legacy=controller;
        legacy.WholeBodyEmergency += EmergencyAndExit;
        thread=new Thread(Run){IsBackground=true,Name="WholeBodyCSP",Priority=ThreadPriority.AboveNormal};if(startThread)thread.Start();
    }
    static Calibration Clone(Calibration c){var copy=JsonSerializer.Deserialize<Calibration>(JsonSerializer.Serialize(c))!;copy.Signs=(int[])c.Signs.Clone();return copy;}
    public void ConnectionClosed()
    {
        lock(gate) { ReleaseControl();message="连接已关闭，旧计划已撤销"; }
    }
    public void SetCalibration(Calibration c) {lock(gate){if(owns && state!=GroupState.Holding)throw new InvalidOperationException("请等待运动结束");c.Validate(robot);calibration=Clone(c);plan=null;}}
    public static CanFrame CspFrame(int id,short raw)=>PdoBuilder.SingleAxis(id,new PdoCommand(true,true,false,3,raw),true);
    MotorFeedback[] Feedback(int age, bool preflight = false)
    {
        if (!transport.IsOpen) throw new InvalidOperationException("CAN 已断开");
        var latest = ids.Select(id => transport.GetFeedback(id)
            ?? throw new FeedbackNotReadyException($"ID {id} 无反馈")).ToArray();
        // Sample the clock after reading the receive cache. A new packet can
        // arrive between entering this method and reading the last axis.
        long now = MotorFeedback.Now;
        foreach (var axis in latest)
        {
            if (!axis.IsFirmware117 || axis.HasError || axis.FaultCode != 0)
                throw new InvalidOperationException($"ID {axis.DeviceId} 协议不符或驱动故障 0x{axis.FaultCode:X4}");
            long delay = now - axis.Timestamp;
            if (delay < 0 || delay > age)
                throw new FeedbackNotReadyException($"ID {axis.DeviceId} 反馈时间异常或陈旧（{delay} ms，限 {age} ms）");
            if (!preflight && state != GroupState.Arming && !CspReady(axis))
                throw new InvalidOperationException($"退出 CSP · {Describe(axis)}");
        }

        long skew = latest.Max(axis => axis.Timestamp) - latest.Min(axis => axis.Timestamp);
        if (skew <= 10)
            coherentFeedback = latest;
        else if (preflight || coherentFeedback is null || now - coherentFeedback.Min(axis => axis.Timestamp) > age)
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
            if(!legacy.TryAcquireWholeBody())throw new InvalidOperationException("请先结束关节页的运动或标定操作");
            owns=true;
            try
            {
                hold=fb.Select((x,i)=>calibration.ToJoint(i,x.PositionRaw)).ToArray();lastRaw=fb.Select(x=>x.PositionRaw).ToArray();
                int sent = transport.SendBatch(ids.Select((id, i) => CspFrame(id, lastRaw[i])).ToArray());
                if (sent != ids.Length) throw new InvalidOperationException($"首批保持帧仅发送 {sent}/{ids.Length}");
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
            var fb=Feedback(30);
            var feedbackJoints = fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray();
            var start = next.Start;
            if (start.Where((x,i)=>Math.Abs(x-feedbackJoints[i])>0.01).Any() ||
                start.Where((x,i)=>Math.Abs(x-hold[i])>0.001).Any())
                throw new InvalidOperationException("起点变化，请从当前保持位置重新规划");
            if (next.ExpectedMoveLPose(0) is { } plannedStart &&
                (!DiskKinematics.Matches(plannedStart, feedbackJoints) ||
                 !DiskKinematics.Matches(plannedStart, hold)))
                throw new InvalidOperationException("实测或保持位姿已偏离 MoveL 起点，请等待稳定后重试");
            moveLDeviation = null;
            plan=next;elapsed=0;state=GroupState.Executing;message=$"最长指令轨迹 {next.Duration:F2} s";
        }
    }
    public double[] PlanningStart() {lock(gate)return (double[])hold.Clone();}
    public void Stop()
    {
        lock(gate) { if(state==GroupState.Executing) {state=GroupState.Stopping;stopStart=elapsed;stopElapsed=0;message="沿共同路径平滑减速（2秒时间缩放）";} }
    }
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
        plan=null;exitRequested=false;state=GroupState.FaultLatched;message=why;
        if(owns&&!SendDisabled())message+="；失能发送失败，请使用硬件安全链";
        Log?.Invoke("全身运控故障 · " + message);
    }
    internal void Tick(double dt,long now)
    {
        lock(gate)
        {
            if(!owns||state==GroupState.FaultLatched)return;
            try
            {
                if (state == GroupState.Disarming) { ConfirmDisarmed(now); return; }
                maxInterval=Math.Max(maxInterval,dt*1000);
                if(dt>MotionPlan.CspMaxInterval||dt<=0||!double.IsFinite(dt))throw new InvalidOperationException("CSP 严重超期");
                lateCount=dt>0.008 ? lateCount+1:0;if(lateCount>=3)throw new InvalidOperationException("连续周期超期");
                var fb=Feedback(state==GroupState.Arming?100:30);
                if(state==GroupState.Arming)
                {
                    if(now-armedAt>1000)throw new InvalidOperationException("模式/使能确认超时 · " + string.Join("；", fb.Select(Describe)));
                    if (now - armedAt >= 150 && fb.All(f => CspReady(f) && f.Timestamp >= armedAt))
                    {
                        if (exitRequested) BeginSettling(now);
                        else { state = GroupState.Holding; message = "CSP 已确认；保持实测起点"; }
                    }
                }
                if(fb.Where((f,i)=>Math.Abs(f.PositionRaw-lastRaw[i])>364).Any())throw new InvalidOperationException("跟踪误差超过 2°，整组停止");
                if (plan?.ExpectedMoveLPose(elapsed) is not null)
                {
                    var actual = DiskKinematics.Forward(fb.Select((axis, i) => calibration.ToJoint(i, axis.PositionRaw)).ToArray());
                    moveLDeviation = plan.MoveLDeviation(elapsed, actual);
                }
                if(state==GroupState.Executing&&plan!=null) {elapsed+=dt;hold=plan.At(elapsed);if(elapsed>=plan.Duration)BeginSettling(now);}
                else if(state==GroupState.Stopping&&plan!=null)
                {
                    stopElapsed=Math.Min(MotionPlan.StopTime,stopElapsed+dt);double u=stopElapsed/MotionPlan.StopTime;
                    elapsed=stopStart+MotionPlan.StopTime*(u-u*u*u+0.5*u*u*u*u);hold=plan.At(elapsed);
                    if(stopElapsed>=MotionPlan.StopTime||elapsed>=plan.Duration)BeginSettling(now);
                }
                var raw=calibration.Encode(robot,hold);
                if(raw.Where((r,i)=>Math.Abs(r-lastRaw[i])>MotionPlan.CspMaxStepRaw).Any())throw new InvalidOperationException("四轴目标步长超限");
                if(transport.SendBatch(ids.Select((id,i)=>CspFrame(id,raw[i])).ToArray())!=4)throw new InvalidOperationException("四轴部分发送失败");
                lastRaw=raw;
                PollFeedback();
                if(state==GroupState.Settling)
                {
                    bool good=fb.Select((f,i)=>Math.Abs(f.PositionRaw-raw[i])<=20&&Math.Abs(f.VelocityRpm)<=2).All(x=>x);
                    if(!good)settledAt=0;else if(settledAt==0)settledAt=now;
                    if (settledAt != 0 && now - settledAt >= 200)
                    {
                        if (exitRequested) BeginDisarming();
                        else { state = GroupState.Holding; plan = null; message = "四轴均已到位并稳定 200 ms"; }
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
    void BeginSettling(long now){state=GroupState.Settling;settledAt=0;settleStarted=now;message="指令轨迹完成，等待四轴实际共同到位";}
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
