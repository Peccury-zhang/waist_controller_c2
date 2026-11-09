using WaistModel;
using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;
using waist_motor_controller.WholeBody;

internal static class WholeBodyTests
{
    static int count;
    static void Check(bool condition,string name){if(!condition)throw new Exception("WBC: "+name);count++;}
    static void Reject(Action action,string name){bool rejected=false;try{action();}catch(ArgumentException){rejected=true;}catch(InvalidOperationException){rejected=true;}catch(InvalidDataException){rejected=true;}Check(rejected,name);}
    public static void Run()
    {
        string root=Path.Combine(AppContext.BaseDirectory,"assets","robot");var r=Robot.Load(Path.Combine(root,"urdf","urdf_g3_06.urdf"),Path.Combine(root,"meshes"));
        Check(r.Links.Count==16&&r.Joints.Count==15,"pruned tree");
        var c=new Calibration{Motors=new[]{new MotorLimit(1,-170,170),new MotorLimit(2,-150,150),new MotorLimit(3,-170,170),new MotorLimit(4,-100,100)}};c.Validate();
        var zero=DiskKinematics.Forward(new double[4]);Check(Math.Abs(zero.Z-.972)<1e-8&&Math.Abs(zero.X-.066)<1e-8,"disk reference not C3");
        CheckTopMotorDirection(r, c);
        CheckParkingRecovery(r);
        CheckCspLateAdvance(r, c);
        CheckParkingInterlockDuringMotion(r);
        CheckStraightPoseMoves(r);
        double[] outsideUrdf=[0,1.0,-.6,0];c.Check(outsideUrdf,false);Check(true,"calibration overrides URDF limits");
        foreach(double bad in new[]{.351,-.501,double.NaN})Reject(()=>c.CheckX(bad),"task bound");
        var rng=new Random(42);
        for(int k=0;k<100;k++)
        {
            double[] q=[(rng.NextDouble()-.5)*1.2,-.3,rng.NextDouble()*.4+.2,-.2];var target=DiskKinematics.Forward(q);var result=DiskKinematics.Solve(c,target,q);Check(DiskKinematics.Matches(target,result),"analytic FK round trip");
            var raw=c.Encode(r,q);Check(raw.Select((x,i)=>Math.Abs(c.ToJoint(i,x)-q[i])).Max()<.0001,"encoder round trip");
        }
        var q0=new double[]{.1,-.2,.4,-.2};var pose=DiskKinematics.Forward(q0);
        // Warm library JIT separately from the operational deadline.
        try{DiskKinematics.Numerical(c,pose,q0.Select(x=>x+.001).ToArray());}catch(TimeoutException){}
        var numerical=DiskKinematics.Numerical(c,pose,q0.Select(x=>x+.001).ToArray());Check(DiskKinematics.Matches(pose,numerical),"numerical fallback");
        var targetPose=pose with {Z=pose.Z-.005};var plan=MotionPlan.Create(r,c,q0,targetPose);
        Check(plan.At(0).Zip(q0).All(p=>Math.Abs(p.First-p.Second)<1e-10),"start continuity");Check(DiskKinematics.Matches(targetPose,plan.At(plan.Duration)),"endpoint");
        for(int i=0;i<=1000;i++){var q=plan.At(plan.Duration*i/1000);c.Encode(r,q);var p=DiskKinematics.Forward(q);Check(Math.Abs(p.X-pose.X)<.0001&&Math.Abs(p.Pitch-pose.Pitch)<.00018&&Math.Abs(DiskKinematics.Wrap(p.Yaw-pose.Yaw))<.00018,"height jog preserves other tasks");}
        string dir=Path.Combine(AppContext.BaseDirectory,"wbc-tests-"+Guid.NewGuid());Directory.CreateDirectory(dir);var store=new CalibrationStore(Path.Combine(dir,"calibration.json"));
        Reject(()=>store.Load(c.Signs),"missing calibration");store.Save(c);var loaded=store.Load(c.Signs);Check(loaded.Fingerprint==c.Fingerprint,"JSON round trip");c.TaskLimits.ForwardLimit=.3;store.Save(c);Check(Directory.GetFiles(dir).Length==1&&store.Load(c.Signs).TaskLimits.ForwardLimit==.3,"single atomic overwrite");File.Delete(store.FilePath);Directory.Delete(dir);
        Check(WholeBodyController.CspFrame(3,-50).Data.SequenceEqual(new byte[]{0xC6,0xFF,0xCE,0,0,0,0}),"CSP golden");
        var bus=new FakeBus();var registry=new MotorRegistry(Path.Combine(Path.GetTempPath(),Guid.NewGuid()+".json"));var ka=new KeepaliveScheduler(bus.Send);var legacy=new MotorController(bus,ka,registry);
        using var wbc=new WholeBodyController(r,new Calibration(),bus,registry,legacy,false);Reject(()=>wbc.Arm(new Dictionary<int,string>()),"missing calibration blocks");Check(bus.Frames.Count==0,"no frames before valid calibration");
        wbc.SetCalibration(c);
        bus.Refresh(false, skew: 40);
        Reject(() => wbc.Arm(new Dictionary<int, string>()), "mixed preflight feedback rejects without handover");
        Check(!legacy.WholeBodyOwned && bus.Frames.Count == 0, "preflight rejection sends no enable or disable");
        bus.Refresh(false);
        wbc.Arm(new Dictionary<int,string>());Check(legacy.WholeBodyOwned,"exclusive owner");Check(!legacy.Enable(3,1,1).Ok,"legacy blocked");Thread.Sleep(160);bus.Refresh(true);wbc.Tick(.005,MotorFeedback.Now);Check(wbc.State==GroupState.Holding,"CSP confirmation");
        Check(bus.Frames.Any(frame => frame.Id == Od.SyncId), "CSP sends its own feedback poll");
        bus.Refresh(true, skew: 12);
        wbc.Tick(.005, MotorFeedback.Now);
        Check(wbc.State == GroupState.Holding, "mixed feedback uses the still fresh coherent group");
        var start = new double[4];
        var motion = MotionPlan.CreateIndependent(r, c, start, DiskKinematics.Forward(start) with { Yaw = .1 }, 1, 1);
        bus.Refresh(true);
        wbc.Execute(motion);
        wbc.Tick(.005, MotorFeedback.Now);
        int emergencyStart = bus.Frames.Count;
        Check(legacy.EmergencyStop().Ok && wbc.State == GroupState.Disarming,
            "shared emergency handler immediately interrupts an active CSP motion");
        var stopFrames = bus.Frames.Skip(emergencyStart).ToArray();
        Check(registry.Motors.All(axis => stopFrames.Any(frame =>
            frame.Id == Od.PdoBase + (uint)axis.DeviceId && frame.Data.All(value => value == 0))),
            "emergency sends disable and brake commands to all four mapped axes");
        int afterEmergency = bus.Frames.Count;
        wbc.Tick(.005, MotorFeedback.Now);
        Check(bus.Frames.Skip(afterEmergency).All(frame => frame.Id == Od.SyncId),
            "emergency stops CSP targets while waiting for status");
        Check(legacy.WholeBodyOwned, "emergency keeps ownership until all axes confirm");
        wbc.Tick(.005, MotorFeedback.Now);
        Thread.Sleep(2);
        bus.Refresh(false, movingAxis: 1);
        wbc.Tick(.005, MotorFeedback.Now);
        Check(wbc.State == GroupState.Disarming && legacy.WholeBodyOwned, "one still-moving axis blocks release");
        bus.Refresh(false);
        wbc.Tick(.005, MotorFeedback.Now);
        Check(wbc.State == GroupState.Observing && !legacy.WholeBodyOwned,
            "emergency exits automatically on fresh, stationary disabled feedback even with remembered mode 3");
        wbc.Arm(new Dictionary<int, string>());
        bus.Refresh(false);
        legacy.EmergencyStop();
        wbc.Tick(.005, MotorFeedback.Now + 1001);
        Check(wbc.State == GroupState.FaultLatched && legacy.WholeBodyOwned, "missing new feedback after emergency locks out further movement");
        legacy.EmergencyStop();
        Thread.Sleep(2);
        bus.Refresh(false);
        wbc.Tick(.005, MotorFeedback.Now);
        Check(wbc.State == GroupState.Observing && !legacy.WholeBodyOwned, "emergency can retry a failed exit confirmation");
        wbc.Arm(new Dictionary<int, string>());
        Thread.Sleep(160);
        bus.Refresh(true);
        wbc.Tick(.005, MotorFeedback.Now);
        wbc.StartRecording();
        wbc.Execute(motion);
        wbc.Tick(.005, MotorFeedback.Now);
        foreach (int error in new[] { 274, 365, 730 })
        {
            int first = bus.Frames.Count;
            bus.Refresh(true, offsetAxis: 1, offsetRaw: error);
            wbc.Tick(.005, MotorFeedback.Now);
            Check(wbc.State == GroupState.Executing &&
                bus.Frames.Skip(first).Count(frame => frame.Length == 7 && frame.Data[0] == 0xC6) == 4,
                "tracking error remains telemetry and all four commands continue");
        }
        Check(wbc.RecordingComplete == false, "tracking error does not stop recording");
        int watchdogTrip = bus.Frames.Count;
        wbc.Tick(.021, MotorFeedback.Now);
        Check(wbc.State == GroupState.FaultLatched && bus.Frames.Skip(watchdogTrip).Count() == 4 &&
            bus.Frames.Skip(watchdogTrip).All(frame => frame.Data.All(value => value == 0)),
            "the CSP watchdog still disables all axes before any further setpoint");
        Check(wbc.RecordingComplete == true, "a watchdog fault completes the recording for export");
        var capture = wbc.TakeRecording()!;
        string traceDir = Path.Combine(Path.GetTempPath(), "wbc-csp-" + Guid.NewGuid().ToString("N"));
        string csv = capture.Save(traceDir);
        try
        {
            var lines = File.ReadAllLines(csv);
            var sample = lines[^4].Split(',');
            Check(lines.Length == 4 * 4 + 2 && lines[0].Contains("feedback_received_ms") &&
                sample[1] == "Executing" && sample[5] == "4" &&
                int.Parse(sample[10]) - int.Parse(sample[9]) == 730 &&
                lines[^1].Contains("fault: CSP 严重超期"),
                "recording retains the large tracking error and the separate watchdog fault");
        }
        finally
        {
            File.Delete(csv);
            Directory.Delete(traceDir);
        }
        legacy.EmergencyStop();
        Thread.Sleep(2);
        bus.Refresh(false);
        wbc.Tick(.005, MotorFeedback.Now);
        wbc.Arm(new Dictionary<int,string>());bus.FailBatch=true;wbc.Tick(.005,MotorFeedback.Now);Check(wbc.State==GroupState.FaultLatched,"partial batch faults group");
        Check(bus.Frames.TakeLast(4).Select(frame => frame.Id).SequenceEqual(registry.Motors.Select(axis => Od.PdoBase + (uint)axis.DeviceId))
            && bus.Frames.TakeLast(4).All(frame => frame.Data[0] == 0), "partial disable batch is retried for all four axes");
        Console.WriteLine($"PASS WBC v2: {count} assertions; simulated transport only.");
    }

    static void CheckParkingRecovery(Robot robot)
    {
        var calibration = new Calibration
        {
            Motors = [new(1, -172, 172), new(2, -100, 15), new(3, -10, 143), new(4, -54, 10)],
        };
        const double toRadians = Math.PI / 180;
        Check(calibration.WholeBodyZone(3, -53.6 * toRadians) == WholeBodyLimitZone.Planning &&
            calibration.WholeBodyZone(3, -53.7012 * toRadians) == WholeBodyLimitZone.LowerParking &&
            calibration.WholeBodyZone(3, -53.8 * toRadians) == WholeBodyLimitZone.Alarm &&
            calibration.WholeBodyZone(3, 9.7 * toRadians) == WholeBodyLimitZone.UpperParking &&
            calibration.WholeBodyZone(3, 9.8 * toRadians) == WholeBodyLimitZone.Alarm,
            "both sides separate planning, parking and alarm at the specified thresholds");
        var bus = new FakeBus();
        bus.SetPosition([0, 7296, 17054, 9757]);
        bus.Refresh(false);
        var registry = new MotorRegistry(Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json"));
        var legacy = new MotorController(bus, new KeepaliveScheduler(bus.Send), registry);
        using var controller = new WholeBodyController(robot, calibration, bus, registry, legacy, false);
        controller.Arm(new Dictionary<int, string>());
        Thread.Sleep(160);
        long jitterNow = MotorFeedback.Now;
        for (int i = 0; i < 60; i++)
        {
            bus.Refresh(true, offsetAxis: 3, offsetRaw: i % 2);
            controller.Tick(.005, jitterNow += 5);
            Check(controller.State == GroupState.Holding && !controller.InParkingZone && !controller.CanRecoverParking,
                "one-count stationary jitter at the recorded boundary never locks the controls");
        }
        bus.Refresh(true, offsetAxis: 3, offsetRaw: 19);
        controller.Tick(.005, MotorFeedback.Now);
        Check(controller.State == GroupState.Holding && controller.InParkingZone && !controller.CanRecoverParking,
            "the recorded -53.7012 degree case holds without an alarm and waits for stability before enabling recovery");
        var start = controller.PlanningStart();
        var measured = (double[])start.Clone();
        measured[3] = calibration.ToJoint(3, 9787);
        var inward = (double[])start.Clone();
        inward[3] += toRadians;
        var ordinary = MotionPlan.CreateMoveL(robot, calibration, start, DiskKinematics.Forward(inward), 1.4, 30);
        var recovery = MotionPlan.CreateParkingRecovery(robot, calibration, start, measured);
        Reject(() => controller.Execute(ordinary), "parking interlock rejects a normal motion with an inward goal");
        Reject(() => controller.Execute(recovery), "parking recovery waits for a stable hold");
        Thread.Sleep(205);
        bus.Refresh(true, offsetAxis: 3, offsetRaw: 30);
        controller.Tick(.005, MotorFeedback.Now);
        Check(controller.CanRecoverParking, "the parking recovery button becomes available after the stable hold");
        controller.Execute(recovery);
        int first = bus.Frames.Count;
        long now = MotorFeedback.Now;
        for (int i = 0; i < 800 && (controller.State != GroupState.Holding || controller.InParkingZone); i++)
        {
            bus.Refresh(true);
            controller.Tick(.005, now += 5);
        }
        Check(controller.State == GroupState.Holding && !controller.InParkingZone,
            "retreat settles back inside the planning range and releases the interlock");
        var sent = bus.Frames.Skip(first).Where(frame => frame.Id == 0x106 && frame.Data[0] == 0xC6)
            .Select(frame => unchecked((short)((frame.Data[1] << 8) | frame.Data[2]))).ToArray();
        Check(sent.Length > 1 && sent[0] == 9757 && sent.Zip(sent.Skip(1)).All(pair => pair.Second <= pair.First),
            "retreat preserves the first setpoint and moves the affected axis inwards");
        int currentRaw = bus.GetFeedback(6)!.PositionRaw;
        int alarmStart = bus.Frames.Count;
        bus.Refresh(true, skew: 12, offsetAxis: 3, offsetRaw: 9794 - currentRaw);
        controller.Tick(.005, MotorFeedback.Now);
        Check(controller.State == GroupState.FaultLatched && controller.Status.Contains("限位报警线") &&
            bus.Frames.Skip(alarmStart).Count() == 4 &&
            bus.Frames.Skip(alarmStart).All(frame => frame.Data.All(value => value == 0)),
            "the first alarm count trips before the cached feedback fallback and disables all axes");
    }

    static void CheckCspLateAdvance(Robot robot, Calibration calibration)
    {
        var bus = new FakeBus();
        var registry = new MotorRegistry(Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json"));
        var legacy = new MotorController(bus, new KeepaliveScheduler(bus.Send), registry);
        using var controller = new WholeBodyController(robot, calibration, bus, registry, legacy, false);
        controller.Arm(new Dictionary<int, string>());
        Thread.Sleep(160);
        long now = MotorFeedback.Now;
        bus.Refresh(true);
        controller.Tick(.005, now);
        var start = controller.PlanningStart();
        var target = DiskKinematics.Forward(start) with { Yaw = 1.2 };
        var plan = MotionPlan.CreateMoveL(robot, calibration, start, target, 11, 30);
        bus.Refresh(true);
        controller.Execute(plan);
        Check(plan.MaxCspAdvanceSeconds is > .007 and < .008, "11 rpm leaves about 7.3 ms of safe trajectory advance");

        double elapsed = 0;
        void Tick(double dt)
        {
            bus.Refresh(true);
            controller.Tick(dt, now += (long)Math.Round(dt * 1000));
        }
        while (elapsed < .48 * plan.Duration)
        {
            Tick(.005);
            elapsed += .005;
        }
        short[] SentTargets() => bus.Frames.Where(frame => frame.Data.Length == 7 && frame.Data[0] == 0xC6)
            .TakeLast(4).Select(frame => unchecked((short)((frame.Data[1] << 8) | frame.Data[2]))).ToArray();
        var before = SentTargets();
        Tick(.019);
        var after = SentTargets();
        elapsed += plan.MaxCspAdvanceSeconds;
        Check(controller.State == GroupState.Executing &&
            before.Zip(after).All(pair => Math.Abs(pair.First - pair.Second) <= MotionPlan.CspMaxStepRaw) &&
            after.SequenceEqual(calibration.Encode(robot, plan.At(elapsed))),
            "a late cycle limits shared progress and preserves all four raw step guards");
        Tick(.005);
        elapsed += .005;
        Check(SentTargets().SequenceEqual(calibration.Encode(robot, plan.At(elapsed))),
            "the next cycle resumes normally without catching up the delayed time");
        controller.Stop();
        Tick(.019);
        double u = plan.MaxCspAdvanceSeconds / MotionPlan.StopTime;
        double stoppedAt = elapsed + MotionPlan.StopTime * (u - u * u * u + .5 * u * u * u * u);
        Check(controller.State == GroupState.Stopping && SentTargets().SequenceEqual(calibration.Encode(robot, plan.At(stoppedAt))),
            "the same late-cycle bound applies during a stop on the planned path");
        Tick(.021);
        Check(controller.State == GroupState.FaultLatched && controller.Status.Contains("严重超期"),
            "a cycle beyond the hard watchdog deadline still faults");
    }

    static void CheckParkingInterlockDuringMotion(Robot robot)
    {
        var calibration = new Calibration
        {
            Motors = [new(1, -172, 172), new(2, -100, 15), new(3, -10, 143), new(4, -54, 10)],
        };
        var bus = new FakeBus();
        var registry = new MotorRegistry(Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json"));
        var legacy = new MotorController(bus, new KeepaliveScheduler(bus.Send), registry);
        using var controller = new WholeBodyController(robot, calibration, bus, registry, legacy, false);
        var log = new List<string>();
        controller.Log += log.Add;
        short[] raw = [0, 7296, 17054, 9575];
        var start = raw.Select((value, i) => calibration.ToJoint(i, value)).ToArray();
        var end = (double[])start.Clone();
        end[3] = calibration.ToJoint(3, 9756);
        var target = DiskKinematics.Forward(end);
        var plan = MotionPlan.CreateMoveL(robot, calibration, start, target, .1, 5);
        bus.SetPosition(raw);
        bus.Refresh(false);
        controller.Arm(new Dictionary<int, string>());
        Thread.Sleep(160);
        long now = MotorFeedback.Now;
        void Tick(int error = 0, bool moving = false)
        {
            bus.Refresh(true, offsetAxis: 3, offsetRaw: error, movingAxis: moving ? 3 : -1);
            controller.Tick(.005, now += 5);
        }
        void WaitForHolding()
        {
            for (int i = 0; i < 2000 && controller.State is not (GroupState.Holding or GroupState.FaultLatched); i++)
                Tick();
        }
        Tick();
        controller.Execute(plan);
        Tick(9776 - raw[3], moving: true);
        Check(controller.State == GroupState.Executing && controller.InParkingZone &&
            controller.PlanningStart().SequenceEqual(plan.At(.005)),
            "entering the parking zone while moving locks new requests and does not change the active progress");
        for (int i = 1; i < (int)Math.Ceiling(plan.Duration / .005) + 2 && controller.State == GroupState.Executing; i++)
            Tick();
        Check(controller.State == GroupState.Settling && controller.PlanningStart().SequenceEqual(plan.At(plan.Duration)),
            "the active trajectory finishes at the original target on schedule");
        for (int i = 0; i < 50 && controller.State == GroupState.Settling; i++)
            Tick(31);
        Check(controller.MotionStatus == (GroupState.Holding, false, false) && controller.InParkingZone &&
            controller.Status.Contains("停车区"),
            "an axis stopped in the parking zone can hold even beyond the normal arrival tolerance");
        Reject(() => controller.Execute(plan), "a parked group rejects a new ordinary trajectory");
        var sent = bus.Frames.Where(frame => frame.Id == 0x106 && frame.Data[0] == 0xC6)
            .Select(frame => unchecked((short)((frame.Data[1] << 8) | frame.Data[2])));
        Check(sent.All(value => calibration.WholeBodyZone(3, calibration.ToJoint(3, value)) == WholeBodyLimitZone.Planning),
            "no normal command enters the measured parking zone");
        Check(log.Any(line => line.Contains("实测输出端峰值") && line.Contains($"轴 4 {10.0 / 161:F2}") &&
            line.Contains($"跟踪峰值偏差 轴 4 {Od.RawToAngle(9776 - raw[3]):F3}°")),
            "the parking hold log reports the speed and tracking peaks");
        for (int i = 0; i < 50 && controller.InParkingZone; i++) Tick();
        Check(!controller.InParkingZone, "returning to the planning zone and settling releases the interlock");
        var retry = MotionPlan.CreateMoveL(robot, calibration, controller.PlanningStart(), DiskKinematics.Forward(start), .1, 5);
        bus.Refresh(true);
        controller.Execute(retry);
        WaitForHolding();
        Check(controller.MotionStatus == (GroupState.Holding, true, false),
            "a later completed trajectory reports arrival");

        short[] recordedStart = [0, 7271, 17026, 9757];
        var measured = recordedStart.Select((value, i) => calibration.ToJoint(i, value)).ToArray();
        var lift = MotionPlan.CreateMoveL(robot, calibration, measured,
            DiskKinematics.Forward(measured) with { X = 0, Z = .86924 }, 5, 62, priority: PositionPriority.Height);
        Check(lift.Duration < 3.4 && lift.MoveLPlannedPeakRpm > 3,
            "a legal move away from the limit is timed by joint rates rather than the old parking envelope");
    }

    static void CheckTopMotorDirection(Robot robot, Calibration reference)
    {
        var calibration = new Calibration { Motors = (MotorLimit[])reference.Motors.Clone() };
        calibration.Motors[0] = new MotorLimit(1, -5, 25);
        calibration.Validate();
        // With Direction=-1, an encoder change of -10° is +10° in joint control.
        const short raw = -1820;
        double[] q = [calibration.ToJoint(0, raw), 0, 0, 0];
        double heading = DiskKinematics.Forward(q).Yaw * 180 / Math.PI;
        Check(Math.Abs(heading - 10) < .01, "positive top motor display angle turns heading toward +Y");
        var disk = robot.Forward(q)["bady_link_4"];
        var forward = System.Numerics.Vector3.TransformNormal(System.Numerics.Vector3.UnitX, disk);
        Check(Math.Abs(Math.Atan2(forward.Y, forward.X) * 180 / Math.PI - heading) < .01,
            "URDF visual and task heading agree for CAN ID 3");
        Check(calibration.Encode(robot, q)[0] == raw, "CSP sends the matching encoder count");
        Check(calibration.RawLimits(robot, 0) == (-4551, 910), "asymmetric saved display limits preserve raw bounds");
        q[0] = -q[0];
        Reject(() => calibration.Encode(robot, q), "negative display angle outside asymmetric limit");
    }

    static void CheckStraightPoseMoves(Robot robot)
    {
        var calibration = new Calibration
        {
            Motors = [new(1, -172, 172), new(2, -100, 15), new(3, -10, 143), new(4, -51.6, 10)],
        };
        double[] start = [0, 0, 0, 0];
        var upright = DiskKinematics.Forward(start);
        CheckIndependentAxes(robot, calibration, start, upright);
        CheckPositionPriority(robot, calibration);
        CheckMoveL(robot, calibration, upright);
        Reject(() => MotionPlan.Create(robot, calibration, start, upright with { Z = upright.Z + .001 }),
            "raising past full extension remains blocked");
        double[] outside = [0, 0, calibration.ToJoint(2, -2003), 0];
        Reject(() => MotionPlan.Create(robot, calibration, outside, upright with { Z = upright.Z - .001 }),
            "motion from CAN ID 5 beyond the saved angle limit remains blocked");
        foreach (var target in new[] { upright with { Z = upright.Z - .001 }, new TaskPose(.066, .963, 0, -10.5 * Math.PI / 180) })
        {
            var plan = MotionPlan.Create(robot, calibration, start, target);
            Check(DiskKinematics.Matches(target, plan.At(plan.Duration)), "plan leaves a straight pose at the angle limit");
            for (int i = 0; i <= 1000; i++)
                calibration.Encode(robot, plan.At(plan.Duration * i / 1000));
            Check(true, "sampled straight-pose path respects saved encoder limits");
        }

        var bus = new FakeBus();
        var registry = new MotorRegistry(Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json"));
        var legacy = new MotorController(bus, new KeepaliveScheduler(bus.Send), registry);
        using var wbc = new WholeBodyController(robot, calibration, bus, registry, legacy, false);
        wbc.Arm(new Dictionary<int, string>());
        bus.Refresh(true);
        long now = MotorFeedback.Now + 160;
        wbc.Tick(.005, now);
        foreach (double descent in new[] { .001, .010 })
        {
            var measured = registry.Motors.Select((axis, i) =>
                calibration.ToJoint(i, bus.GetFeedback(axis.DeviceId)!.PositionRaw)).ToArray();
            var pose = DiskKinematics.Forward(measured);
            var plan = MotionPlan.CreateMoveL(robot, calibration, measured, pose with { Z = pose.Z - descent },
                1, 1, priority: PositionPriority.Height);
            bus.Refresh(true);
            wbc.Execute(plan);
            int ticks = (int)Math.Ceiling((plan.Duration + .5) / .005);
            for (int i = 0; i < ticks && wbc.State is GroupState.Executing or GroupState.Settling; i++)
            {
                bus.Refresh(true);
                wbc.Tick(.005, now += 5);
            }
            Check(wbc.State == GroupState.Holding, "successive downward CSP jog settles: " + wbc.Status);
        }

        bus.Refresh(true);
        var last = registry.Motors.Select((axis, i) =>
            calibration.ToJoint(i, bus.GetFeedback(axis.DeviceId)!.PositionRaw)).ToArray();
        var zeroPlan = MotionPlan.CreateIndependentToZero(robot, calibration, last, 1, 1);
        int zeroStart = bus.Frames.Count;
        bus.Refresh(true);
        wbc.Execute(zeroPlan);
        int zeroTicks = (int)Math.Ceiling((zeroPlan.Duration + .5) / .005);
        for (int i = 0; i < zeroTicks && wbc.State is GroupState.Executing or GroupState.Settling; i++)
        {
            bus.Refresh(true);
            wbc.Tick(.005, now += 5);
        }
        Check(wbc.State == GroupState.Holding && legacy.WholeBodyOwned,
            "CSP zero return settles without releasing ownership");
        Check(registry.Motors.All(axis => bus.GetFeedback(axis.DeviceId)!.PositionRaw == 0),
            "CSP zero return reaches exactly four zero encoder targets");
        Check(bus.Frames.Skip(zeroStart).Where(frame => frame.Length == 7).All(frame => frame.Data[0] == 0xC6),
            "CSP zero return sends only enabled CSP position frames");

        var zeroForbidden = new Calibration
        {
            Motors = [new(1, 1, 20), calibration.Motors[1], calibration.Motors[2], calibration.Motors[3]],
        };
        double[] inBounds = [-.1, 0, 0, 0];
        Reject(() => MotionPlan.CreateIndependentToZero(robot, zeroForbidden, inBounds, 1, 1),
            "CSP zero return rejects a zero target outside calibration");

        bus.Refresh(true);
        last = registry.Motors.Select((axis, i) =>
            calibration.ToJoint(i, bus.GetFeedback(axis.DeviceId)!.PositionRaw)).ToArray();
        var next = MotionPlan.CreateMoveL(robot, calibration, last,
            DiskKinematics.Forward(last) with { Z = DiskKinematics.Forward(last).Z - .005 }, 2.2, 2.2);
        bus.Refresh(true);
        wbc.Execute(next);
        for (int i = 0; i < 40; i++)
        {
            bus.Refresh(true);
            wbc.Tick(.005, now += 5);
        }
        int stopIndex = bus.Frames.Count;
        wbc.StopAndExit();
        Check(wbc.State == GroupState.Stopping && bus.Frames.Count == stopIndex,
            "normal stop does not disable a moving group");
        for (int i = 0; i < 600 && wbc.State is GroupState.Stopping or GroupState.Settling; i++)
        {
            bus.Refresh(true);
            wbc.Tick(.005, now += 5);
        }
        Check(wbc.State == GroupState.Disarming && legacy.WholeBodyOwned,
            "normal stop decelerates and settles before disabling, keeping ownership");
        Thread.Sleep(2);
        bus.Refresh(false);
        wbc.Tick(.005, MotorFeedback.Now);
        Check(wbc.State == GroupState.Observing && !legacy.WholeBodyOwned, "normal exit allows PP control");
        wbc.Arm(new Dictionary<int, string>());
        Check(wbc.State == GroupState.Arming && legacy.WholeBodyOwned, "next pose can enter CSP again");
    }

    static void CheckMoveL(Robot robot, Calibration calibration, TaskPose upright)
    {
        var workspace = MoveLWorkspace.FromCalibration(calibration);
        Check(workspace.MinimumHeightMm == 466 && workspace.MaximumHeightMm == 972, "MoveL height range includes the tested 466 mm floor");
        var narrow = new Calibration { Motors = calibration.Motors, TaskLimits = new() { ForwardLimit = .15 } };
        Check(MoveLWorkspace.FromCalibration(narrow).MinimumHeightMm > workspace.MinimumHeightMm, "tightened X limits recalculate the height floor");
        double[] zero = new double[4];
        Reject(() => MotionPlan.CreateMoveL(robot, calibration, zero, upright with { Z = .46599 }, 2.2, 2.2),
            "MoveL rejects input below the normal height floor");
        Reject(() => MotionPlan.ResolveCenteredEndpoint(calibration, upright with { Z = .46599 }),
            "saved points use the same minimum height");
        Reject(() => MotionPlan.CreateMoveL(robot, calibration, zero, upright with { Z = .973 }, 2.2, 2.2),
            "MoveL rejects input above zero height");
        var descent = MotionPlan.CreateMoveL(robot, calibration, zero, upright with { Z = workspace.MinimumHeightMm / 1000 }, 2.2, 2.2,
            priority: PositionPriority.Height);
        Check(descent.ExpectedMoveLPose(descent.Duration) is { } low && low.X > 0 &&
            Math.Abs(low.Z * 1000 - workspace.MinimumHeightMm) < 1e-8, "deep fold chooses the closest available X when center is unreachable");
        CheckWholeBodyPlanningMargin(robot, calibration, descent);

        var centerRequest = new TaskPose(.08023, .921, 0, 0);
        Check(MotionPlan.SuggestCenteredX(calibration, zero, centerRequest) == 0, "921 mm height preview prefers center");
        var centered = MotionPlan.CreateMoveL(robot, calibration, zero, centerRequest, 2.2, 2.2,
            priority: PositionPriority.Height);
        Check(centered.ExpectedMoveLPose(centered.Duration) is { } center && center.X == 0 &&
            centered.End[2] > .7 && Enumerable.Range(0, 101).All(i => centered.At(centered.Duration * i / 100)[2] >= 0),
            "921 mm MoveL reaches X=0 on the positive fold");
        Check(MotionPlan.SuggestCenteredX(calibration, zero, upright with { Z = .971 }) is > .025 and < .03,
            "near the upright limit the preview approaches center as far as reach permits");
        var manual = MotionPlan.CreateMoveL(robot, calibration, zero, centerRequest, 2.2, 2.2);
        Check(manual.ExpectedMoveLPose(manual.Duration) is { } manualEnd && Math.Abs(manualEnd.X - .08023) < 1e-10,
            "manually entered X is preserved");
        double[] negativeFold = [0, .02, -.04, .02];
        Reject(() => MotionPlan.CreateMoveL(robot, calibration, negativeFold, centerRequest, 2.2, 2.2),
            "MoveL does not automatically choose the negative fold branch");

        var blocked = new TaskPose(.17825, .55, 0, 0);
        Check(DiskKinematics.Matches(blocked, DiskKinematics.Solve(calibration, blocked, zero, 1)), "blocked line has a valid endpoint");
        Reject(() => MotionPlan.CreateMoveL(robot, calibration, zero, blocked, 2.2, 2.2),
            "MoveL rejects a reachable endpoint whose straight path violates a joint limit");

        double[] folded = [0, -.4, .8, -.4];
        var startPose = DiskKinematics.Forward(folded);
        var to = startPose with { X = startPose.X + .008, Z = startPose.Z - .015, Pitch = .08, Yaw = .2 };
        var mixed = MotionPlan.CreateMoveL(robot, calibration, folded, to, 2.2, 2.2);
        Check(mixed.ExpectedMoveLPose(mixed.Duration / 2) is { } middle &&
            Math.Abs(middle.X - (startPose.X + to.X) / 2) < 1e-9 && Math.Abs(middle.Z - (startPose.Z + to.Z) / 2) < 1e-9 &&
            Math.Abs(middle.Pitch - .04) < 1e-9 && Math.Abs(middle.Yaw - .1) < 1e-9,
            "translation, pitch and heading share one progress");
        var pureYaw = MotionPlan.CreateMoveL(robot, calibration, zero, upright with { Yaw = .2 }, 2.2, 2.2);
        Check(pureYaw.At(pureYaw.Duration / 2).Skip(1).All(q => Math.Abs(q) < 1e-7), "pure yaw keeps the upright disk center fixed");
        foreach (var plan in new[] { descent, centered, mixed, pureYaw })
        {
            for (int i = 0; i <= 500; i++)
            {
                double t = plan.Duration * i / 500;
                var expected = plan.ExpectedMoveLPose(t)!.Value;
                var raw = calibration.Encode(robot, plan.At(t));
                var joint = raw.Select((value, axis) => calibration.ToJoint(axis, value)).ToArray();
                var matrix = robot.Forward(joint)["bady_link_4"];
                double y = (expected.Z - .172) / Math.Tan(-1.5708);
                double error = Math.Sqrt(Math.Pow(matrix.M41 - expected.X, 2) +
                    Math.Pow(matrix.M42 - y, 2) + Math.Pow(matrix.M43 - expected.Z, 2));
                if (error > .0001 || !DiskKinematics.Matches(expected, joint))
                    throw new Exception("Quantized MoveL deviates from the planned pose");
            }
        }
        Check(true, "quantized MoveL commands follow the full URDF line");

        const double maxSpeed = 2.2 * Math.PI / 30;
        const double maxAcceleration = 2.2 * Math.PI / 30;
        const double maxJerk = maxAcceleration / MotionPlan.AccelerationRampSeconds;
        void CheckRates(Func<double, double[]> at, double duration)
        {
            const int samples = 2000;
            double dt = duration / samples;
            var previous = at(0);
            double[]? velocity = null, acceleration = null;
            for (int i = 1; i <= samples; i++)
            {
                var current = at(i * dt);
                var rate = current.Zip(previous, (a, b) => (a - b) / dt).ToArray();
                var nextAcceleration = velocity is null ? null : rate.Zip(velocity, (a, b) => (a - b) / dt).ToArray();
                if (rate.Any(v => Math.Abs(v) > maxSpeed + 1e-7) ||
                    nextAcceleration?.Any(a => Math.Abs(a) > maxAcceleration + 1e-7) == true ||
                    (nextAcceleration is not null && acceleration is not null &&
                        nextAcceleration.Zip(acceleration, (a, b) => Math.Abs(a - b) / dt).Any(j => j > maxJerk + 1e-4)))
                    throw new Exception("MoveL exceeded the speed, acceleration or jerk limit");
                previous = current;
                velocity = rate;
                acceleration = nextAcceleration;
            }
        }
        CheckRates(mixed.At, mixed.Duration);
        foreach (double stopAt in new[] { .1, .4, .7, .95 })
        {
            CheckRates(t =>
            {
                double u = t / MotionPlan.StopTime;
                return mixed.At(mixed.Duration * stopAt + MotionPlan.StopTime * (u - u * u * u + .5 * u * u * u * u));
            }, MotionPlan.StopTime);
        }
        Check(true, "MoveL respects speed, acceleration and jerk in normal motion and same-line stops");
        CheckJerkLimitedPlans(robot, calibration, upright);
    }

    static void CheckJerkLimitedPlans(Robot robot, Calibration calibration, TaskPose upright)
    {
        foreach (var (rpm, accelerationRpm) in new[] { (2.2, 2.2), (5.0, 5.0), (5.0, 30.0), (11.0, 62.0) })
        {
            double rateLimit = rpm * Math.PI / 30;
            double accelerationLimit = accelerationRpm * Math.PI / 30;
            double jerkLimit = accelerationLimit / MotionPlan.AccelerationRampSeconds;
            var plans = new[]
            {
                MotionPlan.CreateMoveL(robot, calibration, new double[4], upright with { Z = .921 },
                    rpm, accelerationRpm, priority: PositionPriority.Height),
                MotionPlan.CreateIndependentToZero(robot, calibration, [0, -.4, .8, -.4], rpm, accelerationRpm),
            };
            foreach (var plan in plans)
            {
                const int samples = 5000;
                double dt = plan.Duration / samples;
                var previous = plan.At(0);
                double[]? previousVelocity = null, previousAcceleration = null;
                for (int k = 1; k <= samples; k++)
                {
                    var current = plan.At(k * dt);
                    var velocity = current.Zip(previous, (a, b) => (a - b) / dt).ToArray();
                    var acceleration = previousVelocity is null ? null :
                        velocity.Zip(previousVelocity, (a, b) => (a - b) / dt).ToArray();
                    var jerk = acceleration is null || previousAcceleration is null ? null :
                        acceleration.Zip(previousAcceleration, (a, b) => (a - b) / dt).ToArray();
                    if (velocity.Any(v => Math.Abs(v) > rateLimit + 1e-6) ||
                        acceleration?.Any(a => Math.Abs(a) > accelerationLimit + 1e-6) == true ||
                        jerk?.Any(j => Math.Abs(j) > jerkLimit + 1e-4) == true)
                        throw new Exception("Whole-body trajectory exceeds velocity, acceleration or jerk limits");
                    previous = current;
                    previousVelocity = velocity;
                    previousAcceleration = acceleration;
                }
                Check(true, "MoveL and CSP zero respect the acceleration-derived jerk limit");
                for (int k = 0; k <= 1000; k++)
                {
                    double t = plan.Duration * k / 1000;
                    var raw0 = calibration.Encode(robot, plan.At(t));
                    var raw1 = calibration.Encode(robot, plan.At(t + plan.MaxCspAdvanceSeconds));
                    if (raw0.Zip(raw1, (a, b) => Math.Abs(a - b)).Any(step => step > MotionPlan.CspMaxStepRaw))
                        throw new Exception("CSP trajectory exceeds the raw step limit at the longest accepted interval");
                }
                Check(true, "per-plan cycle advance preserves the command step bound");
            }
        }
        var pureYaw = upright with { Yaw = 2.4 };
        var atFive = MotionPlan.CreateMoveL(robot, calibration, new double[4], pureYaw, 5, 30);
        var atOneFour = MotionPlan.CreateMoveL(robot, calibration, new double[4], pureYaw, 1.4, 30);
        double peakRpm = Enumerable.Range(0, 1000).Max(i =>
        {
            double dt = atFive.Duration / 1000;
            return Math.Abs(atFive.At((i + 1) * dt)[0] - atFive.At(i * dt)[0]) / dt * 30 / Math.PI;
        });
        Check(atFive.Duration < atOneFour.Duration && peakRpm is > 4.8 and <= 5,
            "the 5 rpm request can actually produce a near-5 rpm MoveL command");
        Check(Math.Abs(atFive.MoveLPlannedPeakRpm - peakRpm) < .01,
            "the logged plan peak matches an independent sample of the MoveL joint rates");
        foreach (var (speed, acceleration) in new[] { (11.01, 1.0), (1.0, 62.01), (double.NaN, 1.0), (1.0, double.PositiveInfinity) })
        {
            Reject(() => MotionPlan.CreateMoveL(robot, calibration, new double[4], upright, speed, acceleration),
                "MoveL rejects invalid speed and acceleration independently");
            Reject(() => MotionPlan.CreateIndependentToZero(robot, calibration, new double[4], speed, acceleration),
                "CSP zero rejects invalid speed and acceleration independently");
        }
    }

    static void CheckWholeBodyPlanningMargin(Robot robot, Calibration calibration, MotionPlan deepFold)
    {
        string fingerprint = calibration.Fingerprint;
        void CheckRawMargin(MotionPlan plan)
        {
            Check(plan.CalibrationHash == fingerprint, "planning margin preserves the original calibration identity");
            for (int j = 0; j <= 1000; j++)
            {
                var raw = calibration.Encode(robot, plan.At(plan.Duration * j / 1000));
                for (int i = 0; i < 4; i++)
                {
                    var motor = calibration.Motors.Single(m => m.LogicalId == i + 1);
                    double displayAngle = calibration.Signs[i] * raw[i] * 360.0 / 65536;
                    if (displayAngle < motor.MinAngleDeg + .4 || displayAngle > motor.MaxAngleDeg - .4)
                        throw new Exception("Encoded whole-body command lost its 0.4 degree margin");
                }
            }
        }
        CheckRawMargin(deepFold);
        CheckRawMargin(MotionPlan.CreateIndependentToZero(robot, calibration, [0, -.4, .8, -.4], 2.2, 2.2));
        Check(calibration.Fingerprint == fingerprint, "planning leaves the saved limits intact");

        var yawLimits = new Calibration
        {
            Motors = [new(1, -10, 10), calibration.Motors[1], calibration.Motors[2], calibration.Motors[3]],
        };
        double[] zero = new double[4];
        var upright = DiskKinematics.Forward(zero);
        foreach (double heading in new[] { -9.9, 9.9 })
        {
            bool rejected = false;
            try { MotionPlan.CreateMoveL(robot, yawLimits, zero, upright with { Yaw = heading * Math.PI / 180 }, 1, 1); }
            catch (ArgumentException) { rejected = true; }
            catch (TimeoutException) { rejected = true; }
            Check(rejected, "target inside the 0.4 degree guard is rejected for either top-axis sign");
        }
        double[] nearLimit = [0, -.4, .8, (calibration.Motors[3].MinAngleDeg + .1) * Math.PI / 180];
        calibration.Check(nearLimit);
        Reject(() => MotionPlan.CreateIndependentToZero(robot, calibration, nearLimit, 1, 1),
            "start inside the guard cannot silently bypass the planning margin");
        var tooNarrow = new Calibration
        {
            Motors = [new(1, -.2, .2), calibration.Motors[1], calibration.Motors[2], calibration.Motors[3]],
        };
        Reject(() => MoveLWorkspace.FromCalibration(tooNarrow), "narrow limits cannot hide an invalid planning interval");
    }

    static void CheckPositionPriority(Robot robot, Calibration calibration)
    {
        double[] measured = [calibration.ToJoint(0, -1266), 0, calibration.ToJoint(2, -1), 0];
        var upright = DiskKinematics.Forward(measured);
        var lower = upright with { Z = upright.Z - .010 };
        var plan = MotionPlan.CreateIndependent(robot, calibration, measured, lower, 1, 1,
            positionPriority: PositionPriority.Height);
        Check(plan.End[2] > .3 && DiskKinematics.Matches(lower, plan.End),
            "one negative encoder count does not lock out the positive branch");

        var heightRequest = lower with { X = .200 };
        var heightPlan = MotionPlan.CreateIndependent(robot, calibration, measured, heightRequest, 1, 1,
            positionPriority: PositionPriority.Height);
        var heightResult = DiskKinematics.Forward(heightPlan.End);
        Check(Math.Abs(heightResult.X - heightRequest.X) > .01 &&
            DiskKinematics.Matches(heightRequest with { X = heightResult.X }, heightPlan.End),
            "height priority adjusts only the unreachable X coordinate");

        var foreAftRequest = upright with { X = .100 };
        var foreAftPlan = MotionPlan.CreateIndependent(robot, calibration, measured, foreAftRequest, 1, 1,
            positionPriority: PositionPriority.ForeAft);
        var foreAftResult = DiskKinematics.Forward(foreAftPlan.End);
        Check(foreAftResult.Z < foreAftRequest.Z - .0005 &&
            DiskKinematics.Matches(foreAftRequest with { Z = foreAftResult.Z }, foreAftPlan.End),
            "fore-aft priority adjusts only the unreachable height");
        Reject(() => MotionPlan.CreateIndependent(robot, calibration, measured, foreAftRequest, 1, 1),
            "when both coordinates are explicit, the planner does not relax either");
    }

    static void CheckIndependentAxes(Robot robot, Calibration calibration, double[] start, TaskPose upright)
    {
        var target = upright with { Z = upright.Z - .010 };
        var plan = MotionPlan.CreateIndependent(robot, calibration, start, target, 1, 1);
        var end = plan.End;
        Check(DiskKinematics.Matches(target, plan.At(plan.Duration)), "independent axes reach the requested task pose");
        int[] movingAxes = Enumerable.Range(0, 4).Where(i => Math.Abs(end[i] - start[i]) > 1e-5).ToArray();
        var first = plan.At(.005);
        Check(movingAxes.Length == 3 && movingAxes.All(i => first[i] != start[i]), "moving axes start in the same CSP tick");
        var nearEnd = plan.At(.9 * plan.Duration);
        Check(movingAxes.Any(i => nearEnd[i] == end[i]) && movingAxes.Any(i => nearEnd[i] != end[i]),
            "shorter axes reach and hold their own targets before the longest axis finishes");

        var speedLimited = MotionPlan.CreateIndependent(robot, calibration, start, target, .2, 1);
        var accelLimited = MotionPlan.CreateIndependent(robot, calibration, start, target, 1, .05);
        Check(speedLimited.Duration > plan.Duration && accelLimited.Duration > plan.Duration,
            "both output limits affect trajectory duration");
        var faster = MotionPlan.CreateIndependent(robot, calibration, start, target, 2.2, 2.2);
        Check(faster.Duration < plan.Duration, "2.2 rpm and 2.2 rpm/s shorten the CSP trajectory");
        const int samples = 1000;
        double dt = plan.Duration / samples;
        var previous = plan.At(0);
        double[] velocity = new double[4];
        for (int k = 1; k <= samples; k++)
        {
            var current = plan.At(k * dt);
            calibration.Encode(robot, current);
            for (int axis = 0; axis < 4; axis++)
            {
                double nextVelocity = (current[axis] - previous[axis]) / dt;
                if (Math.Abs(nextVelocity) > Math.PI / 30 + 1e-9 ||
                    Math.Abs(nextVelocity - velocity[axis]) / dt > Math.PI / 30 + 1e-9)
                    throw new Exception("Independent axis exceeds the requested speed or acceleration");
                velocity[axis] = nextVelocity;
            }
            previous = current;
        }
        Check(true, "sampled independent motion stays within the output limits");

        var narrow = new Calibration { Motors = calibration.Motors, TaskLimits = new() { ForwardLimit = .067, BackwardLimit = 0 } };
        Reject(() => MotionPlan.CreateIndependent(robot, narrow, start, target, 1, 1),
            "rejects an intermediate X excursion even when both endpoints are inside");
    }

    sealed class FakeBus:ICanTransport
    {
        public bool IsOpen=>true;public bool FailBatch;public List<CanFrame> Frames=new();readonly short[] raw=new short[4];readonly Dictionary<int,MotorFeedback> feedback=new();
        public event Action<int,MotorFeedback>? FeedbackUpdated;
        public FakeBus(){Refresh(false);}
        public void SetPosition(short[] values) => values.CopyTo(raw, 0);
        public void Refresh(bool enabled, int skew = 0, int movingAxis = -1, int offsetAxis = -1, int offsetRaw = 0)
        {
            long now = MotorFeedback.Now;
            for (int i = 0; i < raw.Length; i++)
            {
                var sample = new MotorFeedback
                {
                    DeviceId = i + 3,
                    PositionRaw = checked((short)(raw[i] + (i == offsetAxis ? offsetRaw : 0))),
                    VelocityRpm = (short)(i == movingAxis ? 10 : 0),
                    IsFirmware117 = true,
                    Mode = 3,
                    Enabled = enabled,
                    BrakeReleased = enabled,
                    Timestamp = now - (i == 0 ? skew : 0),
                };
                feedback[sample.DeviceId] = sample;
                FeedbackUpdated?.Invoke(sample.DeviceId, sample);
            }
        }
        public bool Send(CanFrame f){Frames.Add(f);int i=(int)f.Id-0x103;if(i>=0&&i<4&&f.Length==7&&f.Data[0]==0xC6)raw[i]=unchecked((short)((f.Data[1]<<8)|f.Data[2]));return true;}
        public int SendBatch(IReadOnlyList<CanFrame> f){int n=FailBatch?1:f.Count;for(int i=0;i<n;i++)Send(f[i]);return n;}
        public MotorFeedback? GetFeedback(int id)=>feedback.GetValueOrDefault(id);public bool IsDeviceAlive(int id,int windowMs=500)=>feedback.ContainsKey(id);
        public Task<SdoResponse?> SdoRequestAsync(int id,CanFrame request,int timeoutMs=300)=>Task.FromResult<SdoResponse?>(null);
    }
}

