using WaistModel;

internal static class SmoothMotionTests
{
    public static void Run(Robot robot)
    {
        var c = new Calibration
        {
            Motors = [new(1, -172, 172), new(2, -100, 15), new(3, -10, 143), new(4, -54, 10)],
        };
        const double rad = Math.PI / 180;
        double[] low = [0, -88.76953125 * rad, 142.3663330078125 * rad, -53.5968017578125 * rad];
        var target = new TaskPose(0, .9, 0, 0);
        var up = MotionPlan.CreateSmooth(robot, c, low, target, 8, 60);
        Check(up.IsSmooth && up.SmoothLegCount == 1 && up.PlannedPeakRpm <= 8.000001,
            "466 to 900 has one shared smooth leg within requested speed");
        Check(DiskKinematics.Matches(target, up.End), "900/0 target is reached exactly");
        Verify(c, up, 8, 60, holdPitch: true);
        var back = MotionPlan.CreateSmooth(robot, c, up.End, DiskKinematics.Forward(low), 8, 60);
        Verify(c, back, 8, 60, holdPitch: true);
        Check(back.End[3] >= -53.6 * rad, "reverse endpoint keeps the 0.4 degree planning margin");
        var zeroStart = MotionPlan.CreateSmooth(robot, c, new double[4], target, 5, 30);
        Verify(c, zeroStart, 5, 30, holdPitch: true);
        var poseChange = MotionPlan.CreateSmooth(robot, c, up.End, target with { Pitch = 5 * rad, Yaw = 20 * rad },
            5, 30, keepPitch: false);
        Verify(c, poseChange, 5, 30, holdPitch: false);
        Check(DiskKinematics.Matches(target with { Pitch = 5 * rad, Yaw = 20 * rad }, poseChange.End),
            "explicit pitch and yaw reach the requested endpoint");
        var lockPitch = MotionPlan.CreateSmooth(robot, c, low, target with { Pitch = 5 * rad }, 5, 30);
        Check(Math.Abs(DiskKinematics.Forward(lockPitch.End).Pitch) < 1e-10,
            "pitch lock overrides a saved point pitch");
        foreach (double fold in new[] { .8, -.1 })
        {
            double[] tiltedStart = [.12, -fold / 2 + 2 * rad, fold, -fold / 2];
            var zero = MotionPlan.CreateSmoothToZero(robot, c, tiltedStart, 8, 60);
            Check(zero.IsSmooth && zero.Start.SequenceEqual(tiltedStart) &&
                c.Encode(robot, zero.End).All(raw => raw == 0) &&
                DiskKinematics.Matches(new TaskPose(.066, .972, 0, 0), zero.End),
                "shared smooth zero move reaches exact zero from a tilted pose on either fold branch");
            Verify(c, zero, 8, 60, holdPitch: false);
        }
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        bool cancelled = false;
        try { MotionPlan.CreateSmooth(robot, c, low, target, 8, 60, cancellation.Token); }
        catch (OperationCanceledException) { cancelled = true; }
        Check(cancelled, "cancelled planning cannot produce a trajectory");

        // The direct joint path reaches X=183.5 mm, outside this narrower envelope.
        c.TaskLimits.ForwardLimit = .181;
        var detour = MotionPlan.CreateSmooth(robot, c, low, target, 8, 60);
        Check(detour.SmoothLegCount > 1, "bounded two-coordinate search routes around an invalid direct curve");
        Verify(c, detour, 8, 60, holdPitch: true);
        Console.WriteLine($"PASS smooth paths: 466↔900, upright, pose changes, limits, derivatives, stops, fallback ({detour.SmoothLegCount} legs).");
    }

    private static void Verify(Calibration c, MotionPlan plan, double speed, double acceleration, bool holdPitch)
    {
        double initialPitch = DiskKinematics.Forward(plan.Start).Pitch;
        const double step = .0005;
        double[]? prev = null, prevVelocity = null, prevAcceleration = null;
        for (double t = 0; t <= plan.Duration; t += step)
        {
            var q = plan.At(t);
            var pose = DiskKinematics.Forward(q);
            Check(pose.X >= -c.TaskLimits.BackwardLimit - 1e-9 && pose.X <= c.TaskLimits.ForwardLimit + 1e-9 &&
                pose.Z >= .4659 && pose.Z <= .9721, "continuous path remains in workspace");
            Check(!holdPitch || Math.Abs(pose.Pitch - initialPitch) < 1e-9, "constant pitch throughout");
            foreach (int axis in Enumerable.Range(0, 4))
            {
                var bounds = c.Bounds(axis);
                Check(q[axis] >= bounds.Low + .4 * Math.PI / 180 - 1e-10 &&
                    q[axis] <= bounds.High - .4 * Math.PI / 180 + 1e-10, "all joint samples retain planning margin");
            }
            if (prev is not null)
            {
                var velocity = q.Zip(prev, (a, b) => (a - b) / step * 30 / Math.PI).ToArray();
                Check(velocity.All(v => Math.Abs(v) <= speed + .0001), "speed upper bound");
                if (prevVelocity is not null)
                {
                    var acc = velocity.Zip(prevVelocity, (a, b) => (a - b) / step).ToArray();
                    Check(acc.All(v => Math.Abs(v) <= acceleration + .001), "acceleration upper bound");
                    if (prevAcceleration is not null)
                        Check(acc.Zip(prevAcceleration, (a, b) => Math.Abs(a - b) / step)
                            .All(v => v <= acceleration / MotionPlan.AccelerationRampSeconds + .1), "jerk upper bound");
                    prevAcceleration = acc;
                }
                prevVelocity = velocity;
            }
            prev = q;
        }
        // Check the existing common-clock stopping law against the new path.
        foreach (double fraction in new[] { .05, .35, .8 })
        {
            prev = prevVelocity = prevAcceleration = null;
            for (double t = 0; t <= MotionPlan.StopTime; t += step)
            {
                double u = t / MotionPlan.StopTime;
                double clock = plan.Duration * fraction + MotionPlan.StopTime * (u - u*u*u + .5*u*u*u*u);
                var q = plan.At(clock);
                if (prev is not null)
                {
                    var velocity = q.Zip(prev, (a,b) => (a-b)/step*30/Math.PI).ToArray();
                    Check(velocity.All(v => Math.Abs(v) <= speed + .001), "stop speed");
                    if (prevVelocity is not null)
                    {
                        var acc = velocity.Zip(prevVelocity, (a,b) => (a-b)/step).ToArray();
                        Check(acc.All(v => Math.Abs(v) <= acceleration + .01), "stop acceleration");
                        if (prevAcceleration is not null)
                            Check(acc.Zip(prevAcceleration, (a,b) => Math.Abs(a-b)/step)
                                .All(v => v <= acceleration / MotionPlan.AccelerationRampSeconds + .2), "stop jerk");
                        prevAcceleration = acc;
                    }
                    prevVelocity = velocity;
                }
                prev = q;
            }
        }
    }

    private static void Check(bool passed, string message)
    {
        if (!passed) throw new Exception("Smooth motion: " + message);
    }
}
