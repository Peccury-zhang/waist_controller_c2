using System.Diagnostics;
namespace WaistModel;
public sealed partial class MotionPlan
{
    readonly double[][][] segments;
    readonly double[] start,end;
    readonly double[]? axisDurations;
    public double[] Start=>(double[])start.Clone(); public double[] End=>(double[])end.Clone();
    public string ModelHash {get;} public string CalibrationHash {get;} public double Duration {get;}
    MotionPlan(Robot r, Calibration c, double[] start, double[] end, double duration,
        double[][][] segments, double[]? axisDurations = null, MoveLPath? moveL = null)
    {
        this.start = (double[])start.Clone();
        this.end = (double[])end.Clone();
        this.segments = segments;
        this.axisDurations = axisDurations;
        this.moveL = moveL;
        Duration = duration;
        ModelHash = r.Hash;
        CalibrationHash = c.Fingerprint;
    }
    static double Eval(double[] controls, double u)
    {
        Span<double> work = stackalloc double[6];
        controls.AsSpan().CopyTo(work);
        for (int last = 4; last >= 0; last--)
        for (int i = 0; i <= last; i++)
        {
            double a = work[i], b = work[i + 1];
            work[i] = Math.Clamp(a + (b - a) * u, Math.Min(a, b), Math.Max(a, b));
        }
        return work[0];
    }
    // Near the straight-leg singularity, joint displacement grows with the
    // square root of task displacement. Sixth-order endpoint progress keeps
    // joint velocity and acceleration zero with bounded jerk there.
    static double Progress(double u)
    {
        if (u > .5) return 1 - Progress(1 - u);
        double u3 = u * u * u;
        return u3 * u3 * (462 + u * (-1980 + u * (3465 + u * (-3080 + u * (1386 - 252 * u)))));
    }
    public double[] At(double t)
    {
        if (axisDurations is not null)
            return Enumerable.Range(0, 4).Select(i => AxisPosition(i, t)).ToArray();

        double v = Math.Clamp(t / Duration, 0, 1) * segments.Length;
        int i = Math.Min((int)v, segments.Length - 1);
        return segments[i].Select(a => Eval(a, v - i)).ToArray();
    }

    private double AxisPosition(int axis, double t)
    {
        double from = start[axis], to = end[axis], duration = axisDurations![axis];
        if (t <= 0) return from;
        if (t >= duration) return to;
        double u = t / duration;
        double progress = u * u * u * (10 + u * (-15 + 6 * u));
        return Math.Clamp(from + (to - from) * progress, Math.Min(from, to), Math.Max(from, to));
    }

    public const decimal MinimumOutputLimit = .05m;
    public const decimal MaximumOutputLimit = 5m;

    public static MotionPlan CreateIndependent(Robot robot, Calibration calibration, double[] start,
        TaskPose target, double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel = default,
        PositionPriority positionPriority = PositionPriority.Both) =>
        CreateIndependentCore(robot, calibration, start, target, maxSpeedRpm, maxAccelerationRpmPerSec, positionPriority, cancel);

    public static MotionPlan CreateIndependentToZero(Robot robot, Calibration calibration, double[] start,
        double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel = default) =>
        CreateIndependentCore(robot, calibration, start, null, maxSpeedRpm, maxAccelerationRpmPerSec, PositionPriority.Both, cancel);

    private static MotionPlan CreateIndependentCore(Robot robot, Calibration calibration, double[] start,
        TaskPose? target, double maxSpeedRpm, double maxAccelerationRpmPerSec, PositionPriority priority, CancellationToken cancel)
    {
        if (!ValidOutputLimit(maxSpeedRpm) || !ValidOutputLimit(maxAccelerationRpmPerSec))
            throw new ArgumentException("最大速度与最大加速度须在 0.05～5 的范围内");

        cancel.ThrowIfCancellationRequested();
        var clock = Stopwatch.StartNew();
        ValidateModel(robot);
        calibration.Validate();
        calibration.Check(start);
        MotionPlan Build(double[] end, TaskPose? pose) =>
            BuildIndependent(robot, calibration, start, end, pose, maxSpeedRpm, maxAccelerationRpmPerSec, clock, cancel);

        if (target is not { } request)
        {
            var planningLimits = calibration.CreateWholeBodyPlanningLimits();
            var zero = new double[4];
            // The zero-return curve is monotone on every axis, so its two
            // endpoints bound the entire joint motion.
            planningLimits.CheckPlanningAngles(start);
            planningLimits.CheckPlanningAngles(zero);
            return Build(zero, null);
        }
        const int positiveFold = 1;
        if (priority == PositionPriority.Both)
            return Build(DiskKinematics.Solve(calibration, request, start, positiveFold), request);

        if (!double.IsFinite(request.X) || !double.IsFinite(request.Z) ||
            !double.IsFinite(request.Pitch) || !double.IsFinite(request.Yaw))
            throw new ArgumentException("目标必须为有限数值");
        if (priority == PositionPriority.ForeAft) calibration.CheckX(request.X);

        var candidates = DiskKinematics.PositionAlternatives(calibration, request, priority).Prepend(request);
        foreach (var pose in candidates)
        {
            cancel.ThrowIfCancellationRequested();
            if (clock.Elapsed.TotalSeconds > 1) throw new TimeoutException("可行位姿搜索超过预算，请缩小动作");
            var end = DiskKinematics.FindAnalytic(calibration, pose, start, positiveFold, out _);
            if (end is null) continue;
            try { return Build(end, pose); }
            catch (ArgumentException) { } // Quantization, duration, or path checks can rule out this candidate.
        }

        string required = priority == PositionPriority.Height ? "高度" : "前后";
        throw new ArgumentException($"在保持目标{required}、俯仰和航向时，未找到通过限位校验的正向折叠路径");
    }

    private static MotionPlan BuildIndependent(Robot robot, Calibration calibration, double[] start, double[] end,
        TaskPose? target, double maxSpeedRpm, double maxAccelerationRpmPerSec, Stopwatch clock, CancellationToken cancel)
    {
        var rawEnd = calibration.Encode(robot, end);
        if (target is { } requestedPose &&
            !DiskKinematics.Matches(requestedPose, rawEnd.Select((raw, i) => calibration.ToJoint(i, raw)).ToArray()))
            throw new ArgumentException("目标位姿量化后误差超限");

        const double rpmToRadPerSecond = 2 * Math.PI / 60;
        double speed = maxSpeedRpm * rpmToRadPerSecond;
        double acceleration = maxAccelerationRpmPerSec * rpmToRadPerSecond;
        // Let jerk scale with the selected acceleration, while keeping a
        // bounded acceleration ramp and an absolute CSP ceiling.
        double maxJerk = Math.Min(1.0, acceleration / .25);
        var durations = new double[4];
        for (int i = 0; i < durations.Length; i++)
        {
            double distance = Math.Abs(end[i] - start[i]);
            // Peak normalized derivatives of 10u³ - 15u⁴ + 6u⁵.
            durations[i] = Math.Max(1.875 * distance / speed,
                Math.Max(Math.Sqrt((10 / Math.Sqrt(3)) * distance / acceleration),
                    Math.Cbrt(60 * distance / maxJerk)));
        }

        double duration = durations.Max();
        if (duration > 60) throw new ArgumentException("轨迹超过 60 秒，请减小动作或检查速度设置");
        var plan = new MotionPlan(robot, calibration, start, end, duration, [], durations);
        plan.CertifyIndependentX(calibration, 0, duration, 0, clock, cancel);
        return plan;
    }

    private static bool ValidOutputLimit(double value) =>
        double.IsFinite(value) && value >= (double)MinimumOutputLimit && value <= (double)MaximumOutputLimit;

    private void CertifyIndependentX(Calibration calibration, double from, double to, int depth,
        Stopwatch clock, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();
        if (clock.Elapsed.TotalSeconds > 1) throw new TimeoutException("独立四轴路径验证超过预算");
        var center = new double[4];
        var radius = new double[4];
        for (int axis = 1; axis < 4; axis++)
        {
            double a = AxisPosition(axis, from), b = AxisPosition(axis, to);
            // Each joint moves monotonically, so these endpoints enclose the interval.
            center[axis] = (a + b) / 2;
            radius[axis] = Math.Abs(a - b) / 2;
        }

        double x = DiskKinematics.ForwardX(center);
        // The inward encoder clamp can shift a boundary target by up to one count.
        double rx = .8 * radius[3] + .47 * radius[2] + .09 * radius[1] + 1.36 * (2 * Math.PI / 65536);
        var limits = calibration.TaskLimits;
        if (x - rx >= -limits.BackwardLimit && x + rx <= limits.ForwardLimit) return;
        double mid = (from + to) / 2;
        double actualX = DiskKinematics.ForwardX(At(mid));
        if (depth >= 18 || actualX < -limits.BackwardLimit || actualX > limits.ForwardLimit || from == to)
            throw new ArgumentException("独立四轴路径可能超出前后标定范围，请减小动作");
        CertifyIndependentX(calibration, from, mid, depth + 1, clock, cancel);
        CertifyIndependentX(calibration, mid, to, depth + 1, clock, cancel);
    }
    public static MotionPlan Create(Robot r,Calibration c,double[] start,TaskPose target,CancellationToken cancel=default)
    {
        ValidateModel(r);c.Validate();c.Check(start);c.CheckX(target.X);var clock=Stopwatch.StartNew();const int n=400;var points=new double[n+1][];points[0]=(double[])start.Clone();var from=DiskKinematics.Forward(start);
        double yawDelta=DiskKinematics.Wrap(target.Yaw-from.Yaw);int branch=Math.Sign(start[2]);
        for(int k=1;k<=n;k++)
        {
            cancel.ThrowIfCancellationRequested();if(clock.Elapsed.TotalSeconds>1)throw new TimeoutException("路径规划超过1秒，请减小动作");double s=Progress(k/(double)n);
            var task=new TaskPose(from.X+(target.X-from.X)*s,from.Z+(target.Z-from.Z)*s,from.Pitch+(target.Pitch-from.Pitch)*s,from.Yaw+yawDelta*s);
            points[k]=DiskKinematics.Solve(c,task,points[k-1],branch);if(branch==0&&Math.Abs(points[k][2])>1e-6)branch=Math.Sign(points[k][2]);
        }
        var velocities = new double[n + 1, 4];
        var accelerations = new double[n + 1, 4];
        for (int k = 1; k < n; k++)
        for (int axis = 0; axis < 4; axis++)
        {
            double p = points[k][axis], previous = points[k - 1][axis], next = points[k + 1][axis];
            double v = (next - previous) / 2, a = next - 2 * p + previous;
            var bounds = c.Bounds(axis);
            // The two control points on each side of this node depend on
            // its derivatives. Limit them together to retain C2 continuity.
            double scale = 1;
            foreach (double offset in new[] { v / 5, -v / 5, 2 * v / 5 + a / 20, -2 * v / 5 + a / 20 })
            {
                if (p + offset > bounds.High) scale = Math.Min(scale, (bounds.High - p) / offset);
                if (p + offset < bounds.Low) scale = Math.Min(scale, (bounds.Low - p) / offset);
            }
            velocities[k, axis] = v * scale;
            accelerations[k, axis] = a * scale;
        }
        var coeff=new double[n][][];double duration=.5;
        for(int k=0;k<n;k++)
        {
            coeff[k]=new double[4][];
            for(int axis=0;axis<4;axis++)
            {
                double p = points[k][axis], next = points[k + 1][axis];
                double v = velocities[k, axis], nextV = velocities[k + 1, axis];
                var bounds = c.Bounds(axis);
                double ControlPoint(double value) => Math.Clamp(value, bounds.Low, bounds.High);
                double[] b =
                [
                    p,
                    ControlPoint(p + v / 5),
                    ControlPoint(p + (2 * v / 5 + accelerations[k, axis] / 20)),
                    ControlPoint(next + (-2 * nextV / 5 + accelerations[k + 1, axis] / 20)),
                    ControlPoint(next - nextV / 5),
                    next,
                ];
                coeff[k][axis] = b;
                double vmax = 0, amax = 0, jmax = 0;
                double prevV = 0, prevA = 0;
                for (int j = 1; j < b.Length; j++)
                {
                    double velocity = 5 * (b[j] - b[j - 1]), acceleration = 4 * (velocity - prevV);
                    vmax = Math.Max(vmax, Math.Abs(velocity));
                    if (j >= 2) amax = Math.Max(amax, Math.Abs(acceleration));
                    if (j >= 3) jmax = Math.Max(jmax, Math.Abs(3 * (acceleration - prevA)));
                    prevV = velocity;
                    prevA = acceleration;
                }
                duration=Math.Max(duration,Math.Max(vmax*n/.1,Math.Max(Math.Sqrt(amax*n*n/.1),Math.Cbrt(jmax*n*n*n/.2))));
            }
            Certify(c, coeff[k], 0, clock, cancel);
            // Independent interpolation residual checks, including raw quantization at sample targets.
            for(int j=0;j<=4;j++){double s=Progress((k+j/4.0)/n);var q=coeff[k].Select(a=>Eval(a,j/4.0)).ToArray();c.Encode(r,q);var expected=new TaskPose(from.X+(target.X-from.X)*s,from.Z+(target.Z-from.Z)*s,from.Pitch+(target.Pitch-from.Pitch)*s,from.Yaw+yawDelta*s);if(!DiskKinematics.Matches(expected,q))throw new ArgumentException("任务路径插值误差超限");}
        }
        if(duration>60||!double.IsFinite(duration))throw new ArgumentException("轨迹接近奇异点或耗时超过60秒，请缩小动作");
        cancel.ThrowIfCancellationRequested();if(clock.Elapsed.TotalSeconds>1)throw new TimeoutException("整路验证超过1秒");
        return new(r, c, points[0], points[^1], duration, coeff);
    }
    static void ValidateModel(Robot r)
    {
        foreach(var q in new[]{new double[4],new double[]{.2,-.3,.5,-.2},new double[]{-.3,.2,-.2,.1}})
        {
            var p=DiskKinematics.Forward(q);var f=r.Forward(q)["bady_link_4"];
            if(Math.Abs(p.X-f.M41)>1e-6||Math.Abs(p.Z-f.M43)>1e-6||Math.Abs(DiskKinematics.Wrap(p.Yaw-Math.Atan2(f.M12,f.M11)))>1e-5)throw new InvalidDataException("URDF与四轴解析模型不匹配，禁止执行");
        }
    }
    static (double[] Left, double[] Right) SplitBernstein(double[] points)
    {
        Span<double> work = stackalloc double[6];
        points.AsSpan().CopyTo(work);
        var left = new double[6];
        var right = new double[6];
        left[0] = work[0];
        right[5] = work[5];
        for (int last = 4; last >= 0; last--)
        {
            for (int i = 0; i <= last; i++)
                work[i] = (work[i] + work[i + 1]) / 2;
            left[5 - last] = work[0];
            right[last] = work[last];
        }
        return (left, right);
    }

    static void Certify(Calibration c, double[][] controls, int depth, Stopwatch clock, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();
        if (clock.Elapsed.TotalSeconds > 1) throw new TimeoutException("路径区间验证超过预算");
        var center = new double[4];
        var radius = new double[4];
        bool okay = true;
        for (int i = 0; i < 4; i++)
        {
            var bounds = c.Bounds(i);
            // The control point range encloses the entire polynomial, including
            // paths that start on a limit and move inward.
            double low = controls[i].Min(), high = controls[i].Max();
            okay &= low >= bounds.Low && high <= bounds.High;
            center[i] = (low + high) / 2;
            radius[i] = Math.Max(center[i] - low, high - center[i]);
        }
        // Global bound |dx| <= .8|dq1| + .47|dq2| + .09|dq3|. Add half-count encoding uncertainty.
        double x = DiskKinematics.Forward(center).X;
        double rx = .8 * radius[3] + .47 * radius[2] + .09 * radius[1] + 1.36 * Math.PI / 65536;
        okay &= x - rx >= -c.TaskLimits.BackwardLimit && x + rx <= c.TaskLimits.ForwardLimit;
        if (okay) return;
        if (depth >= 18)
        {
            c.Check(center, false);
            throw new ArgumentException("路径过于接近标定边界，无法证明插值及量化后仍在范围内");
        }
        var halves = controls.Select(SplitBernstein).ToArray();
        Certify(c, halves.Select(half => half.Left).ToArray(), depth + 1, clock, cancel);
        Certify(c, halves.Select(half => half.Right).ToArray(), depth + 1, clock, cancel);
    }
}
