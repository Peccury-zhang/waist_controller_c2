using System.Diagnostics;

namespace WaistModel;

public sealed partial class MotionPlan
{
    private const double PositionTolerance = .0001;
    private const double AngleTolerance = Math.PI / 18000;
    private const double EncoderCount = 2 * Math.PI / 65536;
    public const double StopTime = 2;
    public const int CspMaxStepRaw = 90;
    public const double CspMaxInterval = .02;
    private readonly MoveLPath? moveL;

    private sealed record MoveLPath(TaskPose From, TaskPose To, double YawDelta, bool Regularized, string? LimitingConstraint = null)
    {
        public TaskPose Pose(double u)
        {
            double s = Regularized ? Progress(u) : u * u * u * (10 + u * (-15 + 6 * u));
            return new(From.X + (To.X - From.X) * s, From.Z + (To.Z - From.Z) * s,
                From.Pitch + (To.Pitch - From.Pitch) * s, From.Yaw + YawDelta * s);
        }
    }

    public TaskPose? ExpectedMoveLPose(double elapsed) => moveL?.Pose(Math.Clamp(elapsed / Duration, 0, 1));
    public string? MoveLLimitingConstraint => moveL?.LimitingConstraint;

    public (double CrossTrackMm, double AlongErrorMm, double AngleErrorDeg)? MoveLDeviation(double elapsed, TaskPose actual)
    {
        if (moveL is not { } path || ExpectedMoveLPose(elapsed) is not { } expected) return null;
        double scale = -Math.Sin(DiskKinematics.Alpha);
        double dx = path.To.X - path.From.X, dz = (path.To.Z - path.From.Z) / scale;
        double ex = actual.X - expected.X, ez = (actual.Z - expected.Z) / scale;
        double length = Math.Sqrt(dx * dx + dz * dz);
        double cross = length > 1e-9 ? Math.Abs(dx * ez - dz * ex) / length : Math.Sqrt(ex * ex + ez * ez);
        double along = length > 1e-9 ? (dx * ex + dz * ez) / length : 0;
        double angle = Math.Max(Math.Abs(actual.Pitch - expected.Pitch), Math.Abs(DiskKinematics.Wrap(actual.Yaw - expected.Yaw)));
        return (cross * 1000, along * 1000, angle * 180 / Math.PI);
    }

    private const int PositiveFold = 1;

    private static void RequirePositiveFoldStart(double[] start)
    {
        if (start[2] < 0)
            throw new ArgumentException("逻辑轴 3（中部弯曲）当前为负角。MoveL 仅允许正折叠分支，请先执行 CSP回零位");
    }

    private static IEnumerable<TaskPose> CenteredHeightTargets(Calibration calibration, TaskPose request)
    {
        var centered = request with { X = 0 };
        yield return centered;
        foreach (var pose in DiskKinematics.PositionAlternatives(calibration, centered, PositionPriority.Height, PositiveFold))
            if (pose.X != 0) yield return pose;
    }

    public static double SuggestCenteredX(Calibration calibration, double[] start, TaskPose request)
    {
        calibration.Check(start);
        var limits = calibration.CreateWholeBodyPlanningLimits();
        limits.CheckPlanningAngles(start);
        RequirePositiveFoldStart(start);
        foreach (var pose in CenteredHeightTargets(limits, request))
        {
            var joint = DiskKinematics.FindAnalytic(limits, pose, start, PositiveFold, out _);
            if (joint is not null) return pose.X;
        }
        throw new ArgumentException("此高度和姿态在正折叠分支上没有可用前后值");
    }

    public static MotionPlan CreateMoveL(Robot robot, Calibration calibration, double[] start, TaskPose request,
        double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel = default,
        PositionPriority priority = PositionPriority.Both)
    {
        if (!ValidOutputLimit(maxSpeedRpm) || !ValidOutputLimit(maxAccelerationRpmPerSec))
            throw new ArgumentException("最大速度与最大加速度须在 0.05～5 的范围内");
        if (!double.IsFinite(request.X) || !double.IsFinite(request.Z) ||
            !double.IsFinite(request.Pitch) || !double.IsFinite(request.Yaw))
            throw new ArgumentException("目标必须为有限数值");
        var clock = Stopwatch.StartNew();
        ValidateModel(robot);
        var planningLimits = calibration.CreateWholeBodyPlanningLimits();
        var workspace = MoveLWorkspace.FromPlanningLimits(planningLimits);
        calibration.Check(start);
        planningLimits.CheckPlanningAngles(start);
        RequirePositiveFoldStart(start);
        var from = DiskKinematics.Forward(start);
        workspace.CheckHeight(from.Z, feedbackToleranceMm: .1);
        workspace.CheckHeight(request.Z);
        if (priority != PositionPriority.Height) calibration.CheckX(request.X);

        IEnumerable<TaskPose> candidates = priority switch
        {
            PositionPriority.Height => CenteredHeightTargets(planningLimits, request),
            PositionPriority.ForeAft => DiskKinematics.PositionAlternatives(planningLimits, request, priority, PositiveFold).Prepend(request),
            _ => [request],
        };
        string? failure = null;
        foreach (var target in candidates)
        {
            CheckBudget(clock, cancel);
            if (target.Z * 1000 < workspace.MinimumHeightMm - 1e-7 ||
                target.Z * 1000 > workspace.MaximumHeightMm + 1e-7) continue;
            var end = DiskKinematics.FindAnalytic(planningLimits, target, start, PositiveFold, out _);
            if (end is null && priority == PositionPriority.Both)
                end = DiskKinematics.Solve(planningLimits, target, start, PositiveFold);
            if (end is null) continue;

            double shortestYaw = DiskKinematics.Wrap(target.Yaw - from.Yaw);
            double[] yawOptions = Math.Abs(shortestYaw) < 1e-9 ? [shortestYaw] :
                [shortestYaw, shortestYaw - Math.CopySign(2 * Math.PI, shortestYaw)];
            foreach (double yawDelta in yawOptions)
            {
                bool changingBend = PositionError(from, target) > 1e-9 || Math.Abs(from.Pitch - target.Pitch) > 1e-9;
                var path = new MoveLPath(from, target, yawDelta, changingBend && Math.Min(Math.Abs(start[2]), Math.Abs(end[2])) < .02);
                try
                {
                    return BuildMoveL(robot, calibration, planningLimits, start, path, maxSpeedRpm, maxAccelerationRpmPerSec, clock, cancel);
                }
                catch (ArgumentException e) { failure ??= e.Message; }
            }
            if (priority == PositionPriority.Both) break;
        }
        throw new ArgumentException(failure ?? "未找到满足正折叠分支及 0.3° 内侧余量的 MoveL 目标");
    }

    private static MotionPlan BuildMoveL(Robot robot, Calibration calibration, Calibration planningLimits, double[] start, MoveLPath path,
        double speedRpm, double accelerationRpm, Stopwatch clock, CancellationToken cancel)
    {
        const double rpmToRad = 2 * Math.PI / 60;
        // Allow for one encoder count of error at either end of a late cycle.
        double stepSpeed = (CspMaxStepRaw - 2) * EncoderCount / CspMaxInterval;
        double speed = Math.Min(speedRpm * rpmToRad, stepSpeed);
        double acceleration = accelerationRpm * rpmToRad;
        double jerk = Math.Min(1, acceleration / .25);
        var limits = Enumerable.Range(0, 4).Select(planningLimits.Bounds).ToArray();
        var encoderLimits = Enumerable.Range(0, 4).Select(i =>
        {
            var (low, high) = calibration.RawLimits(robot, i);
            double a = calibration.ToJoint(i, low), b = calibration.ToJoint(i, high);
            return (Low: Math.Min(a, b), High: Math.Max(a, b));
        }).ToArray();
        limits[2] = (Math.Max(0, limits[2].Low), limits[2].High);

        for (int n = 32; n <= 512; n *= 2)
        {
            var points = new double[n + 1][];
            points[0] = start;
            for (int k = 1; k <= n; k++)
            {
                CheckBudget(clock, cancel);
                var wanted = path.Pose(k / (double)n);
                try { points[k] = DiskKinematics.Solve(planningLimits, wanted, points[k - 1], PositiveFold); }
                catch (Exception e) when (e is ArgumentException or TimeoutException)
                {
                    throw new ArgumentException($"MoveL 途中 X={wanted.X * 1000:F1} mm、高度={wanted.Z * 1000:F1} mm 未通过 IK · {e.Message}", e);
                }
                if (points[k].Zip(points[k - 1], (a, b) => Math.Abs(a - b)).Any(delta => delta > .5))
                    throw new ArgumentException("MoveL 中途出现关节跳支，请缩小位移或转角");
                if (k < n && Math.Abs(points[k][2]) < 1e-7 && Math.Abs(start[2]) > 1e-7)
                    throw new ArgumentException("MoveL 中途接近伸直奇异点，请分段执行");
            }

            var velocities = new double[n + 1, 4];
            var accelerations = new double[n + 1, 4];
            for (int k = 1; k < n; k++)
            for (int axis = 0; axis < 4; axis++)
            {
                double p = points[k][axis], previous = points[k - 1][axis], next = points[k + 1][axis];
                double v = (next - previous) / 2, a = next - 2 * p + previous;
                var (low, high) = limits[axis];
                double scale = 1;
                foreach (double offset in new[] { v / 5, -v / 5, 2 * v / 5 + a / 20, -2 * v / 5 + a / 20 })
                {
                    if (p + offset > high) scale = Math.Min(scale, (high - p) / offset);
                    if (p + offset < low) scale = Math.Min(scale, (low - p) / offset);
                }
                velocities[k, axis] = v * scale;
                accelerations[k, axis] = a * scale;
            }

            var curves = new double[n][][];
            double duration = .5;
            string? limitingConstraint = null;
            bool certified = true;
            for (int k = 0; k < n; k++)
            {
                CheckBudget(clock, cancel);
                var segment = curves[k] = new double[4][];
                for (int axis = 0; axis < 4; axis++)
                {
                    double p = points[k][axis], end = points[k + 1][axis];
                    double v = velocities[k, axis], nextV = velocities[k + 1, axis];
                    var (low, high) = limits[axis];
                    double Bound(double value) => Math.Clamp(value, low, high);
                    var b = segment[axis] = [p, Bound(p + v / 5),
                        Bound(p + 2 * v / 5 + accelerations[k, axis] / 20),
                        Bound(end - 2 * nextV / 5 + accelerations[k + 1, axis] / 20),
                        Bound(end - nextV / 5), end];
                    var (d1, d2, d3) = DerivativeBounds(b);
                    double vNorm = d1 * n, aNorm = d2 * n * n;
                    // Reserve acceleration for the same-path stop clock.
                    // |q̈_stop| <= aNorm/T² + (1.5/StopTime) * vNorm/T.
                    double stopTerm = 1.5 * vNorm / StopTime;
                    double accelTime = (stopTerm + Math.Sqrt(stopTerm * stopTerm + 4 * acceleration * aNorm)) / (2 * acceleration);
                    double speedTime = vNorm / speed, jerkTime = n * Math.Cbrt(d3 / jerk);
                    double requiredTime = Math.Max(speedTime, Math.Max(accelTime, jerkTime));
                    if (requiredTime > duration)
                    {
                        duration = requiredTime;
                        string limit = requiredTime == speedTime ? (speed == stepSpeed ? "CSP 步长" : "速度") :
                            requiredTime == accelTime ? "加速度和制动余量" : "加加速度";
                        limitingConstraint = $"轴 {axis + 1} {limit}限速";
                    }
                }
                if (!CertifyMoveL(calibration.TaskLimits, limits, encoderLimits, path, segment,
                    k / (double)n, (k + 1) / (double)n, 0, clock, cancel))
                {
                    certified = false;
                    break;
                }
            }
            if (!certified) continue;
            if (!double.IsFinite(duration) || duration > 60)
                throw new ArgumentException("MoveL 超过 60 秒，请减小动作或检查速度设置");
            var plan = new MotionPlan(robot, calibration, start, points[^1], duration, curves,
                moveL: path with { LimitingConstraint = limitingConstraint });
            VerifyCspSamples(robot, calibration, plan, path, clock, cancel);
            return plan;
        }
        throw new ArgumentException("MoveL 的直线精度、关节范围或前后余量无法验证");
    }

    private static (double Velocity, double Acceleration, double Jerk) DerivativeBounds(double[] b)
    {
        double first = 0, second = 0, third = 0, previousV = 0, previousA = 0;
        for (int i = 1; i < b.Length; i++)
        {
            double v = 5 * (b[i] - b[i - 1]), a = 4 * (v - previousV);
            first = Math.Max(first, Math.Abs(v));
            if (i >= 2) second = Math.Max(second, Math.Abs(a));
            if (i >= 3) third = Math.Max(third, Math.Abs(3 * (a - previousA)));
            previousV = v;
            previousA = a;
        }
        return (first, second, third);
    }

    private static double PositionError(TaskPose a, TaskPose b)
    {
        double x = a.X - b.X, z = (a.Z - b.Z) / Math.Sin(DiskKinematics.Alpha);
        return Math.Sqrt(x * x + z * z);
    }

    private static bool CertifyMoveL(TaskLimits taskLimits, (double Low, double High)[] jointLimits,
        (double Low, double High)[] encoderLimits, MoveLPath path, double[][] b, double u0, double u1,
        int depth, Stopwatch clock, CancellationToken cancel)
    {
        CheckBudget(clock, cancel);
        var bounds = b.Select(DerivativeBounds).ToArray();
        double v1 = bounds[3].Velocity, v2 = v1 + bounds[2].Velocity, v3 = v2 + bounds[1].Velocity;
        double a1 = bounds[3].Acceleration, a2 = a1 + bounds[2].Acceleration, a3 = a2 + bounds[1].Acceleration;
        double positionCurvature = .33 * (a1 + v1 * v1) + .38 * (a2 + v2 * v2) + .09 * (a3 + v3 * v3);
        var q0 = b.Select(axis => axis[0]).ToArray();
        var q1 = b.Select(axis => axis[^1]).ToArray();
        var actual0 = DiskKinematics.Forward(q0);
        var actual1 = DiskKinematics.Forward(q1);
        var expected0 = path.Pose(u0);
        var expected1 = path.Pose(u1);
        double width = u1 - u0;
        double progressCurvature = (path.Regularized ? 55 : 6) * width * width;
        double positionBound = Math.Max(PositionError(actual0, expected0), PositionError(actual1, expected1)) +
            (positionCurvature + PositionError(path.From, path.To) * progressCurvature) / 8;
        double pitchBound = Math.Max(Math.Abs(actual0.Pitch - expected0.Pitch), Math.Abs(actual1.Pitch - expected1.Pitch)) +
            (a3 + Math.Abs(path.To.Pitch - path.From.Pitch) * progressCurvature) / 8;
        var h0 = HeadingResidual(actual0.Pitch, q0[0], expected0.Yaw);
        var h1 = HeadingResidual(actual1.Pitch, q1[0], expected1.Yaw);
        double headingSpeed = v3 + bounds[0].Velocity + Math.Abs(path.YawDelta) * (path.Regularized ? 2.71 : 1.875) * width;
        double headingMargin = (a3 + bounds[0].Acceleration + Math.Abs(path.YawDelta) * progressCurvature +
            headingSpeed * headingSpeed) / 8;
        double headingDot = Math.Min(h0.Dot, h1.Dot) - headingMargin;
        double headingCross = Math.Max(Math.Abs(h0.Cross), Math.Abs(h1.Cross)) + headingMargin;
        Span<double> rounding = stackalloc double[4];
        for (int i = 0; i < 4; i++)
        {
            double low = b[i].Min(), high = b[i].Max();
            if (low < jointLimits[i].Low || high > jointLimits[i].High) return false;
            var (rawLow, rawHigh) = encoderLimits[i];
            rounding[i] = low == high
                ? Math.Abs(low - Math.Clamp(Math.Round(low / EncoderCount) * EncoderCount, rawLow, rawHigh))
                : Math.Max(EncoderCount / 2, Math.Max(rawLow - low, high - rawHigh));
        }
        double positionRounding = .8 * rounding[3] + .47 * rounding[2] + .09 * rounding[1];
        double pitchRounding = rounding[1] + rounding[2] + rounding[3];
        // A bound on the horizontal projection change caused by quantization.
        // The pitch interval uses the joint-curve derivative bound from q0.
        double sineBound = Math.Min(1, Math.Abs(Math.Sin(actual0.Pitch)) + v3 + pitchRounding);
        double headingRounding = rounding[0] + (sineBound + Math.Abs(Math.Cos(DiskKinematics.Alpha))) * pitchRounding;
        double xMargin = positionCurvature / 8 + positionRounding;
        headingDot -= headingRounding;
        if (positionBound + positionRounding <= PositionTolerance && pitchBound + pitchRounding <= AngleTolerance &&
            headingDot > 1e-6 && headingCross + headingRounding <= headingDot * Math.Tan(AngleTolerance) &&
            Math.Min(actual0.X, actual1.X) - xMargin >= -taskLimits.BackwardLimit &&
            Math.Max(actual0.X, actual1.X) + xMargin <= taskLimits.ForwardLimit)
            return true;

        if (depth >= 8 || PositionError(actual0, expected0) > PositionTolerance ||
            PositionError(actual1, expected1) > PositionTolerance) return false;
        var halves = b.Select(SplitBernstein).ToArray();
        double mid = (u0 + u1) / 2;
        return CertifyMoveL(taskLimits, jointLimits, encoderLimits, path, halves.Select(pair => pair.Left).ToArray(),
                u0, mid, depth + 1, clock, cancel)
            && CertifyMoveL(taskLimits, jointLimits, encoderLimits, path, halves.Select(pair => pair.Right).ToArray(),
                mid, u1, depth + 1, clock, cancel);
    }

    private static (double Dot, double Cross) HeadingResidual(double pitch, double top, double yaw)
    {
        var (x, y) = DiskKinematics.HorizontalForward(pitch, top);
        return (x * Math.Cos(yaw) + y * Math.Sin(yaw), y * Math.Cos(yaw) - x * Math.Sin(yaw));
    }

    private static void VerifyCspSamples(Robot robot, Calibration calibration, MotionPlan plan, MoveLPath path,
        Stopwatch clock, CancellationToken cancel)
    {
        short[]? previous = null;
        int samples = (int)Math.Ceiling(plan.Duration / .005);
        for (int k = 0; k <= samples; k++)
        {
            if ((k & 63) == 0) CheckBudget(clock, cancel);
            double t = Math.Min(plan.Duration, k * .005);
            var raw = calibration.Encode(robot, plan.At(t));
            var joints = raw.Select((value, i) => calibration.ToJoint(i, value)).ToArray();
            var disk = robot.Forward(joints)["bady_link_4"];
            var actual = DiskKinematics.Forward(joints);
            var expected = path.Pose(t / plan.Duration);
            double y = (expected.Z - .172) / Math.Tan(DiskKinematics.Alpha);
            double dx = disk.M41 - expected.X, dy = disk.M42 - y, dz = disk.M43 - expected.Z;
            if (Math.Sqrt(dx * dx + dy * dy + dz * dz) > PositionTolerance ||
                Math.Abs(actual.Pitch - expected.Pitch) > AngleTolerance ||
                Math.Abs(DiskKinematics.Wrap(Math.Atan2(disk.M12, disk.M11) - expected.Yaw)) > AngleTolerance)
                throw new ArgumentException("MoveL 量化后的指令位姿误差超限");
            if (previous is not null && raw.Zip(previous, (a, b) => Math.Abs(a - b)).Any(step => step > CspMaxStepRaw))
                throw new ArgumentException("MoveL 相邻 CSP 目标步长超限");
            previous = raw;
        }
        CheckBudget(clock, cancel);
    }

    private static void CheckBudget(Stopwatch clock, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();
        if (clock.Elapsed.TotalSeconds > 1) throw new TimeoutException("MoveL 路径验证超过 1 秒，请缩小动作");
    }
}
