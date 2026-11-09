using System.Diagnostics;

namespace WaistModel;

public sealed partial class MotionPlan
{
    private sealed record SmoothLeg(double[] From, double[] To, double Duration);
    private SmoothLeg[]? smoothLegs;
    public bool IsSmooth => smoothLegs is not null;
    public int SmoothLegCount => smoothLegs?.Length ?? 0;
    public double PlannedPeakRpm => IsSmooth ? smoothPeakRpm : MoveLPlannedPeakRpm;
    private double smoothPeakRpm;
    public TaskPose? ExpectedPose(double time) => IsSmooth ? DiskKinematics.Forward(At(time)) : ExpectedMoveLPose(time);
    public (double CrossTrackMm, double AlongErrorMm, double AngleErrorDeg)? TrackingDeviation(double time, TaskPose actual)
    {
        if (!IsSmooth) return MoveLDeviation(time, actual);
        var expected = DiskKinematics.Forward(At(time));
        double dx = actual.X - expected.X, dz = actual.Z - expected.Z;
        return (1000 * Math.Sqrt(dx * dx + dz * dz), 0,
            Math.Max(Math.Abs(actual.Pitch - expected.Pitch), Math.Abs(DiskKinematics.Wrap(actual.Yaw - expected.Yaw))) * 180 / Math.PI);
    }

    private double[] SmoothPosition(double time)
    {
        if (time <= 0) return Start;
        if (time >= Duration) return End;
        foreach (var leg in smoothLegs!)
        {
            if (time > leg.Duration) { time -= leg.Duration; continue; }
            double u = time / leg.Duration;
            // Symmetry avoids cancellation close to the endpoint.
            double v = Math.Min(u, 1 - u);
            double s = v * v * v * v * (35 + v * (-84 + v * (70 - 20 * v)));
            if (u > .5) s = 1 - s;
            return InterpolateJoints(leg.From, leg.To, s);
        }
        return End;
    }

    private static double[] InterpolateJoints(double[] a, double[] b, double s) =>
        a.Select((value, i) => Math.Clamp(value + (b[i] - value) * s,
            Math.Min(value, b[i]), Math.Max(value, b[i]))).ToArray();

    public static MotionPlan CreateSmooth(Robot robot, Calibration calibration, double[] start, TaskPose request,
        double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel = default,
        PositionPriority priority = PositionPriority.Both, bool keepPitch = true) =>
        CreateSmoothCore(robot, calibration, start, request, maxSpeedRpm, maxAccelerationRpmPerSec,
            cancel, priority, keepPitch, returnToZero: false);

    public static MotionPlan CreateSmoothToZero(Robot robot, Calibration calibration, double[] start,
        double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel = default) =>
        CreateSmoothCore(robot, calibration, start, DiskKinematics.Forward(new double[4]),
            maxSpeedRpm, maxAccelerationRpmPerSec, cancel, PositionPriority.Both, keepPitch: false, returnToZero: true);

    private static MotionPlan CreateSmoothCore(Robot robot, Calibration calibration, double[] start, TaskPose request,
        double maxSpeedRpm, double maxAccelerationRpmPerSec, CancellationToken cancel,
        PositionPriority priority, bool keepPitch, bool returnToZero)
    {
        ValidateOutputLimits(maxSpeedRpm, maxAccelerationRpmPerSec);
        ValidateModel(robot);
        var clock = Stopwatch.StartNew();
        var limits = calibration.CreateWholeBodyPlanningLimits();
        calibration.CheckWholeBodyStart(start);
        limits.CheckPlanningAngles(start);
        if (!returnToZero) RequirePositiveFoldStart(start);
        var workspace = MoveLWorkspace.FromPlanningLimits(limits);
        var from = DiskKinematics.Forward(start);
        workspace.CheckHeight(from.Z, feedbackToleranceMm: .1);
        if (keepPitch) request = request with { Pitch = from.Pitch };
        workspace.CheckHeight(request.Z);
        if (priority == PositionPriority.Height)
            request = ResolveCenteredEndpoint(calibration, request).Pose;
        // Fore/aft is an exact target. Height is never silently adjusted.
        var end = returnToZero ? new double[4] : DiskKinematics.Solve(limits, request, start, PositiveFold);
        // Numerical IK has a pose tolerance. Make the pitch constraint exact
        // before interpolating the joints, then recheck endpoint feasibility.
        end[1] = request.Pitch - end[2] - end[3];
        limits.CheckPlanningAngles(end);
        if (!DiskKinematics.Matches(request, end))
            throw new ArgumentException("目标位姿无法在当前限位内保持指定俯仰");
        calibration.Encode(robot, end);
        double speed = maxSpeedRpm * Math.PI / 30;
        double acceleration = maxAccelerationRpmPerSec * Math.PI / 30;
        var route = new List<double[]> { (double[])start.Clone(), end };
        if (!CertifySmoothEdge(calibration, workspace, start, end, clock, cancel))
            route = SearchSmoothRoute(calibration, limits, workspace, start, end, speed, acceleration, clock, cancel);
        var legs = route.Zip(route.Skip(1), (a, b) =>
            new SmoothLeg(a, b, SmoothDuration(a, b, speed, acceleration))).ToArray();
        double duration = legs.Sum(leg => leg.Duration);
        if (!double.IsFinite(duration) || duration > 60)
            throw new ArgumentException("平滑曲线超过 60 秒，请调整目标或速度");
        var plan = new MotionPlan(robot, calibration, start, end, duration, [], speed) { smoothLegs = legs };
        plan.smoothPeakRpm = legs.Max(leg => 2.1875 * leg.From.Zip(leg.To, (a, b) => Math.Abs(b - a)).Max()
            / leg.Duration * 30 / Math.PI);
        VerifySmoothSamples(robot, calibration, limits, workspace, plan, request, clock, cancel);
        return plan;
    }

    private static double SmoothDuration(double[] a, double[] b, double speed, double acceleration)
    {
        // Exact maximum derivatives of 35u^4 - 84u^5 + 70u^6 - 20u^7.
        double distance = a.Zip(b, (x, y) => Math.Abs(y - x)).Max();
        double v = 2.1875 * distance, acc = (84 * Math.Sqrt(5) / 25) * distance, jerk = 52.5 * distance;
        return Math.Max(.05, Math.Max(v / speed, Math.Max(StopAccelerationTime(v, acc, acceleration),
            StopJerkTime(v, acc, jerk, acceleration / AccelerationRampSeconds))));
    }

    private static void SmoothBudget(Stopwatch clock, CancellationToken cancel)
    {
        cancel.ThrowIfCancellationRequested();
        if (clock.Elapsed.TotalSeconds > 5)
            throw new TimeoutException("平滑曲线路径搜索超过 5 秒，本次未执行");
    }

    // Exact trigonometric interval bounds on an affine joint segment.
    private static (double Low, double High) TrigRange(double a, double b, bool cosine)
    {
        if (cosine) { a += Math.PI / 2; b += Math.PI / 2; }
        double low = Math.Min(a, b), high = Math.Max(a, b);
        double min = Math.Min(Math.Sin(low), Math.Sin(high)), max = Math.Max(Math.Sin(low), Math.Sin(high));
        if (high - low >= 2 * Math.PI) return (-1, 1);
        if (Math.Ceiling((low - Math.PI / 2) / (2 * Math.PI)) <= Math.Floor((high - Math.PI / 2) / (2 * Math.PI))) max = 1;
        if (Math.Ceiling((low + Math.PI / 2) / (2 * Math.PI)) <= Math.Floor((high + Math.PI / 2) / (2 * Math.PI))) min = -1;
        return (min, max);
    }

    private static bool CertifySmoothEdge(Calibration calibration, MoveLWorkspace workspace,
        double[] a, double[] b, Stopwatch clock, CancellationToken cancel, int depth = 0)
    {
        SmoothBudget(clock, cancel);
        double[] anglesA = [a[3], a[3] + a[2], a[3] + a[2] + a[1]];
        double[] anglesB = [b[3], b[3] + b[2], b[3] + b[2] + b[1]];
        double[] lengths = [.33, .38, .09];
        double xLow = .066, xHigh = .066, zLow = .172, zHigh = .172;
        for (int i = 0; i < 3; i++)
        {
            var sine = TrigRange(anglesA[i], anglesB[i], false);
            var cosine = TrigRange(anglesA[i], anglesB[i], true);
            xLow += lengths[i] * sine.Low; xHigh += lengths[i] * sine.High;
            double lengthZ = -Math.Sin(DiskKinematics.Alpha) * lengths[i];
            zLow += lengthZ * cosine.Low; zHigh += lengthZ * cosine.High;
        }
        // Half an encoder count per axis bounds output position roundoff.
        const double rounding = 1.36 * EncoderCount / 2;
        double minZ = (workspace.MinimumHeightMm - .1) / 1000;
        double maxZ = (workspace.MaximumHeightMm + .1) / 1000;
        double minX = -calibration.TaskLimits.BackwardLimit, maxX = calibration.TaskLimits.ForwardLimit;
        var mid = InterpolateJoints(a, b, .5);
        var heading = DiskKinematics.HorizontalForward(anglesA[2] / 2 + anglesB[2] / 2, mid[0]);
        double projection = Math.Sqrt(heading.X * heading.X + heading.Y * heading.Y);
        double headingRadius = (Math.Abs(anglesA[2] - anglesB[2]) + Math.Abs(a[0] - b[0])) / 2 + 2 * EncoderCount;
        if (xLow - rounding >= minX && xHigh + rounding <= maxX &&
            zLow - rounding >= minZ && zHigh + rounding <= maxZ && projection - headingRadius > 1e-6) return true;
        if (depth >= 18) return false;
        double midX = DiskKinematics.ForwardX(mid);
        double midZ = .172 - Math.Sin(DiskKinematics.Alpha) *
            (.33 * Math.Cos(mid[3]) + .38 * Math.Cos(mid[3] + mid[2]) + .09 * Math.Cos(mid[3] + mid[2] + mid[1]));
        if (midX < minX || midX > maxX || midZ < minZ || midZ > maxZ) return false;
        return CertifySmoothEdge(calibration, workspace, a, mid, clock, cancel, depth + 1) &&
            CertifySmoothEdge(calibration, workspace, mid, b, clock, cancel, depth + 1);
    }

    private static List<double[]> SearchSmoothRoute(Calibration calibration, Calibration limits,
        MoveLWorkspace workspace, double[] start, double[] end, double speed, double acceleration,
        Stopwatch clock, CancellationToken cancel)
    {
        // Search the two independent bending coordinates. For a pitch change,
        // interior nodes use the midpoint pitch; the entry/exit legs interpolate it.
        const int count = 25;
        double pitch = (start.Skip(1).Sum() + end.Skip(1).Sum()) / 2;
        var middle = limits.Bounds(2);
        middle.Low = Math.Max(0, middle.Low);
        var bottom = limits.Bounds(3);
        var nodes = new List<double[]> { start, end };
        var grid = new int[count, count];
        for (int row = 0; row < count; row++)
        for (int col = 0; col < count; col++)
        {
            grid[row, col] = -1;
            double q3 = middle.Low + (middle.High - middle.Low) * row / (count - 1);
            double q4 = bottom.Low + (bottom.High - bottom.Low) * col / (count - 1);
            double[] q = [(start[0] + end[0]) / 2, pitch - q3 - q4, q3, q4];
            var upper = limits.Bounds(1);
            if (q[1] < upper.Low || q[1] > upper.High ||
                !CertifySmoothEdge(calibration, workspace, q, q, clock, cancel)) continue;
            grid[row, col] = nodes.Count;
            nodes.Add(q);
        }
        var neighbors = Enumerable.Range(0, nodes.Count).Select(_ => new List<int>()).ToArray();
        for (int row = 0; row < count; row++)
        for (int col = 0; col < count; col++)
        {
            int index = grid[row, col];
            if (index < 0) continue;
            neighbors[0].Add(index);
            neighbors[index].Add(1);
            for (int dr = -1; dr <= 1; dr++)
            for (int dc = -1; dc <= 1; dc++)
            {
                int r = row + dr, c = col + dc;
                if ((dr != 0 || dc != 0) && r >= 0 && r < count && c >= 0 && c < count && grid[r, c] >= 0)
                    neighbors[index].Add(grid[r, c]);
            }
        }
        var cost = Enumerable.Repeat(double.PositiveInfinity, nodes.Count).ToArray();
        var parent = Enumerable.Repeat(-1, nodes.Count).ToArray();
        var queue = new PriorityQueue<int, double>();
        cost[0] = 0;
        queue.Enqueue(0, 0);
        while (queue.TryDequeue(out int current, out double distance))
        {
            SmoothBudget(clock, cancel);
            if (distance > cost[current]) continue;
            if (current == 1) break;
            foreach (int next in neighbors[current])
            {
                double nextCost = distance + SmoothDuration(nodes[current], nodes[next], speed, acceleration);
                if (nextCost >= cost[next] ||
                    !CertifySmoothEdge(calibration, workspace, nodes[current], nodes[next], clock, cancel)) continue;
                cost[next] = nextCost; parent[next] = current; queue.Enqueue(next, nextCost);
            }
        }
        if (parent[1] < 0) throw new ArgumentException("当前限位和姿态下未找到通过验证的曲线路径");
        var route = new List<double[]>();
        for (int i = 1; i >= 0; i = parent[i]) route.Add(nodes[i]);
        route.Reverse();
        // Remove unnecessary grid stops whenever the direct replacement is valid.
        for (int i = 0; i < route.Count - 2; i++)
        for (int j = route.Count - 1; j > i + 1; j--)
        {
            if (!CertifySmoothEdge(calibration, workspace, route[i], route[j], clock, cancel)) continue;
            route.RemoveRange(i + 1, j - i - 1);
            break;
        }
        return route;
    }

    private static void VerifySmoothSamples(Robot robot, Calibration calibration, Calibration limits,
        MoveLWorkspace workspace, MotionPlan plan, TaskPose target, Stopwatch clock, CancellationToken cancel)
    {
        short[]? previous = null;
        int samples = (int)Math.Ceiling(plan.Duration / .005);
        for (int k = 0; k <= samples; k++)
        {
            SmoothBudget(clock, cancel);
            var q = plan.At(Math.Min(plan.Duration, k * .005));
            limits.CheckPlanningAngles(q);
            var raw = calibration.Encode(robot, q);
            var encoded = raw.Select((value, i) => calibration.ToJoint(i, value)).ToArray();
            limits.CheckPlanningAngles(encoded);
            var pose = DiskKinematics.Forward(encoded);
            calibration.CheckX(pose.X);
            workspace.CheckHeight(pose.Z, feedbackToleranceMm: .1);
            if (Math.Abs(pose.Pitch - q.Skip(1).Sum()) > AngleTolerance)
                throw new ArgumentException("曲线编码后的俯仰误差超限");
            if (previous is not null && raw.Zip(previous, (a, b) => Math.Abs(a - b)).Any(step => step > CspMaxStepRaw))
                throw new ArgumentException("曲线相邻 CSP 目标步长超限");
            previous = raw;
        }
        var final = calibration.Encode(robot, plan.End).Select((value, i) => calibration.ToJoint(i, value)).ToArray();
        if (!DiskKinematics.Matches(target, final)) throw new ArgumentException("曲线终点编码后位姿误差超限");
    }
}
