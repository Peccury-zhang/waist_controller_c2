namespace WaistModel;

public readonly record struct MoveLWorkspace(double MinimumHeightMm, double MaximumHeightMm)
{
    public const double ZeroHeightMm = 972;
    public const double ReferenceMinimumMm = 466;

    public static MoveLWorkspace FromCalibration(Calibration calibration)
        => FromPlanningLimits(calibration.CreateWholeBodyPlanningLimits());

    internal static MoveLWorkspace FromPlanningLimits(Calibration calibration)
    {
        var bottom = calibration.Bounds(3);
        var middle = calibration.Bounds(2);
        var upper = calibration.Bounds(1);
        var yaw = calibration.Bounds(0);
        if (yaw.Low > 0 || yaw.High < 0) throw new ArgumentException("当前标定不包含中性航向");
        double betaLow = -upper.High, betaHigh = -upper.Low;
        double bendLow = Math.Max(0, middle.Low), bendHigh = middle.High;
        double minX = -calibration.TaskLimits.BackwardLimit, maxX = calibration.TaskLimits.ForwardLimit;
        TaskPose? lowest = null;
        const double epsilon = 1e-12;

        void Consider(double a, double beta)
        {
            double bend = beta - a;
            if (a < bottom.Low - epsilon || a > bottom.High + epsilon ||
                beta < betaLow - epsilon || beta > betaHigh + epsilon ||
                bend < bendLow - epsilon || bend > bendHigh + epsilon) return;
            var pose = DiskKinematics.Forward([0, -beta, bend, a]);
            if (pose.X >= minX - epsilon && pose.X <= maxX + epsilon &&
                (lowest is null || pose.Z < lowest.Value.Z)) lowest = pose;
        }

        // The joint limits form a polygon in (q1, beta). On each edge, the
        // varying links reduce to one sinusoid. Include both X-limit crossings
        // and all extrema of height on the edge.
        void Edge(double a, double beta, bool moveA, bool moveBeta, double low, double high)
        {
            if (low > high) return;
            double sine = (moveA ? .33 * Math.Sin(a) : 0) + (moveBeta ? .38 * Math.Sin(beta) : 0);
            double cosine = (moveA ? .33 * Math.Cos(a) : 0) + (moveBeta ? .38 * Math.Cos(beta) : 0);
            double offset = .066 + (moveA ? 0 : .33 * Math.Sin(a)) + (moveBeta ? 0 : .38 * Math.Sin(beta));
            double magnitude = Math.Sqrt(sine * sine + cosine * cosine);
            double phase = Math.Atan2(sine, cosine);
            void At(double t)
            {
                if (t >= low - epsilon && t <= high + epsilon)
                    Consider(a + (moveA ? t : 0), beta + (moveBeta ? t : 0));
            }
            At(low);
            At(high);
            for (int turn = -2; turn <= 2; turn++)
            {
                At(turn * Math.PI - phase);
                foreach (double x in new[] { minX, maxX })
                {
                    double ratio = (x - offset) / magnitude;
                    if (Math.Abs(ratio) > 1) continue;
                    double angle = Math.Asin(ratio), shift = turn * 2 * Math.PI - phase;
                    At(angle + shift);
                    At(Math.PI - angle + shift);
                }
            }
        }

        Consider(0, 0);
        foreach (double a in new[] { bottom.Low, bottom.High })
            Edge(a, 0, false, true, Math.Max(betaLow, a + bendLow), Math.Min(betaHigh, a + bendHigh));
        foreach (double beta in new[] { betaLow, betaHigh })
            Edge(0, beta, true, false, Math.Max(bottom.Low, beta - bendHigh), Math.Min(bottom.High, beta - bendLow));
        foreach (double bend in new[] { bendLow, bendHigh })
            Edge(0, bend, true, true, Math.Max(bottom.Low, betaLow - bend), Math.Min(bottom.High, betaHigh - bend));

        if (lowest is not { } floor) throw new ArgumentException("当前标定下没有中性姿态的可用高度");
        double minimum = Math.Ceiling(floor.Z * 1000 - 1e-8);
        // The tested operating floor still applies if the geometry permits a deeper fold.
        minimum = Math.Max(ReferenceMinimumMm, minimum);
        return new(minimum, ZeroHeightMm);
    }

    public void CheckHeight(double metres, double feedbackToleranceMm = 0)
    {
        double mm = metres * 1000;
        if (!double.IsFinite(mm) || mm < MinimumHeightMm - feedbackToleranceMm - 1e-7 ||
            mm > MaximumHeightMm + feedbackToleranceMm + 1e-7)
            throw new ArgumentException($"MoveL 目标高度须在 {MinimumHeightMm:F0}～{MaximumHeightMm:F0} mm");
    }
}
