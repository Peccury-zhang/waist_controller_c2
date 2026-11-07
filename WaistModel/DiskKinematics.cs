using System.Diagnostics;
using MathNet.Numerics.LinearAlgebra;
namespace WaistModel;
public readonly record struct TaskPose(double X,double Z,double Pitch,double Yaw);
public enum PositionPriority { Both, Height, ForeAft }

public static class DiskKinematics
{
    internal const double Alpha=-1.5708;
    public static double Wrap(double a)=>Math.Atan2(Math.Sin(a),Math.Cos(a));
    internal static double ForwardX(double[] q) =>
        .066 + .33 * Math.Sin(q[3]) + .38 * Math.Sin(q[3] + q[2]) + .09 * Math.Sin(q[3] + q[2] + q[1]);
    public static TaskPose Forward(double[] q)
    {
        double p=q[3]+q[2]+q[1],v=.33*Math.Cos(q[3])+.38*Math.Cos(q[3]+q[2])+.09*Math.Cos(p);
        var (fx, fy) = HorizontalForward(p, q[0]);
        if(fx*fx+fy*fy<1e-12)throw new ArgumentException("当前姿态下航向不确定");
        return new(ForwardX(q),.172-Math.Sin(Alpha)*v,p,Math.Atan2(fy,fx));
    }
    internal static (double X, double Y) HorizontalForward(double pitch, double top) =>
        (Math.Cos(pitch) * Math.Cos(top), Math.Cos(Alpha) * Math.Sin(pitch) * Math.Cos(top) + Math.Sin(Alpha) * Math.Sin(top));
    static double[] Error(TaskPose target,TaskPose value)=>[(target.X-value.X)/.0001,(target.Z-value.Z)/.0001,Wrap(target.Pitch-value.Pitch)/.000174532925,Wrap(target.Yaw-value.Yaw)/.000174532925];
    public static bool Matches(TaskPose t,double[] q)=>Error(t,Forward(q)).All(x=>Math.Abs(x)<=1);
    public static double[] Solve(Calibration c,TaskPose p,double[] seed,int branch=0,bool numericalOnly=false)
    {
        if(new[]{p.X,p.Z,p.Pitch,p.Yaw}.Any(x=>!double.IsFinite(x)))throw new ArgumentException("目标必须为有限数值");c.CheckX(p.X);
        if (!numericalOnly)
        {
            var solution = FindAnalytic(c, p, seed, branch, out bool reachable);
            if (solution is not null) return solution;
            if (!reachable) throw new ArgumentException("高度与俯仰组合几何不可达");
        }
        return Numerical(c,p,seed,branch);
    }

    internal static double[]? FindAnalytic(Calibration c, TaskPose p, double[] seed, int branch, out bool reachable)
    {
        var (offsetX, offsetZ, scaleZ) = WristFrame(p.Pitch);
        double x = p.X - offsetX, z = (p.Z - offsetZ) / scaleZ;
        double cos = (x * x + z * z - .33 * .33 - .38 * .38) / (2 * .33 * .38);
        reachable = cos >= -1 - 1e-10 && cos <= 1 + 1e-10;
        if (!reachable || p.X < -c.TaskLimits.BackwardLimit || p.X > c.TaskLimits.ForwardLimit) return null;

        var candidates = new List<double[]>();
        var bounds = Enumerable.Range(0, 4).Select(c.Bounds).ToArray();
        double a = Math.Sin(p.Yaw) * Math.Cos(p.Pitch) - Math.Cos(p.Yaw) * Math.Cos(Alpha) * Math.Sin(p.Pitch);
        double b = Math.Cos(p.Yaw) * Math.Sin(Alpha);
        foreach (int sign in new[] { 1, -1 })
        {
            if (branch != 0 && sign != branch) continue;
            double q2 = sign * Math.Acos(Math.Clamp(cos, -1, 1));
            double q1 = Wrap(Math.Atan2(x, z) - Math.Atan2(.38 * Math.Sin(q2), .33 + .38 * Math.Cos(q2)));
            for (int k = -2; k <= 2; k++)
            {
                double[] q = [Math.Atan2(a, b) + k * Math.PI, p.Pitch - q1 - q2, q2, q1];
                if (q.Where((value, i) => value < bounds[i].Low || value > bounds[i].High).Any()) continue;
                double actualX = ForwardX(q);
                if (actualX < -c.TaskLimits.BackwardLimit || actualX > c.TaskLimits.ForwardLimit) continue;
                try { if (Matches(p, q)) candidates.Add(q); }
                catch (ArgumentException) { } // Heading can be undefined at a singular orientation.
            }
        }
        return candidates.MinBy(q => q.Select((v, i) => (v - seed[i]) * (v - seed[i])).Sum());
    }

    private static (double X, double Z, double ScaleZ) WristFrame(double pitch)
    {
        double scaleZ = -Math.Sin(Alpha);
        return (.066 + .09 * Math.Sin(pitch), .172 + scaleZ * .09 * Math.Cos(pitch), scaleZ);
    }

    internal static IEnumerable<TaskPose> PositionAlternatives(Calibration calibration, TaskPose target, PositionPriority priority, int branch = 1)
    {
        var (offsetX, offsetZ, scaleZ) = WristFrame(target.Pitch);
        bool keepHeight = priority == PositionPriority.Height;
        double fixedValue = keepHeight ? (target.Z - offsetZ) / scaleZ : target.X - offsetX;
        double preferred = keepHeight ? target.X : target.Z;
        double low = keepHeight ? -calibration.TaskLimits.BackwardLimit : offsetZ - .71 * scaleZ;
        double high = keepHeight ? calibration.TaskLimits.ForwardLimit : offsetZ + .71 * scaleZ;
        var edges = new SortedSet<double> { low, high };

        // For fixed pitch, the endpoint at each active joint limit traces a
        // circle in the planar wrist frame. Intersect it with the fixed coordinate.
        void Intersect(double centerX, double centerZ, double radius)
        {
            double offset = fixedValue - (keepHeight ? centerZ : centerX);
            double square = radius * radius - offset * offset;
            if (square < -1e-14) return;
            double other = keepHeight ? centerX : centerZ;
            double delta = Math.Sqrt(Math.Max(0, square));
            foreach (double coordinate in new[] { other - delta, other + delta })
            {
                double world = keepHeight ? offsetX + coordinate : offsetZ + scaleZ * coordinate;
                if (world >= low && world <= high) edges.Add(world);
            }
        }

        var middle = calibration.Bounds(2);
        var bend = branch < 0 ? (middle.Low, Math.Min(0, middle.High)) : (Math.Max(0, middle.Low), middle.High);
        foreach (double q in new[] { bend.Item1, bend.Item2 })
            Intersect(0, 0, Math.Sqrt(.33 * .33 + .38 * .38 + 2 * .33 * .38 * Math.Cos(q)));
        var bottom = calibration.Bounds(3);
        foreach (double q in new[] { bottom.Low, bottom.High })
            Intersect(.33 * Math.Sin(q), .33 * Math.Cos(q), .38);
        var upper = calibration.Bounds(1);
        foreach (double q in new[] { target.Pitch - upper.Low, target.Pitch - upper.High })
            Intersect(.38 * Math.Sin(q), .38 * Math.Cos(q), .33);

        var boundaries = edges.ToArray();
        for (int i = 1; i < boundaries.Length; i++)
        {
            double left = boundaries[i - 1], right = boundaries[i];
            edges.Add((left + right) / 2); // Covers narrow feasible intervals.
            edges.Add(Math.Clamp(preferred, left, right));
            edges.Add(Math.Min(right, left + 1e-7));
            edges.Add(Math.Max(left, right - 1e-7));
            const double step = .00025;
            for (double value = left + step; value < right; value += step) edges.Add(value);
        }
        return edges.OrderBy(value => Math.Abs(value - preferred))
            .Select(value => keepHeight ? target with { X = value } : target with { Z = value });
    }
    public static double[] Numerical(Calibration c,TaskPose target,double[] seed,int branch=0)
    {
        var clock=Stopwatch.StartNew();
        for(int attempt=0;attempt<8;attempt++)
        {
            double[] q=seed.Select((v,i)=>{var b=c.Bounds(i);return Math.Clamp(attempt==0?v:v+(.12*(attempt/2+1)*(attempt%2==0?1:-1))*(i==2?1:.3),b.Low,b.High);}).ToArray();
            for(int iter=0;iter<30;iter++)
            {
                if(clock.Elapsed.TotalMilliseconds>10)throw new TimeoutException("二级 IK 超过10ms计算预算，本次未执行");
                var value=Forward(q);var e=Vector<double>.Build.DenseOfArray(Error(target,value));
                try{c.Check(q);if(e.All(v=>Math.Abs(v)<=1)&&branch*q[2]>=0)return q;}catch(ArgumentException){}
                var j=Matrix<double>.Build.Dense(4,4);
                for(int i=0;i<4;i++){var next=(double[])q.Clone();next[i]+=1e-6;var de=Error(Forward(next),value);for(int row=0;row<4;row++)j[row,i]=de[row]/1e-6;}
                var a=j.TransposeThisAndMultiply(j)+Matrix<double>.Build.DenseIdentity(4)*.1;var step=a.Svd().Solve(j.TransposeThisAndMultiply(e));
                double scale=Math.Min(1,.15/Math.Max(.15,step.L2Norm()));bool accepted=false;
                for(int ls=0;ls<12;ls++,scale*=.5){var next=q.Select((v,i)=>{var b=c.Bounds(i);return Math.Clamp(v+step[i]*scale,b.Low,b.High);}).ToArray();if(Error(target,Forward(next)).Sum(v=>v*v)<e.DotProduct(e)){q=next;accepted=true;break;}}
                if(!accepted)break;
            }
        }
        throw new ArgumentException("IK 无满足标定范围的解；本次未执行");
    }
}
