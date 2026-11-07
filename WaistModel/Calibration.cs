using System.Text.Json;
using System.Text.Json.Serialization;
using System.Security.Cryptography;
namespace WaistModel;
public sealed record MotorLimit(int LogicalId,double MinAngleDeg,double MaxAngleDeg);
public sealed class TaskLimits
{
    public string ReferenceFrame {get;set;}="base_link";
    public string ControlledLink {get;set;}="bady_link_4";
    public string DistanceUnit {get;set;}="m";
    public double ForwardLimit {get;set;}=.35;
    public double BackwardLimit {get;set;}=.50;
}
public sealed class Calibration
{
    public const double WholeBodyPlanningMarginDeg = .3;

    public int SchemaVersion {get;set;}=2;
    public string AngleUnit {get;set;}="deg";
    public string AngleConvention {get;set;}="joint_control_display";
    public MotorLimit[] Motors {get;set;}=[];
    public TaskLimits TaskLimits {get;set;}=new();
    [JsonIgnore] public int[] Signs {get;set;}=[-1,-1,1,-1];
    [JsonIgnore] public string Fingerprint=>Convert.ToHexString(SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(JsonSerializer.Serialize(this,CalibrationStore.Options)+string.Join(",",Signs))));
    public void Validate(Robot? robot=null)
    {
        const double max=32568*360.0/65536;
        if(SchemaVersion!=2||AngleUnit!="deg"||AngleConvention!="joint_control_display"||Motors==null||Motors.Length!=4||!Motors.Select(m=>m.LogicalId).Order().SequenceEqual(new[]{1,2,3,4})||Motors.Any(m=>!double.IsFinite(m.MinAngleDeg)||!double.IsFinite(m.MaxAngleDeg)||m.MinAngleDeg>=m.MaxAngleDeg||m.MinAngleDeg < -max||m.MaxAngleDeg>max))throw new InvalidDataException("四轴范围不完整、顺序重复或超出可编码角度范围");
        if(Signs.Length!=4||Signs.Any(x=>Math.Abs(x)!=1))throw new InvalidDataException("电机方向无效");
        for(int i=0;i<4;i++){var bounds=RawLimits(null,i);if(bounds.Low>bounds.High)throw new InvalidDataException("角度区间小于编码分辨率");}
        var t=TaskLimits;if(t==null||t.ReferenceFrame!="base_link"||t.ControlledLink!="bady_link_4"||t.DistanceUnit!="m"||!double.IsFinite(t.ForwardLimit)||!double.IsFinite(t.BackwardLimit)||t.ForwardLimit<0||t.ForwardLimit>.35||t.BackwardLimit<0||t.BackwardLimit>.5||t.ForwardLimit+t.BackwardLimit<=0)throw new InvalidDataException("前向须在0～350mm、后向须在0～500mm，不能同时为0");
    }
    // The top motor's display convention is positive about +Z when upright.
    // Its URDF axis (local +Y) points toward base -Z in that posture.
    private static int DisplayToUrdfSign(int i) => i == 0 ? -1 : 1;

    public (double Low, double High) Bounds(int i)
    {
        var motor = Motors.Single(m => m.LogicalId == i + 1);
        double low = motor.MinAngleDeg * Math.PI / 180;
        double high = motor.MaxAngleDeg * Math.PI / 180;
        return DisplayToUrdfSign(i) < 0 ? (-high, -low) : (low, high);
    }

    public static double FeedbackToUrdf(int i, int raw, int displayDirection) =>
        DisplayToUrdfSign(i) * displayDirection * raw * (2 * Math.PI / 65536);

    public double ToJoint(int i, int raw) => FeedbackToUrdf(i, raw, Signs[i]);

    private double JointToRaw(int i, double radians) =>
        DisplayToUrdfSign(i) * Signs[i] * radians * (65536 / (2 * Math.PI));
    internal Calibration CreateWholeBodyPlanningLimits()
    {
        Validate();
        // The executor rounds on the original grid. Reserve an extra half
        // count so rounding in either direction still leaves a 0.3° gap.
        const double inset = WholeBodyPlanningMarginDeg + 180.0 / 65536 + 1e-9;
        var motors = Motors.Select(motor => new MotorLimit(motor.LogicalId,
            motor.MinAngleDeg + inset, motor.MaxAngleDeg - inset)).ToArray();
        var unusable = motors.FirstOrDefault(motor => motor.MinAngleDeg >= motor.MaxAngleDeg);
        if (unusable is not null)
            throw new ArgumentException($"轴{unusable.LogicalId}标定区间不足以在两侧各预留 0.3°");
        return new Calibration { Motors = motors, Signs = (int[])Signs.Clone(), TaskLimits = TaskLimits };
    }

    public void Check(double[] q,bool task=true)
        => CheckRange(q, task, "标定");

    internal void CheckPlanningAngles(double[] q)
        => CheckRange(q, false, "规划范围（两侧各留 0.3°）");

    private void CheckRange(double[] q, bool task, string rangeName)
    {
        if(q.Length!=4)throw new ArgumentException("需要四轴");
        for (int i = 0; i < 4; i++)
        {
            var bounds = Bounds(i);
            if (double.IsFinite(q[i]) && q[i] >= bounds.Low && q[i] <= bounds.High) continue;
            var motor = Motors.Single(m => m.LogicalId == i + 1);
            double displayDeg = DisplayToUrdfSign(i) * q[i] * 180 / Math.PI;
            throw new ArgumentException($"轴{i + 1}关节角 {displayDeg:F4}°，超出{rangeName} [{motor.MinAngleDeg:0.####}, {motor.MaxAngleDeg:0.####}]°");
        }
        if(task)CheckX(DiskKinematics.ForwardX(q));
    }
    public void CheckX(double x){if(!double.IsFinite(x)||x < -TaskLimits.BackwardLimit||x>TaskLimits.ForwardLimit)throw new ArgumentException($"圆盘前后 {x*1000:F2}mm 超出标定范围 {-TaskLimits.BackwardLimit*1000:F1}～{TaskLimits.ForwardLimit*1000:F1}mm");}
    public (int Low, int High) RawLimits(Robot? robot, int i)
    {
        var bounds = Bounds(i);
        double a = JointToRaw(i, bounds.Low), b = JointToRaw(i, bounds.High);
        return ((int)Math.Ceiling(Math.Min(a, b)), (int)Math.Floor(Math.Max(a, b)));
    }
    public short[] Encode(Robot? robot,double[] q)
    {
        Check(q);
        var raw = new short[4];
        for (int i = 0; i < 4; i++)
        {
            var bounds = RawLimits(robot, i);
            if (bounds.Low > bounds.High) throw new ArgumentException("标定区间小于编码分辨率");
            raw[i] = checked((short)Math.Clamp(Math.Round(JointToRaw(i, q[i])), bounds.Low, bounds.High));
        }
        Check(raw.Select((value, i) => ToJoint(i, value)).ToArray());
        return raw;
    }
}
public sealed class CalibrationStore
{
    public const string DefaultPath="C:/Users/15401/Desktop/motorcontrol_apps/waist_controller/waist_controller_c2/waist_motor_controller/config/calibration.json";
    public static readonly JsonSerializerOptions Options=new(){PropertyNamingPolicy=JsonNamingPolicy.CamelCase,PropertyNameCaseInsensitive=true,WriteIndented=true};
    public string FilePath {get;}
    public CalibrationStore(string? path=null){FilePath=path??DefaultPath;}
    public Calibration Load(int[] signs){if(!File.Exists(FilePath))throw new InvalidDataException("当前未标定");var c=JsonSerializer.Deserialize<Calibration>(File.ReadAllText(FilePath),Options)??throw new InvalidDataException("空标定文件");c.Signs=(int[])signs.Clone();c.Validate();return c;}
    public void Save(Calibration c){c.Validate();Directory.CreateDirectory(Path.GetDirectoryName(FilePath)!);string temp=FilePath+".tmp";try{File.WriteAllText(temp,JsonSerializer.Serialize(c,Options));File.Move(temp,FilePath,true);}finally{if(File.Exists(temp))File.Delete(temp);}}
}
