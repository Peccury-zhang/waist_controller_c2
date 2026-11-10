using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Serialization;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.Control;

public sealed class CanConfig
{
    public int DeviceIndex { get; set; }
    public int Channel { get; set; }
}
public sealed class MotorConfig
{
    public int LogicalId { get; set; }
    public string Name { get; set; } = "";
    public int DeviceId { get; set; }
    public int Direction { get; set; }
    public bool UseFirmware117Pdo { get; set; } = true;
    public string Model { get; set; } = "";
    public string ExpectedFirmware { get; set; } = "";
    public HeadAngleRange? HeadLimits { get; set; }
    public ProfileConfig? Profile { get; set; }
    [JsonIgnore] public bool IsHead => LogicalId is 5 or 6;
    [JsonIgnore] public string? HeadJoint => LogicalId switch { 5 => "head_joint_2", 6 => "head_joint_1", _ => null };
}

public sealed record HeadAngleRange(double Minimum, double Maximum)
{
    public void Validate()
    {
        if (!double.IsFinite(Minimum) || !double.IsFinite(Maximum) || Minimum >= Maximum ||
            Minimum < Od.MinAngle || Maximum > Od.MaxAngle ||
            Math.Ceiling(Minimum * Od.CountsPerDeg) > Math.Floor(Maximum * Od.CountsPerDeg))
            throw new ArgumentException("头部角度范围无效，请检查上下限和编码范围");
    }
}
public sealed class ProfileConfig
{
    public double TargetDeg { get; set; }
    public double JogStepDeg { get; set; } = 1;
    public double OutputSpeedRpm { get; set; } = 1;
    public double OutputAccelRpmPerSec { get; set; } = 5;
}
public sealed class AppConfig
{
    public const int WaistAxisCount = 4;
    public const int MotorCount = 6;
    public int SchemaVersion { get; set; } = 1;
    public CanConfig Can { get; set; } = new();
    public List<MotorConfig> Motors { get; set; } = new();
    public ProfileConfig Profile { get; set; } = new();
    public static AppConfig CreateDefault() => new()
    {
        SchemaVersion = 3,
        Motors = new()
        {
            new() { LogicalId = 1, Name = "最上部腰部旋转", DeviceId = 3, Direction = -1, Model = "R120", ExpectedFirmware = "V1.17.12" },
            new() { LogicalId = 2, Name = "折叠腿高", DeviceId = 4, Direction = -1, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
            new() { LogicalId = 3, Name = "折叠腿中", DeviceId = 5, Direction = 1, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
            new() { LogicalId = 4, Name = "折叠腿最底部", DeviceId = 6, Direction = -1, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
            new() { LogicalId = 5, Name = "头部点头", DeviceId = 1, Direction = 1, Model = "R48", HeadLimits = new(-40, 40) },
            new() { LogicalId = 6, Name = "头部摇头", DeviceId = 2, Direction = 1, Model = "R48", HeadLimits = new(0, 37) },
        }
    };
}

public sealed class MotorRegistry
{
    private static readonly JsonSerializerOptions JsonOpts = new()
    { WriteIndented = true, Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping };
    private readonly string _path;
    public AppConfig Config { get; private set; } = AppConfig.CreateDefault();
    public string LoadMessage { get; private set; } = "";
    public event Action<string>? Log;
    public MotorRegistry(string configPath) { _path = configPath; Load(); }
    public IReadOnlyList<MotorConfig> Motors => Config.Motors;
    public IEnumerable<MotorConfig> WaistMotors => Motors.Take(AppConfig.WaistAxisCount);
    public IEnumerable<MotorConfig> HeadMotors => Motors.Skip(AppConfig.WaistAxisCount);
    public bool HasUniqueDeviceIds => Motors.Select(m => m.DeviceId).Distinct().Count() == Motors.Count;
    public MotorConfig GetMotor(int logicalId) => Config.Motors.First(m => m.LogicalId == logicalId);
    public int GetDeviceId(int logicalId) => GetMotor(logicalId).DeviceId;
    public void SetDeviceId(int logicalId, int deviceId)
    {
        if (deviceId is < 1 or > 127) throw new ArgumentOutOfRangeException(nameof(deviceId));
        GetMotor(logicalId).DeviceId = deviceId;
    }
    public void Load()
    {
        try
        {
            if (!File.Exists(_path)) { LoadMessage = "使用默认映射：腰部 3～6，头部 1、2"; return; }
            var cfg = JsonSerializer.Deserialize<AppConfig>(File.ReadAllText(_path, Encoding.UTF8));
            if (cfg?.Motors == null || cfg.Motors.Count is not (AppConfig.WaistAxisCount or AppConfig.MotorCount) ||
                cfg.Can == null || cfg.Profile == null ||
                !cfg.Motors.Select(m => m.LogicalId).Order().SequenceEqual(Enumerable.Range(1, cfg.Motors.Count)) ||
                cfg.Motors.Any(m => m.DeviceId is < 1 or > 127) ||
                cfg.Motors.Select(m => m.DeviceId).Distinct().Count() != cfg.Motors.Count)
                throw new InvalidDataException("配置中的逻辑 ID 或 CAN ID 无效");
            cfg.Motors = cfg.Motors.OrderBy(m => m.LogicalId).ToList();
            bool legacy = cfg.SchemaVersion < 2;
            var defaults = AppConfig.CreateDefault();
            for (int i = 0; i < cfg.Motors.Count; i++)
                if (cfg.Motors[i].Direction is not (-1 or 1)) cfg.Motors[i].Direction = defaults.Motors[i].Direction;
            if (legacy)
            {
                // 只迁移完整旧默认映射，保留用户自定义ID；旧速度单位不同，重置为低速默认值。
                bool oldIds = cfg.Motors.Select(m => m.DeviceId).SequenceEqual(new[] { 11, 12, 13, 14 });
                for (int i = 0; i < AppConfig.WaistAxisCount; i++)
                {
                    if (oldIds) cfg.Motors[i].DeviceId = defaults.Motors[i].DeviceId;
                    cfg.Motors[i].Name = defaults.Motors[i].Name;
                    cfg.Motors[i].Model = defaults.Motors[i].Model;
                    cfg.Motors[i].ExpectedFirmware = defaults.Motors[i].ExpectedFirmware;
                }
                cfg.Profile = new();
                if (!File.Exists(_path + ".pre-v2.bak")) File.Copy(_path, _path + ".pre-v2.bak");
                LoadMessage = oldIds ? "旧配置已备份并迁移：11/12/13/14 → 3/4/5/6；速度重置为新版默认值" :
                    "旧配置已备份；保留自定义ID，运动参数改用新版输出端单位";
            }
            else LoadMessage = "已加载设备配置";
            if (cfg.Motors.Count == AppConfig.WaistAxisCount)
            {
                cfg.Motors.AddRange(defaults.Motors.Where(m => m.IsHead));
                LoadMessage += " · 已加入默认头部 ID 1、2";
            }
            foreach (var motor in cfg.Motors.Where(m => m.IsHead))
            {
                var defaultMotor = defaults.Motors[motor.LogicalId - 1];
                motor.Model = defaultMotor.Model;
                motor.Name = defaultMotor.Name;
                motor.UseFirmware117Pdo = true;
                motor.HeadLimits ??= defaultMotor.HeadLimits;
                motor.HeadLimits!.Validate();
            }
            cfg.SchemaVersion = 3;
            Config = cfg;
            if (!HasUniqueDeviceIds) LoadMessage += " · 头部默认 ID 与已有映射重复，请先修正并保存映射";
            else if (legacy) Save();
        }
        catch (Exception ex)
        {
            Config = AppConfig.CreateDefault();
            LoadMessage = $"配置读取/迁移失败：{ex.Message}；使用默认映射（腰部 3～6，头部 1、2）";
        }
    }
    public bool Save()
    {
        try
        {
            if (!HasUniqueDeviceIds) throw new InvalidOperationException("CAN ID 重复，请先修正映射");
            var dir = Path.GetDirectoryName(_path);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            string temporary = _path + "." + Guid.NewGuid().ToString("N") + ".tmp";
            try
            {
                File.WriteAllText(temporary, JsonSerializer.Serialize(Config, JsonOpts), Encoding.UTF8);
                File.Move(temporary, _path, overwrite: true);
            }
            finally { File.Delete(temporary); }
            Log?.Invoke("配置已保存");
            return true;
        }
        catch (Exception ex) { Log?.Invoke("配置保存失败：" + ex.Message); return false; }
    }
}
