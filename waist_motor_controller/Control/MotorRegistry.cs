using System.Text;
using System.Text.Encodings.Web;
using System.Text.Json;

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
    public bool UseFirmware117Pdo { get; set; } = true;
    public string Model { get; set; } = "";
    public string ExpectedFirmware { get; set; } = "";
}
public sealed class ProfileConfig
{
    public double JogStepDeg { get; set; } = 1;
    public double OutputSpeedRpm { get; set; } = 1;
    public double OutputAccelRpmPerSec { get; set; } = 5;
}
public sealed class AppConfig
{
    public int SchemaVersion { get; set; } = 1;
    public CanConfig Can { get; set; } = new();
    public List<MotorConfig> Motors { get; set; } = new();
    public ProfileConfig Profile { get; set; } = new();
    public static AppConfig CreateDefault() => new()
    {
        SchemaVersion = 2,
        Motors = new()
        {
            new() { LogicalId = 1, Name = "最上部腰部旋转", DeviceId = 3, Model = "R120", ExpectedFirmware = "V1.17.12" },
            new() { LogicalId = 2, Name = "折叠腿高", DeviceId = 4, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
            new() { LogicalId = 3, Name = "折叠腿中", DeviceId = 5, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
            new() { LogicalId = 4, Name = "折叠腿最底部", DeviceId = 6, Model = "R120MAX", ExpectedFirmware = "V1.17.13" },
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
            if (!File.Exists(_path)) { LoadMessage = "使用新版默认映射：3、4、5、6"; return; }
            var cfg = JsonSerializer.Deserialize<AppConfig>(File.ReadAllText(_path, Encoding.UTF8));
            if (cfg?.Motors == null || cfg.Motors.Count != 4 || cfg.Can == null || cfg.Profile == null ||
                !cfg.Motors.Select(m => m.LogicalId).Order().SequenceEqual(new[] { 1, 2, 3, 4 }) ||
                cfg.Motors.Any(m => m.DeviceId is < 1 or > 127) || cfg.Motors.Select(m => m.DeviceId).Distinct().Count() != 4)
                throw new InvalidDataException("配置中四个逻辑ID或CAN ID无效");
            cfg.Motors = cfg.Motors.OrderBy(m => m.LogicalId).ToList();
            bool legacy = cfg.SchemaVersion < 2;
            var defaults = AppConfig.CreateDefault();
            if (legacy)
            {
                // 只迁移完整旧默认映射，保留用户自定义ID；旧速度单位不同，重置为低速默认值。
                bool oldIds = cfg.Motors.Select(m => m.DeviceId).SequenceEqual(new[] { 11, 12, 13, 14 });
                for (int i = 0; i < 4; i++)
                {
                    if (oldIds) cfg.Motors[i].DeviceId = defaults.Motors[i].DeviceId;
                    cfg.Motors[i].Name = defaults.Motors[i].Name;
                    cfg.Motors[i].Model = defaults.Motors[i].Model;
                    cfg.Motors[i].ExpectedFirmware = defaults.Motors[i].ExpectedFirmware;
                }
                cfg.Profile = new();
                cfg.SchemaVersion = 2;
                if (!File.Exists(_path + ".pre-v2.bak")) File.Copy(_path, _path + ".pre-v2.bak");
                LoadMessage = oldIds ? "旧配置已备份并迁移：11/12/13/14 → 3/4/5/6；速度重置为新版默认值" :
                    "旧配置已备份；保留自定义ID，运动参数改用新版输出端单位";
            }
            else LoadMessage = "已加载新版配置";
            Config = cfg;
            if (legacy) Save();
        }
        catch (Exception ex)
        {
            Config = AppConfig.CreateDefault();
            LoadMessage = $"配置读取/迁移失败：{ex.Message}；使用默认映射3/4/5/6";
        }
    }
    public bool Save()
    {
        try
        {
            var dir = Path.GetDirectoryName(_path);
            if (!string.IsNullOrEmpty(dir)) Directory.CreateDirectory(dir);
            File.WriteAllText(_path, JsonSerializer.Serialize(Config, JsonOpts), Encoding.UTF8);
            Log?.Invoke("配置已保存");
            return true;
        }
        catch (Exception ex) { Log?.Invoke("配置保存失败：" + ex.Message); return false; }
    }
}
