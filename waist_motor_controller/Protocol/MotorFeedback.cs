namespace waist_motor_controller.Protocol;

public sealed class MotorFeedback
{
    public bool IsFirmware117 { get; init; }
    public int DeviceId { get; init; }
    public short PositionRaw { get; init; }
    public double PositionDeg => Od.RawToAngle(PositionRaw);
    public short VelocityRpm { get; init; } // 输入端 rpm
    public short CurrentMa { get; init; }
    public ushort FaultCode { get; init; }
    // 确认协议为0.1°C；备用PDF分支未给缩放因子，保留其原始值。
    public short TemperatureRaw { get; init; }
    public byte Mode { get; init; }
    public bool Enabled { get; init; }
    public bool BrakeReleased { get; init; }
    public bool HasError { get; init; }
    /// <summary>确认协议B11.bit4；备用协议没有此位时为null。</summary>
    public bool? TargetReached { get; init; }
    public long Timestamp { get; init; }
    public string ModeName => IsFirmware117 ? Mode switch
    { 1 => "PP 轮廓位置", 2 => "PV 轮廓速度", 3 => "CSP 同步位置", 4 => "CSV 同步速度", 0 => "待机(0)", _ => $"模式({Mode})" } : Mode switch
    {
        1 => "同步位置", 2 => "力矩开环（电流）", _ => $"未知({Mode})",
    };
}
