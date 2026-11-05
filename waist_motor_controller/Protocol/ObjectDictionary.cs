namespace waist_motor_controller.Protocol;

/// <summary>《新-巨蟹电机协议》2026-01-13 版。PDO 位置为输出端单圈有符号计数。</summary>
public static class Od
{
    public const uint SyncId = 0x080, EmcyBase = 0x080, PdoBase = 0x100;
    public const uint FeedbackBase = 0x300, SdoReqBase = 0x600, SdoRspBase = 0x580, BootBase = 0x700;
    public const ushort CanNodeId = 0x2530, ZeroSet = 0x2531, FlashCode = 0x2539;
    public const ushort ActualPosition = 0x6064, SoftwareVersion = 0x100A;
    public const double CountsPerDeg = 65536.0 / 360.0;
    // 协议建议距 int16 极值留 200cnt，避免触发正负限位。
    public const short MinPositionRaw = -32568, MaxPositionRaw = 32568;
    public static double MinAngle => RawToAngle(MinPositionRaw);
    public static double MaxAngle => RawToAngle(MaxPositionRaw);
    public const byte PositionMode = 1, CurrentMode = 2;

    public static short AngleToRaw(double deg)
    {
        if (!double.IsFinite(deg) || deg < MinAngle || deg > MaxAngle)
            throw new ArgumentOutOfRangeException(nameof(deg), $"角度须在 {MinAngle:F3}°~{MaxAngle:F3}° 内");
        return checked((short)Math.Round(deg * CountsPerDeg));
    }

    public static double RawToAngle(int raw) => raw / CountsPerDeg;
}
