using waist_motor_controller.Can;

namespace waist_motor_controller.Protocol;

public static class FeedbackParser
{
    public static MotorFeedback Parse(int deviceId, CanFrame f, bool firmware117 = false)
    {
        // 11 个有效字节，CANFD DLC=12（末字节填充）。拒绝旧版 16 字节布局。
        if (f.Id != Od.FeedbackBase + deviceId || f.Length != 12)
            throw new ArgumentException("新版反馈须为 0x300+ID、12 字节 CANFD 帧");
        if (firmware117)
        {
            byte st = f.Data[11];
            return new MotorFeedback
            {
                DeviceId = deviceId, IsFirmware117 = true,
                PositionRaw = f.ReadBe16(0), VelocityRpm = f.ReadBe16(2), CurrentMa = f.ReadBe16(4),
                FaultCode = f.ReadBeU16(6), TemperatureRaw = f.ReadBe16(8), Mode = f.Data[10],
                Enabled = (st & 0x80) != 0, BrakeReleased = (st & 0x40) != 0, HasError = (st & 0x20) != 0,
                TargetReached = (st & 0x10) != 0,
            };
        }
        byte status = f.Data[6];
        return new MotorFeedback
        {
            DeviceId = deviceId,
            PositionRaw = f.ReadBe16(0), VelocityRpm = f.ReadBe16(2), CurrentMa = f.ReadBe16(4),
            Mode = (byte)((status >> 6) & 3), Enabled = (status & 0x20) != 0,
            BrakeReleased = (status & 0x10) != 0, HasError = (status & 0x08) != 0,
            FaultCode = (ushort)(f.Data[7] | (f.Data[8] << 8)),
            TemperatureRaw = f.ReadBe16(9),
        };
    }
}
