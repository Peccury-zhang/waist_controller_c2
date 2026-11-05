using waist_motor_controller.Can;

namespace waist_motor_controller.Protocol;

/// <summary>B0: enable bit7, brake bit6, mode bits5..4, reset bit3；三组 int16 大端。</summary>
public readonly record struct PdoCommand(bool Enable, bool BrakeRelease, bool ClearError,
    byte Mode, short Position, short Current = 0, short CurrentFeedforward = 0)
{
    public static PdoCommand Disable => new(false, false, false, 0, 0);
    public static PdoCommand Hold(short position) => new(true, true, false, Od.PositionMode, position);
    public byte ControlWord => (byte)((Enable ? 0x80 : 0) | (BrakeRelease ? 0x40 : 0) |
        ((Mode & 3) << 4) | (ClearError ? 0x08 : 0));

    public void WriteTo(byte[] b)
    {
        b[0] = ControlWord;
        b[1] = (byte)(Position >> 8); b[2] = (byte)Position;
        b[3] = (byte)(Current >> 8); b[4] = (byte)Current;
        b[5] = (byte)(CurrentFeedforward >> 8); b[6] = (byte)CurrentFeedforward;
    }
}

public static class PdoBuilder
{
    public static CanFrame SingleAxis(int deviceId, PdoCommand cmd, bool firmware117 = false)
    {
        if (deviceId is < 1 or > 127) throw new ArgumentOutOfRangeException(nameof(deviceId));
        var data = new byte[7];
        cmd.WriteTo(data);
        if (firmware117)
        {
            // V1.17系列延续原工程的控制格式：模式bits4..1，清错bit5。
            data[0] = (byte)((cmd.Enable ? 0x80 : 0) | (cmd.BrakeRelease ? 0x40 : 0) |
                (cmd.ClearError ? 0x20 : 0) | ((cmd.Mode & 0x0F) << 1));
            if (cmd.Mode == Od.PositionMode)
            {
                // 此格式为PP；主机仍限制轨迹速度≤5输出rpm，内部速度上限5rpm。
                data[3] = 0x07; data[4] = 0xD0; // 原工程的2000rpm/s输入端轮廓加减速
                data[5] = 0; data[6] = 5;
            }
        }
        return new CanFrame(Od.PdoBase + (uint)deviceId, data, isFd: true, brs: true);
    }

    // 只发定向单轴帧；不使用 0x200 广播，避免影响总线上的另外两台电机。
    public static CanFrame Sync() => new(Od.SyncId, ReadOnlySpan<byte>.Empty, isFd: true, brs: true);
}
