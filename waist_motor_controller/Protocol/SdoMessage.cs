using waist_motor_controller.Can;
namespace waist_motor_controller.Protocol;

public sealed record SdoResponse(byte Command, ushort Index, byte SubIndex, byte[] Data)
{
    public bool IsWriteAck => Command == 0x60;
    public bool IsAbort => Command == 0x80;
    public bool IsExpedited => Command is 0x43 or 0x47 or 0x4B or 0x4F;
    public bool IsSegment => (Command & 0xE0) == 0;
    public uint AbortCode => IsAbort ? unchecked((uint)ReadValueLe32) : 0;
    public int ReadValueLe32 => Data.Length >= 4 ? Data[0] | Data[1] << 8 | Data[2] << 16 | Data[3] << 24 : 0;
}
public static class SdoMessage
{
    private static CanFrame Build(int id, byte cmd, ushort index, byte sub, int value = 0)
    {
        if (id is < 1 or > 127) throw new ArgumentOutOfRangeException(nameof(id));
        var data = new byte[8];
        data[0] = cmd; data[1] = (byte)index; data[2] = (byte)(index >> 8); data[3] = sub;
        var f = new CanFrame(Od.SdoReqBase + (uint)id, data, isFd: true, brs: true);
        f.WriteLe32(4, value);
        return f;
    }
    public static CanFrame Read(int id, ushort index, byte sub) => Build(id, 0x40, index, sub);
    public static CanFrame WriteU8(int id, ushort index, byte sub, byte value) => Build(id, 0x2F, index, sub, value);
    public static CanFrame WriteU16(int id, ushort index, byte sub, ushort value) => Build(id, 0x2B, index, sub, value);
    public static CanFrame WriteU32(int id, ushort index, byte sub, int value) => Build(id, 0x23, index, sub, value);
    public static CanFrame UploadSegment(int id, bool toggle) => Build(id, toggle ? (byte)0x70 : (byte)0x60, 0, 0);
    public static SdoResponse Parse(CanFrame frame)
    {
        if (frame.Length != 8) throw new ArgumentException("SDO响应长度须为8");
        byte cmd = frame.Data[0];
        ushort index = (ushort)(frame.Data[1] | frame.Data[2] << 8);
        byte sub = frame.Data[3];
        byte[] data;
        if ((cmd & 0xE0) == 0)
        {
            int unused = (cmd >> 1) & 7;
            data = frame.Data.AsSpan(1, (cmd & 1) != 0 ? 7 - unused : 7).ToArray();
            index = 0; sub = 0;
        }
        else
        {
            int count = cmd switch { 0x4F => 1, 0x4B => 2, 0x47 => 3, _ => 4 };
            data = frame.Data.AsSpan(4, count).ToArray();
        }
        return new SdoResponse(cmd, index, sub, data);
    }
    public static bool Matches(CanFrame request, SdoResponse rsp)
    {
        byte cmd = request.Data[0];
        ushort index = (ushort)(request.Data[1] | request.Data[2] << 8);
        byte sub = request.Data[3];
        if (cmd is 0x60 or 0x70)
            return rsp.IsAbort || (rsp.IsSegment && (rsp.Command & 0x10) == (cmd & 0x10));
        if (rsp.Index != index || rsp.SubIndex != sub) return false;
        if (rsp.IsAbort) return true;
        return cmd == 0x40 ? (rsp.IsExpedited || rsp.Command == 0x41) : rsp.IsWriteAck;
    }
}
