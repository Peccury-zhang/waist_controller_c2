using System.Text;

namespace waist_motor_controller.Can;

/// <summary>
/// 平台无关的 CAN / CANFD 帧模型（标准帧，ID ≤ 0x7FF）。
/// 提供巨蟹协议所需的大端（PDO/反馈）与小端（SDO）读写助手，字节序注释见各调用处。
/// </summary>
public sealed class CanFrame
{
    public const int MaxDataLen = 64;

    public uint Id { get; }
    public byte Length { get; }
    public byte[] Data { get; }

    /// <summary>true = CANFD 帧；false = 经典 CAN 帧。</summary>
    public bool IsFd { get; }

    /// <summary>CANFD BRS 位（数据段加速），仅 IsFd=true 时有效。</summary>
    public bool Brs { get; }

    public CanFrame(uint id, ReadOnlySpan<byte> data, bool isFd = false, bool brs = true)
    {
        if (id > 0x7FF)
            throw new ArgumentOutOfRangeException(nameof(id), "仅支持 11bit 标准帧 ID");
        if (data.Length > MaxDataLen)
            throw new ArgumentOutOfRangeException(nameof(data));
        Id = id;
        Data = data.ToArray();
        Length = (byte)data.Length;
        IsFd = isFd;
        Brs = isFd && brs;
    }

    /// <summary>本项目驱动实测在有效FD接收ID附带bit29；官方APP按低29位提取ID。
    /// 不将FD队列的bit29直接判成错误帧；经典CAN仍拒绝该错误标志。</summary>
    public static bool TryNormalizeReceivedId(uint rawId, bool isFd, out uint id)
    {
        id = rawId & 0x1FFFFFFFu;
        return (rawId & 0xC0000000u) == 0 && (isFd || (rawId & 0x20000000u) == 0) && id <= 0x7FF;
    }

    // ---------------- 大端（PDO 控制帧 / 0x300 反馈帧，高位在前） ----------------

    public void WriteBe16(int offset, int value)
    {
        short v = (short)value;
        unchecked
        {
            Data[offset] = (byte)(v >> 8);
            Data[offset + 1] = (byte)v;
        }
    }

    public short ReadBe16(int offset)
        => unchecked((short)((Data[offset] << 8) | Data[offset + 1]));

    public ushort ReadBeU16(int offset)
        => unchecked((ushort)((Data[offset] << 8) | Data[offset + 1]));

    // ---------------- 小端（SDO 帧，低位在前） ----------------

    public void WriteLe16(int offset, int value)
    {
        short v = (short)value;
        unchecked
        {
            Data[offset] = (byte)v;
            Data[offset + 1] = (byte)(v >> 8);
        }
    }

    public void WriteLe32(int offset, int value)
    {
        unchecked
        {
            Data[offset] = (byte)value;
            Data[offset + 1] = (byte)(value >> 8);
            Data[offset + 2] = (byte)(value >> 16);
            Data[offset + 3] = (byte)(value >> 24);
        }
    }

    public short ReadLe16(int offset)
        => unchecked((short)(Data[offset] | (Data[offset + 1] << 8)));

    public int ReadLe32(int offset)
        => unchecked(Data[offset] | (Data[offset + 1] << 8) | (Data[offset + 2] << 16) | (Data[offset + 3] << 24));

    public override string ToString()
    {
        var sb = new StringBuilder(96);
        sb.Append(IsFd ? "FD" : "CAN").Append(" 0x").Append(Id.ToString("X3"));
        sb.Append(" [").Append(Length).Append("] ");
        for (int i = 0; i < Length; i++)
            sb.Append(Data[i].ToString("X2")).Append(' ');
        return sb.ToString().TrimEnd();
    }
}
