using System.Text;
using waist_motor_controller.Can;
namespace waist_motor_controller.Protocol;

public static class SdoStringReader
{
    public static async Task<string?> ReadAsync(int id, ushort index, Func<CanFrame, Task<SdoResponse?>> request)
    {
        var initial = await request(SdoMessage.Read(id, index, 0)).ConfigureAwait(false);
        if (initial == null || initial.IsAbort) return null;
        if (initial.IsExpedited) return Encoding.ASCII.GetString(initial.Data).TrimEnd('\0');
        if (initial.Command != 0x41) return null;
        int length = initial.ReadValueLe32;
        if (length <= 0 || length > 256) return null;
        var data = new List<byte>();
        bool toggle = false;
        for (int i = 0; i < 40; i++)
        {
            var part = await request(SdoMessage.UploadSegment(id, toggle)).ConfigureAwait(false);
            if (part == null || !part.IsSegment || (part.Command & 0x10) != (toggle ? 0x10 : 0)) return null;
            data.AddRange(part.Data);
            if (data.Count > length) return null;
            if ((part.Command & 1) != 0)
                return data.Count == length ? Encoding.ASCII.GetString(data.ToArray()).TrimEnd('\0') : null;
            toggle = !toggle;
        }
        return null;
    }
}
