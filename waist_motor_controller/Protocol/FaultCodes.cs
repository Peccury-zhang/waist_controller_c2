namespace waist_motor_controller.Protocol;

/// <summary>来源：shared_files/报错逻辑&故障代码V1.3.xlsx，ZYR120&120MAX报错逻辑工作表。</summary>
public static class FaultCodes
{
    public sealed record Entry(ushort Code, string Name, string ClearMethod, bool RequiresRestart);
    public static IReadOnlyList<Entry> Entries { get; } = Array.AsReadOnly(new[]
    {
        new Entry(0x0001, "过压", "重启清除", true),
        new Entry(0x0002, "欠压", "重启清除", true),
        new Entry(0x0004, "过温", "恢复到85°C以下后指令清除", false),
        new Entry(0x0008, "堵转", "重启清除", true),
        new Entry(0x0010, "过载", "重启清除", true),
        new Entry(0x0020, "速度跟踪误差过大", "重启清除", true),
        new Entry(0x0040, "正限位保护", "指令清除，向限位反方向可运行", false),
        new Entry(0x0080, "负限位保护", "指令清除，向限位反方向可运行", false),
        new Entry(0x0100, "输出端编码器异常", "重启清除", true),
        new Entry(0x0200, "位置跃迁过大", "指令清除", false),
        new Entry(0x0400, "电流采样错误", "重启清除", true),
        new Entry(0x0800, "电流跟踪误差超差", "重启清除", true),
        new Entry(0x1000, "位置跟踪误差过大", "重启清除", true),
    });
    public const ushort KnownMask = 0x1FFF;
    public static bool RequiresRestart(ushort code) => Entries.Any(e => e.RequiresRestart && (code & e.Code) != 0);
    public static bool CanClearWithCommand(ushort code) => code != 0 && (code & ~KnownMask) == 0 && !RequiresRestart(code);
    public static string Describe(ushort code)
    {
        if (code == 0) return "无";
        var parts = Entries.Where(e => (code & e.Code) != 0).Select(e => $"{e.Name}（{e.ClearMethod}）").ToList();
        ushort unknown = (ushort)(code & ~KnownMask);
        if (unknown != 0) parts.Add($"未定义故障位0x{unknown:X4}（该工作表未分配代码）");
        return $"0x{code:X4}：" + string.Join("；", parts);
    }
}
