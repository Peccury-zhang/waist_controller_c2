namespace waist_motor_controller.UI;

/// <summary>只应用变化值；尤其避免重复SetToolTip和背景颜色导致整片控件重绘。</summary>
internal sealed class UiValueCache
{
    private readonly Dictionary<System.Windows.Forms.Control, string> _tips = new();
    public static void Text(System.Windows.Forms.Control control, string value)
    { if (control.Text != value) control.Text = value; }
    public static void Color(System.Windows.Forms.Control control, Color value)
    { if (control.ForeColor != value) control.ForeColor = value; }
    public static void Enabled(System.Windows.Forms.Control control, bool value)
    { if (control.Enabled != value) control.Enabled = value; }
    public void Tip(ToolTip toolTip, System.Windows.Forms.Control control, string value)
    {
        if (_tips.TryGetValue(control, out string? old) && old == value) return;
        _tips[control] = value;
        toolTip.SetToolTip(control, value);
    }
}
