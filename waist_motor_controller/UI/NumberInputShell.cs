namespace waist_motor_controller.UI;

/// <summary>保留原生数值编辑和范围校验，使用填满输入框高度的宽幅调节按钮。</summary>
internal sealed class NumberInputShell : SurfacePanel
{
    private readonly NumericUpDown _number;
    private readonly Panel _editorViewport;
    private readonly Button _up, _down;
    private readonly System.Windows.Forms.Timer _repeat = new();
    private Button? _held;
    private bool _repeated;

    public NumberInputShell(NumericUpDown number)
    {
        _number = number;
        FillColor = BackColor = UiTheme.Input;
        _up = MakeArrow("▲", "Increment");
        _down = MakeArrow("▼", "Decrement");
        // 原生 NumericUpDown 会反复恢复自身箭头与编辑框布局。
        // 用视口裁掉原生箭头，而非修改其子控件，消除黑色占位和重绘残影。
        _editorViewport = new Panel { BackColor = UiTheme.Input, Margin = Padding.Empty };
        _editorViewport.Controls.Add(number);
        Controls.AddRange(new System.Windows.Forms.Control[] { _editorViewport, _up, _down });
        number.Dock = DockStyle.None;
        number.EnabledChanged += (_, _) => UpdateEnabled();
        number.FontChanged += (_, _) => PerformLayout();
        _repeat.Tick += (_, _) =>
        {
            if (_held == null || !_held.Enabled) { _repeat.Stop(); return; }
            Step(_held); _repeated = true; _repeat.Interval = 75;
        };
        UpdateEnabled();
    }

    private Button MakeArrow(string text, string name)
    {
        var button = new ActionButton { Text = text, Name = name, Font = UiTheme.Font(8), Margin = Padding.Empty, TabStop = false };
        button.AccessibleName = name == "Increment" ? "增加数值" : "减少数值";
        button.Click += (_, _) => { if (!_repeated) Step(button); _repeated = false; };
        button.MouseDown += (_, e) =>
        {
            if (e.Button != MouseButtons.Left) return;
            _held = button; _repeated = false; _repeat.Interval = 400; _repeat.Start();
        };
        button.MouseUp += (_, _) => StopRepeat();
        button.MouseLeave += (_, _) => StopRepeat();
        button.MouseCaptureChanged += (_, _) => { if (!button.Capture) StopRepeat(); };
        return button;
    }

    private void Step(Button button)
    {
        if (!_number.Enabled || _number.ReadOnly) return;
        if (button == _up) _number.UpButton(); else _number.DownButton();
    }
    private void StopRepeat() { _repeat.Stop(); _held = null; }
    private void UpdateEnabled()
    {
        _up.Enabled = _down.Enabled = _number.Enabled && !_number.ReadOnly;
        if (!_number.Enabled) StopRepeat();
    }
    protected override void OnLayout(LayoutEventArgs e)
    {
        base.OnLayout(e);
        if (_number == null || _up == null || _down == null) return;
        float scale = DeviceDpi / 96f;
        int inset = (int)Math.Round(2 * scale), left = (int)Math.Round(10 * scale);
        int width = (int)Math.Round(38 * scale), height = Math.Max(0, ClientSize.Height - inset * 2);
        int x = Math.Max(0, ClientSize.Width - width - inset);
        _up.SetBounds(x, inset, width, height / 2);
        _down.SetBounds(x, inset + height / 2, width, height - height / 2);
        _editorViewport.SetBounds(left, Math.Max(0, (ClientSize.Height - _number.PreferredHeight) / 2),
            Math.Max(1, x - left - inset), _number.PreferredHeight);
        int nativeArrowWidth = _number.Controls.Cast<System.Windows.Forms.Control>()
            .Where(child => child is not TextBox).Select(child => child.Width).DefaultIfEmpty((int)(16 * scale)).Max();
        _number.SetBounds(0, 0, _editorViewport.Width + nativeArrowWidth, _number.PreferredHeight);
    }
    protected override void Dispose(bool disposing)
    {
        if (disposing) _repeat.Dispose();
        base.Dispose(disposing);
    }
}
