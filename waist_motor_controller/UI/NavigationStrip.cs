namespace waist_motor_controller.UI;

/// <summary>紧凑的左对齐页签，紫色基线延伸到整个导航栏。</summary>
internal sealed class NavigationStrip : Panel
{
    public static readonly Color BackgroundColor = UiTheme.Background;
    public static readonly Color AccentColor = Color.FromArgb(164, 137, 250);

    public NavigationStrip()
    {
        Dock = DockStyle.Fill; Margin = new Padding(0, 0, 0, 10);
        BackColor = BackgroundColor; DoubleBuffered = true;
        SetStyle(ControlStyles.ResizeRedraw, true);
    }
    protected override void OnLayout(LayoutEventArgs e)
    {
        base.OnLayout(e);
        if (Controls.Count == 0) return;
        int width = Math.Min(ClientSize.Width / Controls.Count, (int)Math.Round(200 * DeviceDpi / 96f));
        for (int i = 0; i < Controls.Count; i++) Controls[i].SetBounds(i * width, 0, width, ClientSize.Height);
    }
    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        float scale = DeviceDpi / 96f;
        using var pen = new Pen(AccentColor, 1.4f * scale);
        e.Graphics.DrawLine(pen, 0, Height - scale, Width, Height - scale);
    }
}
