using System.Drawing.Drawing2D;

namespace waist_motor_controller.UI;

internal static class UiTheme
{
    public static readonly Color Background = Color.FromArgb(12, 19, 31);
    public static readonly Color Surface = Color.FromArgb(21, 31, 47);
    public static readonly Color Input = Color.FromArgb(28, 41, 60);
    public static readonly Color Border = Color.FromArgb(49, 67, 91);
    public static readonly Color Text = Color.FromArgb(250, 252, 255);
    public static readonly Color Muted = Color.FromArgb(210, 221, 236);
    public static readonly Color ValueBlue = Color.FromArgb(128, 188, 255);
    public static readonly Color Blue = Color.FromArgb(64, 134, 247);
    public static readonly Color Green = Color.FromArgb(68, 218, 170);
    public static readonly Color Amber = Color.FromArgb(255, 195, 93);
    public static readonly Color Red = Color.FromArgb(238, 87, 105);
    public static Font Font(float size = 11, FontStyle style = FontStyle.Regular) => new("Microsoft YaHei UI", size, style);
    public static Color ParentBackground(System.Windows.Forms.Control control)
    {
        for (var p = control.Parent; p != null; p = p.Parent)
        {
            if (p is SurfacePanel surface) return surface.FillColor;
            if (p.BackColor.A == 255) return p.BackColor;
        }
        return Background;
    }
    public static GraphicsPath Round(RectangleF rect, float radius)
    {
        float d = Math.Min(radius * 2, Math.Min(rect.Width, rect.Height));
        var path = new GraphicsPath();
        if (d <= 0) return path;
        path.AddArc(rect.X, rect.Y, d, d, 180, 90);
        path.AddArc(rect.Right - d, rect.Y, d, d, 270, 90);
        path.AddArc(rect.Right - d, rect.Bottom - d, d, d, 0, 90);
        path.AddArc(rect.X, rect.Bottom - d, d, d, 90, 90);
        path.CloseFigure();
        return path;
    }
}

internal enum ActionKind { Secondary, Primary, Success, Warning, Danger, Navigation, SoftDanger, SoftOrange }
internal enum SegmentPosition { Standalone, First, Middle, Last }

/// <summary>自行绘制全部状态，避免系统主题把禁用文字绘成深色，支持鼠标与键盘反馈。</summary>
internal class ActionButton : Button
{
    private bool _hovered, _pressed;
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public ActionKind Kind { get; set; }
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public bool WhiteText { get; set; }
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public SegmentPosition Segment { get; set; }

    private GraphicsPath Outline(float scale)
    {
        if (Segment == SegmentPosition.Standalone)
            return UiTheme.Round(new RectangleF(1, 1, Width - 3, Height - 3), 9 * scale);
        var path = new GraphicsPath();
        float left = 0.5f, top = 0.5f, right = Width - 0.5f, bottom = Height - 0.5f;
        float d = Math.Min(18 * scale, Math.Min(Width - 1, Height - 1));
        if (Segment == SegmentPosition.First)
        {
            path.AddArc(left, top, d, d, 180, 90);
            path.AddLine(left + d / 2, top, right, top);
            path.AddLine(right, top, right, bottom);
            path.AddLine(right, bottom, left + d / 2, bottom);
            path.AddArc(left, bottom - d, d, d, 90, 90);
        }
        else if (Segment == SegmentPosition.Last)
        {
            path.AddLine(left, top, right - d / 2, top);
            path.AddArc(right - d, top, d, d, 270, 90);
            path.AddArc(right - d, bottom - d, d, d, 0, 90);
            path.AddLine(right - d / 2, bottom, left, bottom);
        }
        else path.AddRectangle(new RectangleF(left, top, right - left, bottom - top));
        path.CloseFigure();
        return path;
    }
    public ActionButton()
    {
        SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        FlatStyle = FlatStyle.Flat;
        FlatAppearance.BorderSize = 0;
        UseVisualStyleBackColor = false;
        Font = UiTheme.Font(12, FontStyle.Bold);
        Cursor = Cursors.Hand;
        Height = 54;
        Margin = new Padding(0, 0, 12, 0);
        BackColor = UiTheme.Surface;
        ForeColor = UiTheme.Text;
    }
    protected override void OnMouseEnter(EventArgs e) { base.OnMouseEnter(e); _hovered = true; Invalidate(); }
    protected override void OnMouseLeave(EventArgs e) { base.OnMouseLeave(e); _hovered = false; _pressed = false; Invalidate(); }
    protected override void OnMouseDown(MouseEventArgs e) { base.OnMouseDown(e); if (e.Button == MouseButtons.Left) _pressed = true; Invalidate(); }
    protected override void OnMouseUp(MouseEventArgs e) { base.OnMouseUp(e); _pressed = false; Invalidate(); }
    protected override void OnMouseCaptureChanged(EventArgs e) { base.OnMouseCaptureChanged(e); if (!Capture) _pressed = false; Invalidate(); }
    protected override void OnKeyDown(KeyEventArgs e) { base.OnKeyDown(e); if (e.KeyCode == Keys.Space) { _pressed = true; Invalidate(); } }
    protected override void OnKeyUp(KeyEventArgs e) { base.OnKeyUp(e); _pressed = false; Invalidate(); }
    protected override void OnEnabledChanged(EventArgs e) { base.OnEnabledChanged(e); _hovered = _pressed = false; Cursor = Enabled ? Cursors.Hand : Cursors.Default; Invalidate(); }
    protected override void OnGotFocus(EventArgs e) { base.OnGotFocus(e); Invalidate(); }
    protected override void OnLostFocus(EventArgs e) { base.OnLostFocus(e); _pressed = false; Invalidate(); }
    protected override void OnPaint(PaintEventArgs e)
    {
        e.Graphics.Clear(UiTheme.ParentBackground(this));
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        Color fill, text, border;
        if (!Enabled)
        {
            fill = Color.FromArgb(28, 38, 53); text = Color.FromArgb(178, 191, 209); border = Color.FromArgb(42, 56, 76);
        }
        else
        {
            (fill, text, border) = Kind switch
            {
                ActionKind.Primary => (UiTheme.Blue, Color.White, UiTheme.Blue),
                ActionKind.Navigation => (Color.FromArgb(230, 126, 34), Color.White, Color.FromArgb(250, 166, 75)),
                ActionKind.Danger => (Color.FromArgb(205, 57, 79), Color.White, UiTheme.Red),
                ActionKind.Success => (Color.FromArgb(23, 69, 63), UiTheme.Green, Color.FromArgb(47, 130, 111)),
                ActionKind.Warning => (Color.FromArgb(70, 55, 35), UiTheme.Amber, Color.FromArgb(137, 103, 55)),
                ActionKind.SoftDanger => (Color.FromArgb(66, 37, 47), Color.FromArgb(255, 178, 189), Color.FromArgb(133, 77, 91)),
                ActionKind.SoftOrange => (Color.FromArgb(75, 53, 36), Color.FromArgb(255, 202, 148), Color.FromArgb(145, 106, 69)),
                _ => (Color.FromArgb(33, 48, 70), UiTheme.Text, UiTheme.Border),
            };
            if (_hovered || (Focused && ShowFocusCues))
            {
                fill = Kind switch
                {
                    ActionKind.Primary => Color.FromArgb(92, 157, 255),
                    ActionKind.Navigation => Color.FromArgb(230, 126, 34),
                    ActionKind.Danger => Color.FromArgb(240, 79, 101),
                    ActionKind.Success => Color.FromArgb(31, 104, 87),
                    ActionKind.Warning => Color.FromArgb(103, 78, 42),
                    ActionKind.SoftDanger => Color.FromArgb(101, 49, 64),
                    ActionKind.SoftOrange => Color.FromArgb(111, 76, 45),
                    _ => Color.FromArgb(53, 81, 116),
                };
                border = Kind == ActionKind.Navigation ? UiTheme.Amber
                    : Kind == ActionKind.Danger ? Color.FromArgb(255, 160, 171) : UiTheme.Blue;
            }
            if (_pressed)
            {
                fill = Kind == ActionKind.Navigation ? Color.FromArgb(195, 97, 20)
                    : Kind == ActionKind.Primary ? Color.FromArgb(36, 96, 194) : Color.FromArgb(25, 42, 63);
                border = UiTheme.Text;
            }
        }
        if (WhiteText) text = Color.White;
        float scale = DeviceDpi / 96f;
        using var outline = Outline(scale);
        using var brush = new SolidBrush(fill);
        using var pen = new Pen(border, (_hovered && Enabled ? 1.7f : 1f) * scale);
        e.Graphics.FillPath(brush, outline);
        if (Enabled && !_pressed) PaintFillOverlay(e.Graphics, outline);
        e.Graphics.DrawPath(pen, outline);
        var content = new Rectangle(8, _pressed ? 1 : 0, Width - 16, Height);
        TextRenderer.DrawText(e.Graphics, Text, Font, content, text,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.EndEllipsis | TextFormatFlags.SingleLine);
        if (Enabled && Focused && ShowFocusCues)
            ControlPaint.DrawFocusRectangle(e.Graphics, Rectangle.Inflate(ClientRectangle, -7, -7), text, fill);
    }

    protected virtual void PaintFillOverlay(Graphics graphics, GraphicsPath outline) { }
}

internal class SurfacePanel : Panel
{
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public Color FillColor { get; set; } = UiTheme.Surface;
    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public Color StrokeColor { get; set; } = UiTheme.Border;
    public SurfacePanel()
    {
        DoubleBuffered = true;
        SetStyle(ControlStyles.ResizeRedraw, true);
        BackColor = UiTheme.Surface;
    }
    protected override void OnPaintBackground(PaintEventArgs e)
    {
        e.Graphics.Clear(UiTheme.ParentBackground(this));
        if (Width < 3 || Height < 3) return;
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        using var shape = UiTheme.Round(new RectangleF(0.5f, 0.5f, Width - 2, Height - 2), 12 * DeviceDpi / 96f);
        using var fill = new SolidBrush(FillColor);
        using var pen = new Pen(StrokeColor);
        e.Graphics.FillPath(fill, shape);
        e.Graphics.DrawPath(pen, shape);
    }
}

internal sealed class BufferedPanel : Panel
{
    public BufferedPanel() { DoubleBuffered = true; SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true); }
}
internal sealed class BufferedTableLayoutPanel : TableLayoutPanel
{
    public BufferedTableLayoutPanel() { DoubleBuffered = true; SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true); }
}
internal sealed class BufferedFlowLayoutPanel : FlowLayoutPanel
{
    public BufferedFlowLayoutPanel() { DoubleBuffered = true; SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true); }
}
