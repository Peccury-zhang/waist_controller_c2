using System.Drawing.Drawing2D;

namespace waist_motor_controller.UI;

/// <summary>保留原生下拉列表与键盘选择，统一绘制收起状态的边框、文字和箭头。</summary>
internal sealed class JointComboBox : ComboBox
{
    private bool _hovered;
    public JointComboBox()
    {
        SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        DropDownStyle = ComboBoxStyle.DropDownList;
        DrawMode = DrawMode.OwnerDrawFixed;
        Cursor = Cursors.Hand;
        AccessibleName = "选择控制关节";
    }
    protected override void OnPaint(PaintEventArgs e)
    {
        float scale = DeviceDpi / 96f;
        e.Graphics.Clear(UiTheme.ParentBackground(this));
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        using var shape = UiTheme.Round(new RectangleF(1, 1, Width - 3, Height - 3), 9 * scale);
        using var fill = new SolidBrush(UiTheme.Input);
        using var border = new Pen(Focused || DroppedDown ? UiTheme.Blue : _hovered ? UiTheme.Muted : UiTheme.Border, scale);
        e.Graphics.FillPath(fill, shape); e.Graphics.DrawPath(border, shape);
        int arrowWidth = (int)(38 * scale), inset = (int)(14 * scale);
        TextRenderer.DrawText(e.Graphics, Text, Font, new Rectangle(inset, 0, Math.Max(0, Width - inset - arrowWidth), Height),
            Enabled ? UiTheme.Text : UiTheme.Muted, TextFormatFlags.Left | TextFormatFlags.VerticalCenter | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
        float x = Width - arrowWidth / 2f, y = Height / 2f;
        using var pen = new Pen(DroppedDown ? UiTheme.Blue : UiTheme.Muted, 1.8f * scale) { StartCap = LineCap.Round, EndCap = LineCap.Round };
        float direction = DroppedDown ? -1 : 1;
        e.Graphics.DrawLines(pen, new[] { new PointF(x - 4 * scale, y - direction * 2 * scale), new PointF(x, y + direction * 2 * scale), new PointF(x + 4 * scale, y - direction * 2 * scale) });
    }
    protected override void OnMouseEnter(EventArgs e) { base.OnMouseEnter(e); _hovered = true; Invalidate(); }
    protected override void OnMouseLeave(EventArgs e) { base.OnMouseLeave(e); _hovered = false; Invalidate(); }
    protected override void OnGotFocus(EventArgs e) { base.OnGotFocus(e); Invalidate(); }
    protected override void OnLostFocus(EventArgs e) { base.OnLostFocus(e); Invalidate(); }
    protected override void OnDropDown(EventArgs e) { base.OnDropDown(e); Invalidate(); }
    protected override void OnDropDownClosed(EventArgs e) { base.OnDropDownClosed(e); Invalidate(); }
    protected override void OnSelectedIndexChanged(EventArgs e) { base.OnSelectedIndexChanged(e); Invalidate(); }
}
