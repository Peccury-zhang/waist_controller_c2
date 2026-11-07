using System.Drawing.Drawing2D;
using System.Runtime.InteropServices;

namespace waist_motor_controller.UI;

/// <summary>保留原生列表与键盘选择，自绘输入框和下拉选项。</summary>
internal sealed class JointComboBox : ComboBox
{
    private bool _hovered;
    private int _committedIndex = -1;
    private static readonly Color PopupBackground = Color.FromArgb(23, 32, 48);
    private static readonly Color PopupSelection = Color.FromArgb(52, 44, 80);
    private const int SetItemHeight = 0x0153;
    private const int GetItemHeightMessage = 0x0154;
    public JointComboBox()
    {
        SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
        DropDownStyle = ComboBoxStyle.DropDownList;
        DrawMode = DrawMode.OwnerDrawFixed;
        MinimumSize = new Size(0, 44);
        Cursor = Cursors.Hand;
        AccessibleName = "选择控制关节";
    }
    protected override void OnHandleCreated(EventArgs e)
    {
        base.OnHandleCreated(e);
        UpdateListHeight();
        UpdateSelectionHeight();
    }
    protected override void OnDpiChangedAfterParent(EventArgs e)
    {
        base.OnDpiChangedAfterParent(e);
        UpdateListHeight();
        UpdateSelectionHeight();
    }
    protected override void OnFontChanged(EventArgs e)
    {
        base.OnFontChanged(e);
        if (IsHandleCreated) UpdateSelectionHeight();
    }
    protected override void OnSizeChanged(EventArgs e)
    {
        base.OnSizeChanged(e);
        if (IsHandleCreated) UpdateSelectionHeight();
    }
    private void UpdateSelectionHeight()
    {
        int height = (int)Math.Round(42 * DeviceDpi / 96d);
        if (SendMessage(Handle, GetItemHeightMessage, -1, 0) != height)
            SendMessage(Handle, SetItemHeight, -1, height);
    }

    private void UpdateListHeight() => ItemHeight = (int)Math.Round(39 * DeviceDpi / 96d);

    [DllImport("user32.dll", EntryPoint = "SendMessageW", ExactSpelling = true)]
    private static extern nint SendMessage(nint window, int message, nint wParam, nint lParam);

    protected override void OnDrawItem(DrawItemEventArgs e)
    {
        if (e.Index < 0) return;
        float scale = DeviceDpi / 96f;
        bool committed = e.Index == _committedIndex;
        bool highlighted = (e.State & DrawItemState.Selected) != 0;
        var graphics = e.Graphics;
        var state = graphics.Save();
        graphics.SetClip(e.Bounds, CombineMode.Intersect);
        graphics.SmoothingMode = SmoothingMode.AntiAlias;
        using var background = new SolidBrush(PopupBackground);
        graphics.FillRectangle(background, e.Bounds);
        if (highlighted || committed)
        {
            var inset = RectangleF.Inflate(e.Bounds, -5 * scale, -3 * scale);
            using var shape = UiTheme.Round(inset, 6 * scale);
            using var fill = new SolidBrush(highlighted ? PopupSelection : Color.FromArgb(36, 42, 64));
            graphics.FillPath(fill, shape);
        }

        int left = (int)Math.Round(15 * scale), right = (int)Math.Round(30 * scale);
        var textBounds = new Rectangle(e.Bounds.Left + left, e.Bounds.Top, Math.Max(0, e.Bounds.Width - left - right), e.Bounds.Height);
        TextRenderer.DrawText(graphics, Items[e.Index]?.ToString() ?? "", Font, textBounds, UiTheme.Text,
            TextFormatFlags.VerticalCenter | TextFormatFlags.Left | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
        if (committed)
        {
            float x = e.Bounds.Right - 16 * scale, y = e.Bounds.Top + e.Bounds.Height / 2f;
            using var check = new Pen(NavigationStrip.AccentColor, 1.5f * scale) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            graphics.DrawLines(check, new[] { new PointF(x - 4 * scale, y), new PointF(x - scale, y + 3 * scale), new PointF(x + 5 * scale, y - 4 * scale) });
        }

        var frame = new RectangleF(e.Bounds.Left + 0.5f, e.Bounds.Top - e.Index * e.Bounds.Height + 0.5f,
            e.Bounds.Width - 1, e.Bounds.Height * Items.Count - 1);
        using var outline = UiTheme.Round(frame, 8 * scale);
        using var border = new Pen(UiTheme.Border, scale);
        graphics.DrawPath(border, outline);
        graphics.Restore(state);
    }
    protected override void OnPaint(PaintEventArgs e)
    {
        float scale = DeviceDpi / 96f;
        e.Graphics.Clear(UiTheme.ParentBackground(this));
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        using var shape = UiTheme.Round(new RectangleF(1, 1, Width - 3, Height - 3), 9 * scale);
        using var fill = new SolidBrush(UiTheme.Input);
        Color accent = NavigationStrip.AccentColor;
        using var border = new Pen(DroppedDown ? accent : Focused || _hovered ? Color.FromArgb(86, 91, 126) : UiTheme.Border, scale);
        e.Graphics.FillPath(fill, shape); e.Graphics.DrawPath(border, shape);
        int arrowWidth = (int)(38 * scale), inset = (int)(14 * scale);
        TextRenderer.DrawText(e.Graphics, Text, Font, new Rectangle(inset, 0, Math.Max(0, Width - inset - arrowWidth), Height),
            Enabled ? UiTheme.Text : UiTheme.Muted, TextFormatFlags.Left | TextFormatFlags.VerticalCenter | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
        float x = Width - arrowWidth / 2f, y = Height / 2f;
        using var pen = new Pen(DroppedDown ? accent : UiTheme.Muted, 1.6f * scale) { StartCap = LineCap.Round, EndCap = LineCap.Round };
        float direction = DroppedDown ? -1 : 1;
        e.Graphics.DrawLines(pen, new[] { new PointF(x - 4 * scale, y - direction * 2 * scale), new PointF(x, y + direction * 2 * scale), new PointF(x + 4 * scale, y - direction * 2 * scale) });
    }
    protected override void OnMouseEnter(EventArgs e) { base.OnMouseEnter(e); _hovered = true; Invalidate(); }
    protected override void OnMouseLeave(EventArgs e) { base.OnMouseLeave(e); _hovered = false; Invalidate(); }
    protected override void OnGotFocus(EventArgs e) { base.OnGotFocus(e); Invalidate(); }
    protected override void OnLostFocus(EventArgs e) { base.OnLostFocus(e); Invalidate(); }
    protected override void OnDropDown(EventArgs e)
    {
        _committedIndex = SelectedIndex;
        int textWidth = Items.Cast<object>().Select(item => TextRenderer.MeasureText(item.ToString(), Font).Width).DefaultIfEmpty(0).Max();
        DropDownWidth = Math.Max(Width, textWidth + (int)Math.Ceiling(48 * DeviceDpi / 96f));
        base.OnDropDown(e);
        Invalidate();
        BeginInvoke(() => { if (!IsDisposed && IsHandleCreated && DroppedDown) ShapePopup(); });
    }
    protected override void OnDropDownClosed(EventArgs e) { base.OnDropDownClosed(e); Invalidate(); }
    protected override void OnSelectedIndexChanged(EventArgs e) { base.OnSelectedIndexChanged(e); Invalidate(); }

    private void ShapePopup()
    {
        nint list = PopupHandle;
        var origin = Point.Empty;
        if (list == 0 || !GetWindowRect(list, out var window) || !GetClientRect(list, out var client) || !ClientToScreen(list, ref origin)) return;
        int left = origin.X - window.Left, top = origin.Y - window.Top;
        int diameter = (int)Math.Round(16 * DeviceDpi / 96f);
        // Clip out the native square border; each row paints its portion of the rounded frame.
        nint region = CreateRoundRectRgn(left, top, left + client.Right, top + client.Bottom, diameter, diameter);
        if (region != 0 && SetWindowRgn(list, region, true) == 0) DeleteObject(region);
    }

    internal nint PopupHandle
    {
        get
        {
            var info = new ComboBoxInfo { Size = Marshal.SizeOf<ComboBoxInfo>() };
            return GetComboBoxInfo(Handle, ref info) ? info.List : 0;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct NativeRect { public int Left, Top, Right, Bottom; }
    [StructLayout(LayoutKind.Sequential)]
    private struct ComboBoxInfo
    {
        public int Size;
        public NativeRect Item, Button;
        public int ButtonState;
        public nint Combo, Edit, List;
    }

    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetComboBoxInfo(nint window, ref ComboBoxInfo info);
    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetWindowRect(nint window, out NativeRect rect);
    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool GetClientRect(nint window, out NativeRect rect);
    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern bool ClientToScreen(nint window, ref Point point);
    [DllImport("gdi32.dll", ExactSpelling = true)]
    private static extern nint CreateRoundRectRgn(int left, int top, int right, int bottom, int width, int height);
    [DllImport("user32.dll", ExactSpelling = true)]
    private static extern int SetWindowRgn(nint window, nint region, bool redraw);
    [DllImport("gdi32.dll", ExactSpelling = true)]
    private static extern bool DeleteObject(nint handle);
}
