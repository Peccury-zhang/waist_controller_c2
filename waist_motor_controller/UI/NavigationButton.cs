using System.Diagnostics;
using System.Drawing.Drawing2D;

namespace waist_motor_controller.UI;

/// <summary>深色标签页，选中时紫色描边与下方分隔线衔接。</summary>
internal sealed class NavigationButton : ActionButton
{
    private readonly System.Windows.Forms.Timer _timer = new() { Interval = 15 };
    private readonly Stopwatch _elapsed = new();
    private float _highlight, _startHighlight, _targetHighlight;
    private bool _selected;

    [System.ComponentModel.DesignerSerializationVisibility(System.ComponentModel.DesignerSerializationVisibility.Hidden)]
    public bool Selected
    {
        get => _selected;
        set { if (_selected == value) return; _selected = value; Invalidate(); }
    }

    public NavigationButton()
    {
        Dock = DockStyle.Fill;
        Height = 44;
        Kind = ActionKind.Navigation;
        Font = UiTheme.Font(11.5F);
        _timer.Tick += (_, _) =>
        {
            float progress = Math.Clamp(_elapsed.ElapsedMilliseconds / 180f, 0, 1);
            float ease = 1 - MathF.Pow(1 - progress, 3);
            _highlight = _startHighlight + (_targetHighlight - _startHighlight) * ease;
            Invalidate();
            if (progress >= 1) _timer.Stop();
        };
    }

    private void AnimateHighlight(float target)
    {
        _timer.Stop();
        _startHighlight = _highlight;
        _targetHighlight = target;
        _elapsed.Restart(); _timer.Start();
    }

    protected override void OnMouseEnter(EventArgs e) { base.OnMouseEnter(e); if (Enabled) AnimateHighlight(1); }
    protected override void OnMouseLeave(EventArgs e) { base.OnMouseLeave(e); AnimateHighlight(0); }
    protected override void OnEnabledChanged(EventArgs e)
    {
        base.OnEnabledChanged(e);
        if (!Enabled) { _timer.Stop(); _highlight = 0; Invalidate(); }
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        var graphics = e.Graphics;
        graphics.Clear(NavigationStrip.BackgroundColor);
        if (Width < 4 || Height < 4) return;
        float scale = DeviceDpi / 96f, radius = 5 * scale, top = 3 * scale;
        float left = scale, right = Width - scale, bottom = Height;
        graphics.SmoothingMode = SmoothingMode.AntiAlias;
        using var outline = new GraphicsPath();
        outline.AddLine(left, bottom, left, top + radius);
        outline.AddArc(left, top, radius * 2, radius * 2, 180, 90);
        outline.AddLine(left + radius, top, right - radius, top);
        outline.AddArc(right - radius * 2, top, radius * 2, radius * 2, 270, 90);
        outline.AddLine(right, top + radius, right, bottom);
        using var fill = new SolidBrush(_selected ? Color.FromArgb(128, 64, 192) : NavigationStrip.BackgroundColor);
        graphics.FillPath(fill, outline);
        if (!_selected && _highlight > 0 && Enabled)
        {
            var state = graphics.Save(); graphics.SetClip(outline);
            using var hover = new SolidBrush(Color.FromArgb(40, 35, 57));
            graphics.FillRectangle(hover, 0, Height * (1 - _highlight), Width, Height * _highlight);
            graphics.Restore(state);
        }
        using var border = new Pen(NavigationStrip.AccentColor, 1.4f * scale);
        if (_selected) graphics.DrawPath(border, outline);
        else graphics.DrawLine(border, 0, Height - scale, Width, Height - scale);
        using var titleFont = new Font(Font, _selected ? FontStyle.Bold : FontStyle.Regular);
        TextRenderer.DrawText(graphics, Text, titleFont, new Rectangle((int)(12 * scale), (int)top,
            Math.Max(0, Width - (int)(24 * scale)), Height - (int)top), Enabled ? Color.White : UiTheme.Muted,
            TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter | TextFormatFlags.SingleLine | TextFormatFlags.EndEllipsis);
        if (Focused && ShowFocusCues)
            ControlPaint.DrawFocusRectangle(graphics, Rectangle.Inflate(ClientRectangle, -(int)(6 * scale), -(int)(7 * scale)));
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing) _timer.Dispose();
        base.Dispose(disposing);
    }
}
