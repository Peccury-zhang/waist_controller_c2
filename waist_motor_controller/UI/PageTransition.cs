using System.Diagnostics;
using System.Drawing.Imaging;

namespace waist_motor_controller.UI;

/// <summary>仅在导航时短暂绘制页面快照，实时状态和控制定时器继续运行。</summary>
internal sealed class PageTransition : System.Windows.Forms.Control
{
    private readonly System.Windows.Forms.Timer _timer = new() { Interval = 15 };
    private readonly Stopwatch _elapsed = new();
    private Bitmap? _before, _after;

    public PageTransition()
    {
        SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true);
        Dock = DockStyle.Fill;
        Visible = false;
        TabStop = false;
        _timer.Tick += (_, _) => { if (_elapsed.ElapsedMilliseconds >= 180) Finish(); else Invalidate(); };
    }

    public Bitmap? TakeSnapshot()
    {
        Finish();
        if (Parent == null || !Parent.Visible || Parent.ClientSize.Width <= 0 || Parent.ClientSize.Height <= 0) return null;
        var bitmap = new Bitmap(Parent.ClientSize.Width, Parent.ClientSize.Height);
        Parent.DrawToBitmap(bitmap, new Rectangle(Point.Empty, bitmap.Size));
        return bitmap;
    }

    public void Start(Bitmap? before)
    {
        if (before == null) return;
        _after = TakeSnapshot();
        if (_after == null) { before.Dispose(); return; }
        _before = before;
        BringToFront(); Visible = true;
        _elapsed.Restart(); _timer.Start();
    }

    public void Finish()
    {
        _timer.Stop(); Visible = false;
        _before?.Dispose(); _before = null;
        _after?.Dispose(); _after = null;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        if (_before == null || _after == null) return;
        e.Graphics.DrawImage(_after, ClientRectangle);
        float progress = Math.Clamp(_elapsed.ElapsedMilliseconds / 180f, 0, 1);
        float opacity = (1 - progress) * (1 - progress);
        using var attributes = new ImageAttributes();
        attributes.SetColorMatrix(new ColorMatrix { Matrix33 = opacity });
        e.Graphics.DrawImage(_before, ClientRectangle, 0, 0, _before.Width, _before.Height, GraphicsUnit.Pixel, attributes);
    }

    protected override void OnSizeChanged(EventArgs e) { base.OnSizeChanged(e); Finish(); }
    protected override void Dispose(bool disposing)
    {
        if (disposing) { Finish(); _timer.Dispose(); }
        base.Dispose(disposing);
    }
}
