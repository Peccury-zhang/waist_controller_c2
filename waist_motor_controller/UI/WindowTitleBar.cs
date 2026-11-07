using System.Drawing.Drawing2D;

namespace waist_motor_controller.UI;

internal sealed class WindowTitleBar : Panel
{
    private readonly Form _form;
    private readonly Label _title;
    private readonly PictureBox _logo;
    private readonly FlowLayoutPanel _actions;

    public WindowTitleBar(Form form)
    {
        _form = form;
        Name = "WindowTitleBar"; Dock = DockStyle.Top; Height = 52;
        BackColor = UiTheme.Background; DoubleBuffered = true;
        _title = new CaptionLabel { Text = form.Text, Font = UiTheme.Font(13, FontStyle.Bold), ForeColor = Color.White,
            TextAlign = ContentAlignment.MiddleLeft, AutoEllipsis = true };
        using var logoStream = typeof(WindowTitleBar).Assembly.GetManifestResourceStream("waist_motor_controller.yclab2_orange.png");
        using var logo = logoStream == null ? null : Image.FromStream(logoStream);
        _logo = new CaptionLogo { Image = logo == null ? form.Icon?.ToBitmap() : new Bitmap(logo), SizeMode = PictureBoxSizeMode.Zoom };
        _actions = new FlowLayoutPanel { FlowDirection = FlowDirection.LeftToRight, WrapContents = false,
            Margin = Padding.Empty, Padding = Padding.Empty };
        var minimize = new CaptionButton(0) { AccessibleName = "最小化" };
        var maximize = new CaptionButton(1) { AccessibleName = "最大化或还原" };
        var close = new CaptionButton(2) { AccessibleName = "关闭" };
        minimize.Click += (_, _) => form.WindowState = FormWindowState.Minimized;
        maximize.Click += (_, _) => form.WindowState = form.WindowState == FormWindowState.Maximized ? FormWindowState.Normal : FormWindowState.Maximized;
        close.Click += (_, _) => form.Close();
        _actions.Controls.AddRange(new System.Windows.Forms.Control[] { minimize, maximize, close });
        Controls.AddRange(new System.Windows.Forms.Control[] { _title, _logo, _actions });
        form.TextChanged += UpdateTitle;
    }

    private void UpdateTitle(object? sender, EventArgs e) => _title.Text = _form.Text;

    // Child HWNDs otherwise consume WM_NCHITTEST before MainForm sees it.
    // HTTRANSPARENT lets Windows continue hit testing on the parent, preserving
    // native move, double-click maximize, right-click menu and top-edge resizing.
    // The action buttons remain ordinary client controls.
    protected override void WndProc(ref Message m)
    {
        if (m.Msg == 0x84) { m.Result = -1; return; }
        base.WndProc(ref m);
    }

    private sealed class CaptionLabel : Label
    {
        protected override void WndProc(ref Message m)
        {
            if (m.Msg == 0x84) { m.Result = -1; return; }
            base.WndProc(ref m);
        }
    }

    private sealed class CaptionLogo : PictureBox
    {
        protected override void WndProc(ref Message m)
        {
            if (m.Msg == 0x84) { m.Result = -1; return; }
            base.WndProc(ref m);
        }
    }

    protected override void OnLayout(LayoutEventArgs e)
    {
        base.OnLayout(e);
        if (_logo == null) return;
        float scale = DeviceDpi / 96f;
        int logoSize = (int)(48 * scale), inset = (int)(12 * scale), buttonWidth = (int)(46 * scale);
        _logo.SetBounds(inset, (Height - logoSize) / 2, logoSize, logoSize);
        _actions.SetBounds(Width - buttonWidth * 3, 0, buttonWidth * 3, Height);
        foreach (System.Windows.Forms.Control button in _actions.Controls) button.Size = new Size(buttonWidth, Height);
        int textLeft = _logo.Right + (int)(10 * scale);
        _title.SetBounds(textLeft, 0, Math.Max(0, _actions.Left - textLeft - inset), Height);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing) { _form.TextChanged -= UpdateTitle; _logo.Image?.Dispose(); }
        base.Dispose(disposing);
    }

    private sealed class CaptionButton(int kind) : Button
    {
        private bool _hover;
        protected override void OnCreateControl()
        {
            base.OnCreateControl();
            SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true);
            Margin = Padding.Empty; FlatStyle = FlatStyle.Flat; FlatAppearance.BorderSize = 0; TabStop = false;
        }
        protected override void OnMouseEnter(EventArgs e) { base.OnMouseEnter(e); _hover = true; Invalidate(); }
        protected override void OnMouseLeave(EventArgs e) { base.OnMouseLeave(e); _hover = false; Invalidate(); }
        protected override void OnPaint(PaintEventArgs e)
        {
            e.Graphics.Clear(_hover ? kind == 2 ? Color.FromArgb(205, 57, 79) : UiTheme.Input : UiTheme.Background);
            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
            float scale = DeviceDpi / 96f, half = 5 * scale, x = Width / 2f, y = Height / 2f;
            using var pen = new Pen(Color.White, 1.2f * scale);
            if (kind == 0) e.Graphics.DrawLine(pen, x - half, y, x + half, y);
            else if (kind == 1)
            {
                if (FindForm()?.WindowState == FormWindowState.Maximized)
                {
                    e.Graphics.DrawRectangle(pen, x - half + 3 * scale, y - half, 2 * half - 3 * scale, 2 * half - 3 * scale);
                    e.Graphics.DrawRectangle(pen, x - half, y - half + 3 * scale, 2 * half - 3 * scale, 2 * half - 3 * scale);
                }
                else e.Graphics.DrawRectangle(pen, x - half, y - half, half * 2, half * 2);
            }
            else { e.Graphics.DrawLine(pen, x - half, y - half, x + half, y + half); e.Graphics.DrawLine(pen, x + half, y - half, x - half, y + half); }
        }
    }
}
