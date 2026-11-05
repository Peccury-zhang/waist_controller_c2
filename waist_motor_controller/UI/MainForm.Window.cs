using System.Runtime.InteropServices;

namespace waist_motor_controller.UI;

public sealed partial class MainForm
{
    private const int NativeCaptionStyle = 0x00C00000;

    // Native sizing styles remain enabled, but WM_NCCALCSIZE removes their insets.
    // WinForms must use the same geometry when assigning ClientSize.
    protected override Size SizeFromClientSize(Size clientSize) => clientSize;

    protected override void SetClientSizeCore(int x, int y)
        => SetBoundsCore(Left, Top, x, y, BoundsSpecified.Size);
    protected override CreateParams CreateParams
    {
        get
        {
            var parameters = base.CreateParams;
            // Keep native resize, taskbar and system-menu behavior with a custom caption.
            parameters.Style |= 0x40000 | 0x80000 | 0x10000 | 0x20000;
            parameters.Style &= ~NativeCaptionStyle;
            return parameters;
        }
    }

    protected override void WndProc(ref Message m)
    {
        // Both WM_NCCALCSIZE forms must leave the entire window as client area.
        if (m.Msg == 0x83) { m.Result = 0; return; }
        // Re-activation and classic theme fallback must never paint a second caption.
        if (m.Msg == 0x85) { m.Result = 0; return; } // WM_NCPAINT
        if (m.Msg == 0x86) // WM_NCACTIVATE: update activation without native frame painting.
        {
            m.LParam = new nint(-1);
            base.WndProc(ref m);
            return;
        }
        if (m.Msg == 0x7C && m.WParam == -16) // WM_STYLECHANGING / GWL_STYLE
        {
            var style = Marshal.PtrToStructure<WindowStyle>(m.LParam);
            style.NewStyle &= ~NativeCaptionStyle;
            Marshal.StructureToPtr(style, m.LParam, false);
        }
        base.WndProc(ref m);
        if (m.Msg == 0x24) // WM_GETMINMAXINFO: respect the taskbar on each monitor.
        {
            var screen = Screen.FromHandle(Handle);
            var info = Marshal.PtrToStructure<WindowMinMaxInfo>(m.LParam);
            info.MaxPosition = new Point(screen.WorkingArea.Left - screen.Bounds.Left, screen.WorkingArea.Top - screen.Bounds.Top);
            info.MaxSize = new Point(screen.WorkingArea.Width, screen.WorkingArea.Height);
            Marshal.StructureToPtr(info, m.LParam, false);
        }
        if (m.Msg != 0x84) return; // WM_NCHITTEST
        long coordinates = m.LParam.ToInt64();
        var point = PointToClient(new Point(unchecked((short)coordinates), unchecked((short)(coordinates >> 16))));
        int edge = (int)Math.Ceiling(7 * DeviceDpi / 96f);
        if (WindowState == FormWindowState.Normal)
        {
            bool left = point.X < edge, right = point.X >= ClientSize.Width - edge;
            bool top = point.Y < edge, bottom = point.Y >= ClientSize.Height - edge;
            int hit = top && left ? 13 : top && right ? 14 : bottom && left ? 16 : bottom && right ? 17
                : left ? 10 : right ? 11 : top ? 12 : bottom ? 15 : 0;
            if (hit != 0) { m.Result = hit; return; }
        }
        // Native caption hit testing provides dragging, double-click maximize and the system menu.
        if (point.Y >= 0 && point.Y < 52 * DeviceDpi / 96f && point.X < ClientSize.Width - 138 * DeviceDpi / 96f)
            m.Result = 2;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WindowMinMaxInfo
    {
        public Point Reserved, MaxSize, MaxPosition, MinTrackSize, MaxTrackSize;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WindowStyle { public int OldStyle, NewStyle; }
}
