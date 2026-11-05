using System.Runtime.InteropServices;

namespace waist_motor_controller.UI;

internal static class WindowFrameTheme
{
    public static (int Dark, int Border, int Caption, int Text) Apply(nint window)
    {
        // The main window owns its whole caption. DWM must not draw native chrome over it.
        int policy = 1; // DWMNCRP_DISABLED
        DwmSetWindowAttribute(window, 2, ref policy, sizeof(int));
        if (!OperatingSystem.IsWindowsVersionAtLeast(10, 0, 17763)) return (1, 1, 1, 1);
        int dark = 1;
        // Older Windows 10 builds use the previous attribute number.
        int modeResult = DwmSetWindowAttribute(window, 20, ref dark, sizeof(int));
        if (modeResult < 0) modeResult = DwmSetWindowAttribute(window, 19, ref dark, sizeof(int));
        if (!OperatingSystem.IsWindowsVersionAtLeast(10, 0, 22000)) return (modeResult, 1, 1, 1);
        int background = ColorRef(UiTheme.Background), text = ColorRef(UiTheme.Text);
        int border = DwmSetWindowAttribute(window, 34, ref background, sizeof(int));
        int caption = DwmSetWindowAttribute(window, 35, ref background, sizeof(int));
        int title = DwmSetWindowAttribute(window, 36, ref text, sizeof(int));
        return (modeResult, border, caption, title);
    }

    private static int ColorRef(Color color) => color.R | color.G << 8 | color.B << 16;

    [DllImport("dwmapi.dll", ExactSpelling = true)]
    private static extern int DwmSetWindowAttribute(nint hwnd, int attribute, ref int value, int size);
}
