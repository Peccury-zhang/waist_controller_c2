using System.Runtime.InteropServices;

namespace waist_motor_controller.Native;

/// <summary>
/// ZLG ControlCANFD.dll 的 P/Invoke 封装。
/// 结构体布局与官方 C# 样例（c#_example_vs2022_v1.4/ZLGCAN/zlgcan.cs）保持一致，不可改动字段顺序。
/// 仅支持 x64（native/ControlCANFD.dll）。
/// </summary>
internal static class ZlgCanNative
{
    internal const int TYPE_CAN = 0;
    internal const int TYPE_CANFD = 1;

    internal const uint ZCAN_USBCANFD_200U = 41;
    internal const int STATUS_OK = 1;

    /// <summary>CANFD 帧的 BRS 标志（发送 FD 帧时置位以启用 5M 数据段速率）。</summary>
    internal const byte CANFD_BRS = 0x01;

    // ---------------- 结构体 ----------------

    [StructLayout(LayoutKind.Sequential)]
    internal struct ZCAN
    {
        public uint acc_code;
        public uint acc_mask;
        public uint reserved;
        public byte filter;
        public byte timing0;
        public byte timing1;
        public byte mode;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct CANFD
    {
        public uint acc_code;
        public uint acc_mask;
        public uint abit_timing;
        public uint dbit_timing;
        public uint brp;
        public byte filter;
        public byte mode;
        public ushort pad;
        public uint reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct can_frame
    {
        public uint can_id;   /* 32 bit: 低 11/29 位为 ID，高位为 EFF/RTR/ERR 标志 */
        public byte can_dlc;
        public byte __pad;
        public byte __res0;
        public byte __res1;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
        public byte[] data;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct canfd_frame
    {
        public uint can_id;
        public byte len;
        public byte flags;
        public byte __res0;
        public byte __res1;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)]
        public byte[] data;
    }

    [StructLayout(LayoutKind.Explicit)]
    internal struct ZCAN_CHANNEL_INIT_CONFIG
    {
        [FieldOffset(0)]
        public uint can_type;          // TYPE_CAN / TYPE_CANFD

        [FieldOffset(4)]
        public ZCAN can;

        [FieldOffset(4)]
        public CANFD canfd;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ZCAN_Transmit_Data
    {
        public can_frame frame;
        public uint transmit_type;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ZCAN_Receive_Data
    {
        public can_frame frame;
        public ulong timestamp; // us
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ZCAN_TransmitFD_Data
    {
        public canfd_frame frame;
        public uint transmit_type;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ZCAN_ReceiveFD_Data
    {
        public canfd_frame frame;
        public ulong timestamp; // us
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ChannelErrorInfo
    {
        public uint error_code;
        public byte passive0, passive1, passive2, arbitrationLost;
    }
    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_ReadChannelErrInfo(IntPtr channelHandle, out ChannelErrorInfo error);

    // ---------------- IProperty（属性接口，Cdecl 函数指针） ----------------

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate int SetValueFunc(string path, string value);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate string GetValueFunc(string path, string value);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    internal delegate IntPtr GetPropertysFunc(string path, string value);

    [StructLayout(LayoutKind.Sequential)]
    internal struct IProperty
    {
        public SetValueFunc SetValue;
        public GetValueFunc GetValue;
        public GetPropertysFunc GetPropertys;
    }

    // ---------------- 函数导出（StdCall） ----------------

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern IntPtr ZCAN_OpenDevice(uint device_type, uint device_index, uint reserved);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_CloseDevice(IntPtr device_handle);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern IntPtr ZCAN_InitCAN(IntPtr device_handle, uint can_index, IntPtr pInitConfig);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_StartCAN(IntPtr channel_handle);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_ResetCAN(IntPtr channel_handle);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_Transmit(IntPtr channel_handle, IntPtr pTransmit, uint len);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_TransmitFD(IntPtr channel_handle, IntPtr pTransmit, uint len);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_GetReceiveNum(IntPtr channel_handle, byte type);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_Receive(IntPtr channel_handle, IntPtr data, uint len, int wait_time = -1);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ZCAN_ReceiveFD(IntPtr channel_handle, IntPtr data, uint len, int wait_time = -1);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern IntPtr GetIProperty(IntPtr device_handle);

    [DllImport("ControlCANFD.dll", CallingConvention = CallingConvention.StdCall)]
    internal static extern uint ReleaseIProperty(IntPtr pIProperty);
}
