using System.Runtime.InteropServices;
using waist_motor_controller.Native;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.Can;

/// <summary>
/// 传输层：封装 ZLG 设备打开/关闭、发送（加锁）、接收线程（双队列轮询）、
/// 反馈缓存与 SDO 响应路由。不包含任何业务时序（见 Control 层）。
/// </summary>
public sealed class CanFdTransport : IDisposable, ICanTransport
{
    private const int MaxDeviceId = 127;
    private const int RxBatch = 200;

    private readonly object _sendGate = new();
    private readonly object _cacheGate = new();
    private readonly object _sdoGate = new();

    private IntPtr _deviceHandle = IntPtr.Zero;
    private IntPtr _channelHandle = IntPtr.Zero;
    private IntPtr _propertyHandle = IntPtr.Zero;

    // 防 GC：IProperty 内含函数指针委托，结构体实例必须作为字段存活到 Close 为止
    private ZlgCanNative.IProperty _property;

    private Thread? _rxThread;
    private volatile bool _running;

    // 反馈缓存（下标 = 设备 ID）
    private readonly MotorFeedback?[] _feedback = new MotorFeedback?[MaxDeviceId + 1];
    private readonly long[] _lastFeedbackTick = new long[MaxDeviceId + 1];

    // SDO 等待：每设备一个 TCS + 串行信号量（同 ID 并发请求会串扰）
    private readonly TaskCompletionSource<SdoResponse>?[] _sdoPending = new TaskCompletionSource<SdoResponse>?[MaxDeviceId + 1];
    private readonly CanFrame?[] _sdoRequests = new CanFrame?[MaxDeviceId + 1];
    private readonly SemaphoreSlim[] _sdoLocks = new SemaphoreSlim[MaxDeviceId + 1];

    // 统计计数（Interlocked）
    private long _rxCount;
    private long _txCount;
    private long _feedbackCount;
    private long _badFeedbackCount;
    private long _rawCan, _rawFd;

    /// <summary>日志事件（可能在接收线程触发，订阅方自行跨线程封送）。</summary>
    public event Action<string>? Log;

    /// <summary>未分类帧事件（仅日志用途，高频帧不会到达这里）。</summary>
    public event Action<CanFrame>? FrameReceived;
    public event Action<CanFrame>? RawFrameReceived;

    /// <summary>反馈更新事件（接收线程触发；反馈同时写入缓存供 UI 定时拉取）。</summary>
    public event Action<int, MotorFeedback>? FeedbackUpdated;

    private volatile bool _closing;
    public bool IsOpen => !_closing && _channelHandle != IntPtr.Zero;

    public long RxFrameCount => Interlocked.Read(ref _rxCount);
    public long TxFrameCount => Interlocked.Read(ref _txCount);
    public long FeedbackCount => Interlocked.Read(ref _feedbackCount);
    public long BadFeedbackCount => Interlocked.Read(ref _badFeedbackCount);

    private readonly HashSet<int> _firmware117Ids = new();
    public void ConfigureFirmware117(IEnumerable<int> ids)
    { lock (_cacheGate) { _firmware117Ids.Clear(); _firmware117Ids.UnionWith(ids); } }

    public CanFdTransport()
    {
        for (int i = 0; i <= MaxDeviceId; i++)
            _sdoLocks[i] = new SemaphoreSlim(1, 1);
    }

    /// <summary>打开设备：Open → GetIProperty(波特率) → InitCAN → 终端电阻 → StartCAN → 接收线程。</summary>
    public bool Open(int deviceIndex, int channel, out string error)
    {
        error = string.Empty;
        if (IsOpen)
        {
            error = "设备已处于打开状态";
            return false;
        }

        try
        {
            _closing = false;
            Interlocked.Exchange(ref _rxCount, 0); Interlocked.Exchange(ref _txCount, 0);
            Interlocked.Exchange(ref _feedbackCount, 0); Interlocked.Exchange(ref _badFeedbackCount, 0);
            Interlocked.Exchange(ref _rawCan, 0); Interlocked.Exchange(ref _rawFd, 0);
            lock (_cacheGate) { Array.Clear(_feedback); Array.Clear(_lastFeedbackTick); }
            _deviceHandle = ZlgCanNative.ZCAN_OpenDevice(ZlgCanNative.ZCAN_USBCANFD_200U, (uint)deviceIndex, 0);
            if (_deviceHandle == IntPtr.Zero)
            {
                error = $"打开设备失败：USBCANFD-200U(type=41) 索引 {deviceIndex}（请检查 USB 连接与设备索引）";
                return false;
            }

            _propertyHandle = ZlgCanNative.GetIProperty(_deviceHandle);
            if (_propertyHandle == IntPtr.Zero)
            {
                error = "GetIProperty 失败";
                RollbackOpen();
                return false;
            }
            _property = Marshal.PtrToStructure<ZlgCanNative.IProperty>(_propertyHandle);

            // 波特率属性（路径前缀 = 通道号）
            string ch = channel.ToString();
            if (_property.SetValue(ch + "/canfd_standard", "0") == 0 ||
                _property.SetValue(ch + "/canfd_abit_baud_rate", "1000000") == 0 ||
                _property.SetValue(ch + "/canfd_dbit_baud_rate", "5000000") == 0)
            {
                error = "设置 CANFD 波特率属性失败（仲裁 1M / 数据 5M）";
                RollbackOpen();
                return false;
            }

            // InitCAN（TYPE_CANFD，过滤全收）
            var cfg = new ZlgCanNative.ZCAN_CHANNEL_INIT_CONFIG
            {
                can_type = (uint)ZlgCanNative.TYPE_CANFD,
                canfd = new ZlgCanNative.CANFD(),
            };
            IntPtr pCfg = Marshal.AllocHGlobal(Marshal.SizeOf<ZlgCanNative.ZCAN_CHANNEL_INIT_CONFIG>());
            try
            {
                Marshal.StructureToPtr(cfg, pCfg, false);
                _channelHandle = ZlgCanNative.ZCAN_InitCAN(_deviceHandle, (uint)channel, pCfg);
            }
            finally
            {
                Marshal.FreeHGlobal(pCfg);
            }
            if (_channelHandle == IntPtr.Zero)
            {
                error = $"InitCAN 失败：通道 {channel}";
                RollbackOpen();
                return false;
            }

            if (_property.SetValue(ch + "/initenal_resistance", "1") == 0)
                Log?.Invoke("警告：终端电阻使能属性设置失败（设备可能已内置）");

            if (ZlgCanNative.ZCAN_StartCAN(_channelHandle) != ZlgCanNative.STATUS_OK)
            {
                error = "StartCAN 失败";
                RollbackOpen();
                return false;
            }

            // 接收线程
            _running = true;
            _rxThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "CanFdReceiveLoop" };
            _rxThread.Start();

            Log?.Invoke($"设备已打开：USBCANFD-200U 索引 {deviceIndex} 通道 {channel}，CANFD ISO / BRS，仲裁 1M / 数据 5M，终端电阻开；SDO及同步查询使用CANFD");
            return true;
        }
        catch (DllNotFoundException)
        {
            error = "未找到 ControlCANFD.dll（应位于程序目录，x64）";
            return false;
        }
        catch (Exception ex)
        {
            error = "打开设备异常：" + ex.Message;
            RollbackOpen();
            return false;
        }
    }

    /// <summary>关闭设备：停接收线程 → ResetCAN → ReleaseIProperty → CloseDevice。调用方须先执行急停。</summary>
    public void Close()
    {
        if (_deviceHandle == IntPtr.Zero && _channelHandle == IntPtr.Zero)
            return;

        _closing = true;
        Interlocked.Increment(ref _connectionGeneration);
        _running = false;
        _rxThread?.Join(500);
        _rxThread = null;

        // 放弃所有未完成的 SDO 等待
        lock (_sdoGate)
        {
            for (int i = 0; i <= MaxDeviceId; i++)
            {
                _sdoPending[i]?.TrySetResult(new SdoResponse(0x80, 0, 0, new byte[4]));
                _sdoPending[i] = null;
                _sdoRequests[i] = null;
            }
        }
        lock (_cacheGate)
        {
            Array.Clear(_feedback);
            Array.Clear(_lastFeedbackTick);
        }

        lock (_sendGate)
        {
            if (_channelHandle != IntPtr.Zero)
            {
                ZlgCanNative.ZCAN_ResetCAN(_channelHandle);
                _channelHandle = IntPtr.Zero;
            }
            if (_propertyHandle != IntPtr.Zero)
            {
                ZlgCanNative.ReleaseIProperty(_propertyHandle);
                _propertyHandle = IntPtr.Zero;
            }
            if (_deviceHandle != IntPtr.Zero)
            {
                ZlgCanNative.ZCAN_CloseDevice(_deviceHandle);
                _deviceHandle = IntPtr.Zero;
            }
        }
        Log?.Invoke("设备已关闭");
    }

    private void RollbackOpen()
    {
        if (_channelHandle != IntPtr.Zero)
        {
            ZlgCanNative.ZCAN_ResetCAN(_channelHandle);
            _channelHandle = IntPtr.Zero;
        }
        if (_propertyHandle != IntPtr.Zero)
        {
            ZlgCanNative.ReleaseIProperty(_propertyHandle);
            _propertyHandle = IntPtr.Zero;
        }
        if (_deviceHandle != IntPtr.Zero)
        {
            ZlgCanNative.ZCAN_CloseDevice(_deviceHandle);
            _deviceHandle = IntPtr.Zero;
        }
    }

    /// <summary>发送一帧（内部加锁，线程安全）。FD 帧置 BRS。</summary>
    public bool Send(CanFrame frame)
    {
        if (!IsOpen)
            return false;

        lock (_sendGate)
        {
            if (!IsOpen) return false;
            try
            {
                uint ret;
                if (frame.IsFd)
                {
                    var tx = new ZlgCanNative.ZCAN_TransmitFD_Data
                    {
                        frame = ToFdFrame(frame),
                        transmit_type = 0,
                    };
                    IntPtr p = Marshal.AllocHGlobal(Marshal.SizeOf<ZlgCanNative.ZCAN_TransmitFD_Data>());
                    try
                    {
                        Marshal.StructureToPtr(tx, p, false);
                        ret = ZlgCanNative.ZCAN_TransmitFD(_channelHandle, p, 1);
                    }
                    finally
                    {
                        Marshal.FreeHGlobal(p);
                    }
                }
                else
                {
                    var tx = new ZlgCanNative.ZCAN_Transmit_Data
                    {
                        frame = ToClassicFrame(frame),
                        transmit_type = 0,
                    };
                    IntPtr p = Marshal.AllocHGlobal(Marshal.SizeOf<ZlgCanNative.ZCAN_Transmit_Data>());
                    try
                    {
                        Marshal.StructureToPtr(tx, p, false);
                        ret = ZlgCanNative.ZCAN_Transmit(_channelHandle, p, 1);
                    }
                    finally
                    {
                        Marshal.FreeHGlobal(p);
                    }
                }
                if (ret == 1) Interlocked.Increment(ref _txCount);
                return ret == 1;
            }
            catch (Exception ex)
            {
                Log?.Invoke("发送异常：" + ex.Message);
                return false;
            }
        }
    }

    public int SendBatch(IReadOnlyList<CanFrame> frames)
    {
        if (frames.Count is < 1 or > 6 || frames.Any(f => !f.IsFd))
            throw new ArgumentException("Only 1-6 FD frames allowed", nameof(frames));
        lock (_sendGate)
        {
            if (!IsOpen) return 0;
            int size = Marshal.SizeOf<ZlgCanNative.ZCAN_TransmitFD_Data>();
            IntPtr buffer = Marshal.AllocHGlobal(size * frames.Count);
            try
            {
                for (int i = 0; i < frames.Count; i++) Marshal.StructureToPtr(new ZlgCanNative.ZCAN_TransmitFD_Data { frame = ToFdFrame(frames[i]), transmit_type = 0 }, buffer + i * size, false);
                uint count = ZlgCanNative.ZCAN_TransmitFD(_channelHandle, buffer, (uint)frames.Count);
                int accepted = count <= frames.Count ? (int)count : 0;
                Interlocked.Add(ref _txCount, accepted); return accepted;
            }
            catch (Exception ex) { Log?.Invoke("Batch send failed: " + ex.Message); return 0; }
            finally { Marshal.FreeHGlobal(buffer); }
        }
    }

    private static ZlgCanNative.canfd_frame ToFdFrame(CanFrame f)
        => new()
        {
            can_id = f.Id,
            len = f.Length,
            flags = f.Brs ? ZlgCanNative.CANFD_BRS : (byte)0,
            __res0 = 0,
            __res1 = 0,
            data = PadData(f, 64),
        };

    private static ZlgCanNative.can_frame ToClassicFrame(CanFrame f)
        => new()
        {
            can_id = f.Id,
            can_dlc = f.Length,
            __pad = 0,
            __res0 = 0,
            __res1 = 0,
            data = PadData(f, 8),
        };

    private static byte[] PadData(CanFrame f, int size)
    {
        var d = new byte[size];
        Array.Copy(f.Data, d, f.Length);
        return d;
    }

    /// <summary>读取反馈缓存（无数据返回 null）。</summary>
    public MotorFeedback? GetFeedback(int deviceId)
    {
        lock (_cacheGate)
            return _feedback[deviceId];
    }

    /// <summary>该设备最近 <paramref name="windowMs"/> 毫秒内是否收到过反馈（在线判定）。</summary>
    public bool IsDeviceAlive(int deviceId, int windowMs = 500)
    {
        lock (_cacheGate)
            return _lastFeedbackTick[deviceId] != 0 &&
                   Environment.TickCount64 - _lastFeedbackTick[deviceId] < windowMs;
    }

    /// <summary>反馈帧时间戳（TickCount64，未收到过为 0）。</summary>
    public long GetFeedbackTick(int deviceId)
    {
        lock (_cacheGate)
            return _lastFeedbackTick[deviceId];
    }

    /// <summary>
    /// SDO 请求-响应配对：同一设备串行排队；返回 null = 发送失败或超时；
    /// 响应（含 abort）由 SdoMessage.Parse 解析后返回。
    /// </summary>
    private int _connectionGeneration;

    public async Task<SdoResponse?> SdoRequestAsync(int deviceId, CanFrame request, int timeoutMs = 300)
    {
        if (!IsOpen || deviceId is < 1 or > MaxDeviceId) return null;
        int generation = _connectionGeneration;
        await _sdoLocks[deviceId].WaitAsync().ConfigureAwait(false);
        try { return generation == _connectionGeneration ? await SdoRequestCoreAsync(deviceId, request, timeoutMs).ConfigureAwait(false) : null; }
        finally { _sdoLocks[deviceId].Release(); }
    }

    public async Task<string?> ReadSoftwareVersionAsync(int deviceId, bool isFd = true)
    {
        if (!IsOpen || deviceId is < 1 or > MaxDeviceId) return null;
        int generation = _connectionGeneration;
        await _sdoLocks[deviceId].WaitAsync().ConfigureAwait(false);
        try
        {
            return await SdoStringReader.ReadAsync(deviceId, Od.SoftwareVersion, request =>
                generation == _connectionGeneration ? SdoRequestCoreAsync(deviceId, new CanFrame(request.Id, request.Data, isFd: isFd), 300) : Task.FromResult<SdoResponse?>(null)).ConfigureAwait(false);
        }
        finally { _sdoLocks[deviceId].Release(); }
    }

    private async Task<SdoResponse?> SdoRequestCoreAsync(int deviceId, CanFrame request, int timeoutMs)
    {
        if (!IsOpen) return null;
        var tcs = new TaskCompletionSource<SdoResponse>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_sdoGate) { _sdoPending[deviceId] = tcs; _sdoRequests[deviceId] = request; }
        try
        {
            if (!Send(request)) return null;
            var completed = await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs)).ConfigureAwait(false);
            return completed == tcs.Task ? await tcs.Task.ConfigureAwait(false) : null;
        }
        finally { lock (_sdoGate) { _sdoPending[deviceId] = null; _sdoRequests[deviceId] = null; } }
    }

    // ---------------- 接收线程 ----------------

    private void ReceiveLoop()
    {
        int fdSize = Marshal.SizeOf<ZlgCanNative.ZCAN_ReceiveFD_Data>();
        int clSize = Marshal.SizeOf<ZlgCanNative.ZCAN_Receive_Data>();
        IntPtr fdBuf = Marshal.AllocHGlobal(fdSize * RxBatch);
        IntPtr clBuf = Marshal.AllocHGlobal(clSize * RxBatch);
        try
        {
            while (_running)
            {
                bool any = false;

                uint nFd = ZlgCanNative.ZCAN_GetReceiveNum(_channelHandle, (byte)ZlgCanNative.TYPE_CANFD);
                if (nFd > 0)
                {
                    any = true;
                    uint got = ZlgCanNative.ZCAN_ReceiveFD(_channelHandle, fdBuf, Math.Min(nFd, RxBatch), 0);
                    for (int i = 0; i < got; i++)
                    {
                        var rd = Marshal.PtrToStructure<ZlgCanNative.ZCAN_ReceiveFD_Data>(fdBuf + i * fdSize);
                        if (Interlocked.Increment(ref _rawFd) <= 8)
                            Log?.Invoke($"原始FD RX ID=0x{rd.frame.can_id:X8} len={rd.frame.len}");
                        if (CanFrame.TryNormalizeReceivedId(rd.frame.can_id, true, out uint id)) ProcessFrame(id, rd.frame.len, rd.frame.data);
                    }
                }

                uint nCl = ZlgCanNative.ZCAN_GetReceiveNum(_channelHandle, (byte)ZlgCanNative.TYPE_CAN);
                if (nCl > 0)
                {
                    any = true;
                    uint got = ZlgCanNative.ZCAN_Receive(_channelHandle, clBuf, Math.Min(nCl, RxBatch), 0);
                    for (int i = 0; i < got; i++)
                    {
                        var rd = Marshal.PtrToStructure<ZlgCanNative.ZCAN_Receive_Data>(clBuf + i * clSize);
                        if (Interlocked.Increment(ref _rawCan) <= 8)
                            Log?.Invoke($"原始CAN RX ID=0x{rd.frame.can_id:X8} len={rd.frame.can_dlc}");
                        if (CanFrame.TryNormalizeReceivedId(rd.frame.can_id, false, out uint id)) ProcessFrame(id, rd.frame.can_dlc, rd.frame.data);
                    }
                }

                if (!any)
                    Thread.Sleep(10);
            }
        }
        catch (Exception ex)
        {
            Log?.Invoke("接收线程异常：" + ex.Message);
        }
        finally
        {
            Marshal.FreeHGlobal(fdBuf);
            Marshal.FreeHGlobal(clBuf);
        }
    }

    private void ProcessFrame(uint id, byte len, byte[] data)
    {
        Interlocked.Increment(ref _rxCount);
        if (len == 0 || len > data.Length || len > 64)
            return;

        RawFrameReceived?.Invoke(new CanFrame(id, data.AsSpan(0, len), isFd: len > 8));
        // 0x300+ID 反馈帧
        if (id > Od.FeedbackBase && id <= Od.FeedbackBase + MaxDeviceId)
        {
            int devId = (int)(id - Od.FeedbackBase);
            if (len == 12)
            {
                var frame = new CanFrame(id, data.AsSpan(0, len));
                bool firmware117;
                lock (_cacheGate) firmware117 = _firmware117Ids.Contains(devId);
                var fb = FeedbackParser.Parse(devId, frame, firmware117);
                lock (_cacheGate)
                {
                    _feedback[devId] = fb;
                    _lastFeedbackTick[devId] = Environment.TickCount64;
                }
                if (_lastFeedbackTick[devId] != 0 && _feedbackCount < 6)
                    Log?.Invoke($"反馈 ID {devId} {Convert.ToHexString(frame.Data)} 布局={(firmware117 ? "Firmware1.17" : "PDF2026")} 故障=0x{fb.FaultCode:X4} 状态使能={fb.Enabled}");
                Interlocked.Increment(ref _feedbackCount);
                FeedbackUpdated?.Invoke(devId, fb);
            }
            else
            {
                Interlocked.Increment(ref _badFeedbackCount);
            }
            return;
        }

        // 0x580+ID SDO 响应
        if (id > Od.SdoRspBase && id <= Od.SdoRspBase + MaxDeviceId)
        {
            if (len != 8) return;
            int devId = (int)(id - Od.SdoRspBase);
            var frame = new CanFrame(id, data.AsSpan(0, len));
            var response = SdoMessage.Parse(frame);
            TaskCompletionSource<SdoResponse>? tcs;
            lock (_sdoGate)
            {
                var request = _sdoRequests[devId];
                if (request == null || !SdoMessage.Matches(request, response)) return;
                tcs = _sdoPending[devId];
                _sdoPending[devId] = null;
            }
            tcs?.TrySetResult(response);
            return;
        }

        // 0x081..0x0FF EMCY 紧急故障帧
        if (id >= Od.EmcyBase + 1 && id <= 0x0FF)
        {
            var frame = new CanFrame(id, data.AsSpan(0, len));
            Log?.Invoke($"EMCY 故障帧 {frame}");
            if (len >= 2)
            {
                int devId = (int)(id - Od.EmcyBase);
                var fault = new MotorFeedback { DeviceId = devId, HasError = true,
                    FaultCode = (ushort)(data[0] | data[1] << 8) };
                lock (_cacheGate) _feedback[devId] = fault;
                FeedbackUpdated?.Invoke(devId, fault);
            }
            return;
        }

        // 0x700+ID 节点上线帧
        if (id > Od.BootBase && id <= Od.BootBase + MaxDeviceId)
        {
            int devId = (int)(id - Od.BootBase);
            Log?.Invoke($"节点上线：设备 ID {devId}（0x700+ID，boot 完成）");
            return;
        }

        // 其余帧（如 ID 9/10 的其它报文）
        FrameReceived?.Invoke(new CanFrame(id, data.AsSpan(0, len)));
    }

    public string GetBusDiagnostics()
    {
        lock (_sendGate)
        {
            if (!IsOpen) return "适配器未打开";
            string counters = $"原始CAN={Interlocked.Read(ref _rawCan)}，原始FD={Interlocked.Read(ref _rawFd)}，不支持的反馈长度={BadFeedbackCount}";
            try
            {
                uint result = ZlgCanNative.ZCAN_ReadChannelErrInfo(_channelHandle, out var info);
                return counters + $"；通道错误读取返回={result}，错误码=0x{info.error_code:X8}，被动错误={info.passive0:X2} {info.passive1:X2} {info.passive2:X2}，仲裁丢失={info.arbitrationLost:X2}";
            }
            catch (Exception ex) { return counters + "；错误状态读取失败：" + ex.Message; }
        }
    }

    public void Dispose() => Close();
}
