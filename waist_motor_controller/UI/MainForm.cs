using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.UI;

/// <summary>
/// 主界面：顶部四页导航；通信页包含连接与日志，关节页包含选择与控制。
/// 使用DPI缩放与自绘交互状态，小窗口保留垂直滚动。
/// 线程模型：接收线程只写缓存；UI 100ms 定时器拉取刷新；keepalive 独立位置发送线程。
/// </summary>
public sealed partial class MainForm : Form
{
    private const int MaxLogLines = 2000;

    private readonly CanFdTransport _transport = new();
    private readonly MotorRegistry _registry;
    private readonly KeepaliveScheduler _keepalive;
    private readonly MotorController _controller;

    // 框栏①：CANFD 连接
    private Label _connStateLabel = null!;
    private Label _busIdLabel = null!;
    private Label _rxTxLabel = null!;
    private Button _openCloseButton = null!;
    private Button _changeIdButton = null!;
    private Button _eStopButton = null!;

    // 框栏②：电机 ID 映射
    private readonly NumericUpDown[] _idNums = new NumericUpDown[4];
    private readonly Label[] _onlineLabels = new Label[4];

    // 框栏③：单电机控制
    private ComboBox _motorCombo = null!;
    private Label _posLabel = null!;
    private Label _velLabel = null!;
    private Label _modeLabel = null!;
    private Label _curLabel = null!;
    private Label _faultLabel = null!;
    private Label _tempLabel = null!;
    private Label _firmwareLabel = null!;
    private readonly Label[] _stateLights = new Label[4];   // 使能/抱闸释放/报错/到位
    private Button _enableButton = null!;
    private Button _disableButton = null!;
    private Button _clearErrButton = null!;
    private Button _setZeroButton = null!;
    private NumericUpDown _targetAngleNum = null!;
    private Button _moveButton = null!;
    private Button _returnZeroButton = null!;
    private NumericUpDown _jogSpeedNum = null!;
    private NumericUpDown _jogAccelNum = null!;
    private Button _jogFwdButton = null!;
    private Button _jogRevButton = null!;
    private Button _stopMotionButton = null!;
    private NumericUpDown _jogStepNum = null!;
    private readonly Dictionary<int, string> _firmware = new();
    private readonly string _logPath = Path.Combine(Application.StartupPath, "logs", $"session-{DateTime.Now:yyyyMMdd-HHmmss}-{Environment.ProcessId}.log");
    private int _session;
    private bool _readingFirmware;

    // 框栏④：日志
    private TextBox _logBox = null!;

    private System.Windows.Forms.Timer _uiTimer = null!;      // 50ms 状态刷新
    private System.Windows.Forms.Timer _syncTimer = null!;    // 50ms 同步帧（反馈轮询）
    private System.Windows.Forms.Timer _logFlushTimer = null!;// 2s 高频帧汇总

    private long _lastFeedbackCount;
    private long _lastKeepaliveCount;
    private readonly UiValueCache _uiValues = new();
    private long _lastCountersUiTick;
    private string _noticeText = "";
    private long _noticeUntil;

    public MainForm()
    {
        string configDir = Path.Combine(Application.StartupPath, "config");
        _registry = new MotorRegistry(Path.Combine(configDir, "motors.json"));
        _keepalive = new KeepaliveScheduler(f => _transport.Send(f), CanContinuePosition,
            id => _registry.Motors.Any(m => m.DeviceId == id && m.UseFirmware117Pdo));
        _controller = new MotorController(_transport, _keepalive, _registry);

        _transport.Log += OnLogFromThread;
        _transport.FrameReceived += f => OnLogFromThread($"RX {f}");
        _registry.Log += OnLogFromThread;
        _controller.Log += OnLogFromThread;

        DoubleBuffered = true;
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer, true);
        SuspendLayout();
        AutoScaleDimensions = new SizeF(96F, 96F);
        AutoScaleMode = AutoScaleMode.Dpi;
        Font = UiTheme.Font(11);
        BuildUi();
        LoadConfigToUi();

        _uiTimer = new System.Windows.Forms.Timer { Interval = 100 };
        _uiTimer.Tick += (_, _) => RefreshStatus();
        _syncTimer = new System.Windows.Forms.Timer { Interval = 50 };
        _syncTimer.Tick += (_, _) => _controller.SendSync();
        _logFlushTimer = new System.Windows.Forms.Timer { Interval = 2000 };
        _logFlushTimer.Tick += (_, _) => FlushFrameStats();

        Text = "yclab折叠腰控制台";
        using (var iconStream = typeof(MainForm).Assembly.GetManifestResourceStream("waist_motor_controller.yclab.ico"))
        {
            if (iconStream != null)
            {
                using var logo = new Icon(iconStream);
                Icon = (Icon)logo.Clone();
            }
        }
        StartPosition = FormStartPosition.CenterScreen;
        FormBorderStyle = FormBorderStyle.None;
        Controls.Add(new WindowTitleBar(this));
        BackColor = BgColor;
        ResumeLayout(true);
        MinimumSize = new Size(1200, 760);
        var workingArea = Screen.FromControl(this).WorkingArea;
        ClientSize = new Size(Math.Min(1440, workingArea.Width - 80), Math.Min(980, workingArea.Height - 80));
        Deactivate += (_, _) =>
        {
            foreach (var m in _registry.Motors)
                if (_controller.IsJogging(m.DeviceId)) _controller.StopMotion(m.DeviceId);
        };

        AppendLog(_registry.LoadMessage);
        AppendLog("定位与增量点动使用输出端角度；首次使用请低速、小角度核对方向和机械行程。");
        UpdateConnectionUi();
        RefreshStatus();
    }

    private bool CanContinuePosition(int id)
    {
        var fb = _transport.GetFeedback(id);
        return _transport.IsDeviceAlive(id) && fb != null && !fb.HasError && fb.FaultCode == 0 &&
            (_controller.IsWithinEnableWindow(id) || (fb.Enabled && fb.BrakeReleased));
    }

    // ================= 辅助 =================

    private int CurrentLogicalIndex => Math.Max(0, _motorCombo.SelectedIndex);

    private int CurrentDeviceId => _registry.Motors[CurrentLogicalIndex].DeviceId;

    private double JogAccel => (double)_jogAccelNum.Value;

    private double PpProfileSpeed => (double)_jogSpeedNum.Value;

    private void LoadConfigToUi()
    {
        _jogSpeedNum.Value = (decimal)Math.Clamp(_registry.Config.Profile.OutputSpeedRpm, 0.05, 5);
        _jogAccelNum.Value = (decimal)Math.Clamp(_registry.Config.Profile.OutputAccelRpmPerSec, 0.1, 50);
        _jogStepNum.Value = (decimal)Math.Clamp(_registry.Config.Profile.JogStepDeg, 0.01, 30);
        for (int i = 0; i < 4; i++)
            _idNums[i].Value = Math.Clamp(_registry.Motors[i].DeviceId, 1, 127);
    }

    // ================= 连接 / 设备 =================

    private void OnOpenClose()
    {
        if (!_transport.IsOpen)
        {
            string dll = Path.Combine(Application.StartupPath, "ControlCANFD.dll");
            if (!File.Exists(dll))
            {
                MessageBox.Show(this, $"未找到 {dll}\n请确认 native/ControlCANFD.dll 已复制到输出目录。", "错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            int devIdx = Math.Clamp(_registry.Config.Can.DeviceIndex, 0, 15);
            int channel = Math.Clamp(_registry.Config.Can.Channel, 0, 3);
            _transport.ConfigureFirmware117(_registry.Motors.Where(m => m.UseFirmware117Pdo).Select(m => m.DeviceId));
            if (!_transport.Open(devIdx, channel, out string err))
            {
                MessageBox.Show(this, err, "打开设备失败", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            _lastCountersUiTick = 0;
            _keepalive.Start();
            _uiTimer.Start();
            _syncTimer.Start();
            _logFlushTimer.Start();
            _firmware.Clear();
            ++_session;
            _ = ReadFirmwareAsync(_session);
            AppendLog("已连接：反馈查询50ms、独立线程同步位置发送、UI刷新100ms");
        }
        else
        {
            CloseDeviceSafely();
        }
        UpdateConnectionUi();
        RefreshStatus();
    }

    /// <summary>关闭设备前先急停（否则电机将在 500ms 后以看门狗自锁方式非受控抱闸）。</summary>
    private void CloseDeviceSafely()
    {
        ++_session;
        _readingFirmware = false;
        if (_transport.IsOpen)
            _controller.EmergencyStop();
        _uiTimer.Stop();
        _syncTimer.Stop();
        _logFlushTimer.Stop();
        _keepalive.Stop();
        _transport.Close();
    }

    private void UpdateConnectionUi()
    {
        bool open = _transport.IsOpen;
        _connStateLabel.Text = open ? "● 适配器已打开" : "● 未连接";
        _connStateLabel.ForeColor = open ? OkColor : TextSub;
        _openCloseButton.Text = open ? "关闭设备" : "打开设备";
        _changeIdButton.Enabled = open;
        _eStopButton.Enabled = open;
        _saveMappingButton.Enabled = !open;
        foreach (var number in _idNums) number.Enabled = !open;
    }

    // ================= 操作处理 =================

    private void RunMotorOp(string title, Func<(bool Ok, string Message)> op, bool nonModalFailure = false)
    {
        var (ok, msg) = op();
        AppendLog($"{title}：{msg}");
        if (!ok)
        {
            if (nonModalFailure) ShowNotice(msg);
            else MessageBox.Show(this, msg, title + "失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }
        RefreshStatus();
    }

    private void ShowNotice(string message)
    {
        _noticeText = message; _noticeUntil = Environment.TickCount64 + 1600;
        UiValueCache.Text(_motionHint, message);
    }

    private void RunMotionOp(string title, Func<(bool Ok, string Message)> op)
    {
        if (_controller.IsJogging(CurrentDeviceId))
        {
            ShowNotice("当前运动尚未完成，请等待到位或点击“停止 / 保持”。");
            return;
        }
        RunMotorOp(title, op, nonModalFailure: true);
    }

    private async Task OnSetZero()
    {
        var devId = CurrentDeviceId;
        if (_controller.IsJogging(devId) || !_controller.IsAtTarget(devId))
        {
            ShowNotice("请先停止并等待电机到位，再设置零位。");
            return;
        }
        var result = MessageBox.Show(this,
            $"将对设备 ID {devId} 执行零位标定（标定 → 失能 → 复位 → 校验，结束后手动使能）。\n" +
            "请确认电机已位于目标零位。继续？",
            "设置为原点", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
        if (result != DialogResult.Yes)
            return;

        _setZeroButton.Enabled = false;
        try
        {
            var (ok, msg) = await _controller.SetZeroOriginAsync(devId, JogAccel, PpProfileSpeed);
            if (!ok)
                MessageBox.Show(this, msg, "标零失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            else
                AppendLog(msg);
        }
        finally
        {
            RefreshStatus();
        }
    }

    private void OnChangeId()
    {
        if (_readingFirmware) { AppendLog("请等待版本读取结束"); return; }
        var used = new HashSet<int>(_registry.Motors.Select(m => m.DeviceId));
        using var dlg = new ChangeMotorIdDialog(CurrentDeviceId, used);
        if (dlg.ShowDialog(this) != DialogResult.OK)
            return;

        _ = RunChangeIdAsync(dlg.CurrentId, dlg.NewId);
    }

    private async Task RunChangeIdAsync(int currentId, int newId)
    {
        int logicalIndex = CurrentLogicalIndex;
        var (ok, msg) = await _controller.ChangeDeviceIdAsync(currentId, newId);
        AppendLog(msg);
        if (!ok)
        {
            MessageBox.Show(this, msg, "更改电机ID失败", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        var result = MessageBox.Show(this,
            msg + "\n\n是否立即更新映射框栏中当前选中电机的 ID？（仅更新界面数值，不自动保存）",
            "更改电机ID", MessageBoxButtons.YesNo, MessageBoxIcon.Question);
        if (result == DialogResult.Yes)
        {
            _idNums[logicalIndex].Value = newId;
            AppendLog($"映射框栏已更新：设备 {currentId} → {newId}（点击[保存映射]写入 json）");
        }
    }

    private void OnSaveMapping()
    {
        if (_transport.IsOpen) { AppendLog("请先关闭设备再保存映射及参数"); return; }
        // 检查 ID 重复
        var ids = new List<int>();
        for (int i = 0; i < 4; i++)
            ids.Add((int)_idNums[i].Value);
        if (ids.Distinct().Count() != 4)
        {
            MessageBox.Show(this, "4 个电机的总线 ID 存在重复，请修正后再保存。", "保存映射",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        for (int i = 0; i < 4; i++)
        {
            var motor = _registry.Motors[i];
            int newId = ids[i];
            if (newId != motor.DeviceId)
            {
                // 清除旧 ID 的 keepalive（映射变更，旧控制帧不再有效）
                _keepalive.Disarm(motor.DeviceId);
                _registry.SetDeviceId(motor.LogicalId, newId);
            }
        }
        _registry.Config.Profile.OutputSpeedRpm = PpProfileSpeed;
        _registry.Config.Profile.OutputAccelRpmPerSec = JogAccel;
        _registry.Config.Profile.JogStepDeg = (double)_jogStepNum.Value;
        _registry.Save();
        RefreshStatus();
    }

    private async Task ReadFirmwareAsync(int session)
    {
        _readingFirmware = true;
        foreach (var m in _registry.Motors)
        {
            string? version = await _transport.ReadSoftwareVersionAsync(m.DeviceId);
            if (session != _session || IsDisposed) return;
            if (version == null)
            {
                AppendLog($"ID {m.DeviceId} CANFD版本查询无有效响应，尝试只读CAN2.0查询以诊断帧格式");
                version = await _transport.ReadSoftwareVersionAsync(m.DeviceId, isFd: false);
                if (session != _session || IsDisposed) return;
                if (version != null) AppendLog($"ID {m.DeviceId} 仅CAN2.0版本读取成功：请核对电机服务报文格式");
            }
            _firmware[m.DeviceId] = version ?? "未读到";
            AppendLog($"ID {m.DeviceId} {m.Model} 软件版本：{version ?? "未读到"}；预期 {m.ExpectedFirmware}");
        }
        _readingFirmware = false;
        AppendLog("通信诊断：" + _transport.GetBusDiagnostics());
        if (_transport.RxFrameCount == 0)
            AppendLog("未收到任何总线报文。适配器打开成功不代表电机通信成功；请对照官方的接口、帧协议和接线。TX仅表示驱动接收了发送请求。");
        RefreshStatus();
    }

    // ================= 状态刷新（50ms 拉取缓存） =================

    private void RefreshStatus()
    {
        bool open = _transport.IsOpen;
        long now = Environment.TickCount64;
        var alive = new List<string>();
        for (int i = 0; i < 4; i++)
        {
            int id = _registry.Motors[i].DeviceId;
            bool online = open && _transport.IsDeviceAlive(id);
            if (online) alive.Add(id.ToString());
            UiValueCache.Text(_onlineLabels[i], online ? "● 在线" : "● 离线");
            UiValueCache.Color(_onlineLabels[i], online ? OkColor : TextSub);
        }
        UiValueCache.Text(_busIdLabel, open ? alive.Count == 0 ? "无" : string.Join(", ", alive) : "--");
        if (!open || now - _lastCountersUiTick >= 500)
        {
            _lastCountersUiTick = now;
            UiValueCache.Text(_rxTxLabel, open ? $"{_transport.RxFrameCount:N0} / {_transport.TxFrameCount:N0}" : "--");
            _uiValues.Tip(_tips, _rxTxLabel, $"有效反馈：{_transport.FeedbackCount:N0} 帧；异常反馈长度：{_transport.BadFeedbackCount:N0} 帧");
        }
        int idSelected = CurrentDeviceId;
        var fb = open && _transport.IsDeviceAlive(idSelected) ? _transport.GetFeedback(idSelected) : null;
        var motor = _registry.Motors[CurrentLogicalIndex];
        if (fb == null)
        {
            foreach (var label in new[] { _posLabel, _velLabel, _modeLabel, _curLabel, _faultLabel, _tempLabel }) UiValueCache.Text(label, "—");
            UiValueCache.Color(_faultLabel, TextSub);
            UiValueCache.Text(_firmwareLabel, $"{motor.Model} · {_firmware.GetValueOrDefault(idSelected, "—")}");
            _uiValues.Tip(_tips, _posLabel, "等待有效反馈");
            _uiValues.Tip(_tips, _faultLabel, "等待有效反馈");
        }
        else
        {
            UiValueCache.Text(_posLabel, $"{fb.PositionDeg:F2}°");
            _uiValues.Tip(_tips, _posLabel, $"输出端位置原值：{fb.PositionRaw} cnt");
            UiValueCache.Text(_velLabel, $"{fb.VelocityRpm} RPM");
            UiValueCache.Text(_modeLabel, fb.ModeName);
            UiValueCache.Text(_curLabel, $"{fb.CurrentMa} mA");
            string fault = FaultCodes.Describe(fb.FaultCode);
            UiValueCache.Text(_faultLabel, fb.FaultCode == 0 && fb.HasError ? "故障状态（代码尚未上报）" : fault);
            UiValueCache.Color(_faultLabel, fb.HasError || fb.FaultCode != 0 ? UiTheme.Red : OkColor);
            _uiValues.Tip(_tips, _faultLabel, fault);
            UiValueCache.Text(_tempLabel, fb.IsFirmware117 ? $"{fb.TemperatureRaw * 0.1:F1} °C" : $"{fb.TemperatureRaw} (raw)");
            UiValueCache.Text(_firmwareLabel, $"{motor.Model} · {_firmware.GetValueOrDefault(idSelected, "—")}");
        }
        SetLights(fb);
        bool faulted = fb != null && (fb.HasError || fb.FaultCode != 0);
        bool clearable = fb != null && (fb.FaultCode == 0 || FaultCodes.CanClearWithCommand(fb.FaultCode));
        bool armed = _keepalive.IsArmed(idSelected);
        bool atTarget = _controller.IsAtTarget(idSelected);
        bool moving = _controller.IsJogging(idSelected) || (armed && fb?.Enabled == true && !atTarget);
        ApplyOperationState(new OperationUiState(open, fb != null, faulted, _controller.IsBusy(idSelected),
            armed, fb?.Enabled == true, _readingFirmware,
            _controller.IsWithinEnableWindow(idSelected), moving, atTarget, clearable));
    }

    internal void ApplyOperationState(OperationUiState state)
    {
        UiValueCache.Enabled(_returnAllZeroButton, state.Connected);
        UiValueCache.Enabled(_enableAllButton, state.Connected);
        UiValueCache.Enabled(_enableButton, state.Connected && state.FeedbackFresh && !state.Faulted && !state.Configuring && !state.Armed);
        UiValueCache.Enabled(_disableButton, state.Connected && (state.Armed || (state.FeedbackFresh && state.MotorEnabled)));
        UiValueCache.Enabled(_clearErrButton, state.Connected && state.FeedbackFresh && state.Faulted && !state.Configuring && state.CommandClearable);
        // 不再随每次点动闪成禁用色。重入、保护期和标零到位检查在操作入口及控制层执行。
        UiValueCache.Enabled(_setZeroButton, state.CanCommand && !state.ReadingFirmware);
        UiValueCache.Enabled(_moveButton, state.CanCommand);
        UiValueCache.Enabled(_returnZeroButton, state.CanCommand);
        UiValueCache.Enabled(_jogFwdButton, state.CanCommand);
        UiValueCache.Enabled(_jogRevButton, state.CanCommand);
        UiValueCache.Enabled(_stopMotionButton, state.Connected && !state.Configuring && state.Armed);
        UiValueCache.Text(_motionHint, state.CanCommand && Environment.TickCount64 < _noticeUntil ? _noticeText : state.Hint);
    }

    private void SetLights(MotorFeedback? fb)
    {
        bool[] values = { fb?.Enabled == true, fb?.BrakeReleased == true, fb?.HasError == true || fb?.FaultCode > 0,
            fb?.TargetReached ?? (fb != null && _controller.IsAtTarget(fb.DeviceId)) };
        UiValueCache.Text(_stateLights[3], fb?.TargetReached.HasValue == true ? "● 到位" : "● 到位估算");
        Color[] colors = { OkColor, AccentColor, UiTheme.Red, UiTheme.Amber };
        for (int i = 0; i < values.Length; i++) UiValueCache.Color(_stateLights[i], values[i] ? colors[i] : TextSub);
    }

    /// <summary>高频帧 2s 汇总一条，避免日志刷屏。</summary>
    private void FlushFrameStats()
    {
        if (!_transport.IsOpen)
            return;
        long fb = _transport.FeedbackCount - _lastFeedbackCount;
        long ka = _keepalive.SentCount - _lastKeepaliveCount;
        _lastFeedbackCount = _transport.FeedbackCount;
        _lastKeepaliveCount = _keepalive.SentCount;
        AppendLog($"[2s 汇总] RX {_transport.RxFrameCount}（反馈帧 {fb}） TX {_transport.TxFrameCount}（位置帧 {ka}）");
    }

    // ================= 日志 =================

    private void OnLogFromThread(string message)
    {
        if (IsDisposed || Disposing) return;
        if (IsHandleCreated)
        {
            try { BeginInvoke(() => { if (!IsDisposed) AppendLog(message); }); }
            catch (InvalidOperationException) { }
        }
        else
            AppendLog(message);
    }

    private void AppendLog(string message)
    {
        string line = $"[{DateTime.Now:HH:mm:ss.fff}] {message}";
        if (_logBox.TextLength > 400_000)
            _logBox.Clear();
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(_logPath)!);
            File.AppendAllText(_logPath, line + Environment.NewLine);
        }
        catch (IOException) { }
        catch (UnauthorizedAccessException) { }
        _logBox.AppendText(line + Environment.NewLine);
        _logBox.SelectionStart = _logBox.TextLength;
        _logBox.ScrollToCaret();
        // 限量 2000 行环形裁剪
        if (_logBox.Lines.Length > MaxLogLines)
        {
            var keep = _logBox.Lines.Skip(_logBox.Lines.Length - MaxLogLines / 2).ToArray();
            _logBox.Lines = keep;
        }
    }

    protected override void OnHandleCreated(EventArgs e)
    {
        base.OnHandleCreated(e);
        WindowFrameTheme.Apply(Handle);
    }

    protected override void OnActivated(EventArgs e)
    {
        base.OnActivated(e);
        WindowFrameTheme.Apply(Handle);
    }

    protected override void OnShown(EventArgs e)
    {
        base.OnShown(e);
        WindowFrameTheme.Apply(Handle);
    }

    // ================= 关闭 =================

    protected override void OnFormClosed(FormClosedEventArgs e)
    {
        _tips.Dispose();
        _uiTimer.Dispose(); _syncTimer.Dispose(); _logFlushTimer.Dispose();
        base.OnFormClosed(e);
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        if (_transport.IsOpen)
            CloseDeviceSafely();
        _transport.Dispose();
        base.OnFormClosing(e);
    }
}
