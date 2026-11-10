using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;
using WaistModel;

namespace waist_motor_controller.UI;

/// <summary>
/// 主界面：顶部四页导航；通信页包含连接与日志，关节页包含选择与控制。
/// 使用DPI缩放与自绘交互状态，小窗口保留垂直滚动。
/// 线程模型：接收线程只写缓存；UI 100ms 定时器拉取刷新；keepalive 独立位置发送线程。
/// </summary>
public sealed partial class MainForm : Form
{
    private const int MaxLogLines = 2000;
    private waist_motor_controller.WholeBody.WholeBodyPage? _wholeBodyPage;

    private readonly CanFdTransport _transport = new();
    private readonly MotorRegistry _registry;
    private readonly KeepaliveScheduler _keepalive;
    private readonly MotorController _controller;
    private readonly CalibrationStore _calibrationStore = new();

    // 框栏①：CANFD 连接
    private Label _connStateLabel = null!;
    private Label _busIdLabel = null!;
    private Label _mappedIdLabel = null!;
    private Label _rxTxLabel = null!;
    private Button _openCloseButton = null!;
    private Button _changeIdButton = null!;
    private Button _eStopButton = null!;

    // 框栏②：电机 ID 映射
    private readonly NumericUpDown[] _idNums = new NumericUpDown[AppConfig.MotorCount];
    private readonly Label[] _onlineLabels = new Label[AppConfig.MotorCount];

    // 框栏③：单电机控制
    private ComboBox _motorCombo = null!;
    private ComboBox _directionCombo = null!;
    private readonly int[] _pendingDirections = new int[AppConfig.MotorCount];
    private readonly ProfileConfig[] _jointProfiles = new ProfileConfig[AppConfig.MotorCount];
    private int _profileIndex = -1;
    private bool _switchingJointProfile;
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
    private Label _targetAngleCaption = null!;
    private Button _moveButton = null!;
    private Button _returnZeroButton = null!;
    private NumericUpDown _jogSpeedNum = null!;
    private NumericUpDown _jogAccelNum = null!;
    private Button _jogFwdButton = null!;
    private Button _jogRevButton = null!;
    private Button _stopMotionButton = null!;
    private NumericUpDown _jogStepNum = null!;
    private double _jogStepDegrees;
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
        _controller = new MotorController(_transport, _keepalive, _registry, _calibrationStore);

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
        _jogStepDegrees = (double)_jogStepNum.Value;
        _jogStepNum.ValueChanged += (_, _) =>
        {
            _jogStepDegrees = (double)_jogStepNum.Value;
            if (!_switchingJointProfile) RefreshStatus();
        };

        _uiTimer = new System.Windows.Forms.Timer { Interval = 100 };
        _uiTimer.Tick += (_, _) => RefreshStatus();
        _syncTimer = new System.Windows.Forms.Timer { Interval = 50 };
        _syncTimer.Tick += (_, _) =>
        {
            if (_wholeBodyPage?.PollsFeedback != true) _controller.SendSync();
        };
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

    private int CurrentLogicalIndex => MotorDisplayOrder[Math.Max(0, _motorCombo.SelectedIndex)];

    private int CurrentDeviceId => _registry.Motors[CurrentLogicalIndex].DeviceId;

    private double JogAccel => (double)_jogAccelNum.Value;

    private double PpProfileSpeed => (double)_jogSpeedNum.Value;

    private void LoadConfigToUi()
    {
        for (int i = 0; i < _idNums.Length; i++)
        {
            var motor = _registry.Motors[i];
            _idNums[i].Value = Math.Clamp(motor.DeviceId, 1, 127);
            _pendingDirections[i] = motor.Direction;
            // Older files only have one common profile. Use it as the initial value for each axis.
            _jointProfiles[i] = motor.Profile ?? _registry.Config.Profile;
        }
        ShowSelectedJointProfile();
        UpdateDirectionUi();
    }

    private void CaptureJointProfile()
    {
        if (_profileIndex < 0) return;
        _jointProfiles[_profileIndex] = new ProfileConfig
        {
            TargetDeg = (double)_targetAngleNum.Value,
            OutputSpeedRpm = PpProfileSpeed,
            OutputAccelRpmPerSec = JogAccel,
            JogStepDeg = (double)_jogStepNum.Value,
        };
    }

    private void ShowSelectedJointProfile()
    {
        int index = CurrentLogicalIndex;
        if (index == _profileIndex) return;
        _switchingJointProfile = true;
        try
        {
            CaptureJointProfile();
            _profileIndex = index;
            var profile = _jointProfiles[index];
            UpdateTargetAngleRange(_controller.GetJointLimits(_registry.Motors[index]));
            SetProfileValue(_targetAngleNum, profile.TargetDeg, 0);
            SetProfileValue(_jogSpeedNum, profile.OutputSpeedRpm, 1);
            SetProfileValue(_jogAccelNum, profile.OutputAccelRpmPerSec, 5);
            SetProfileValue(_jogStepNum, profile.JogStepDeg, 1);
        }
        finally { _switchingJointProfile = false; }
    }

    private static void SetProfileValue(NumericUpDown input, double value, double fallback)
    {
        double requested = double.IsFinite(value) ? value : fallback;
        decimal bounded = (decimal)Math.Clamp(requested, (double)input.Minimum, (double)input.Maximum);
        input.Value = Math.Clamp(bounded, input.Minimum, input.Maximum);
    }

    private void UpdateDirectionUi()
    {
        bool open = _transport.IsOpen;
        _directionCombo.Enabled = !open;
        int direction = open ? _registry.Motors[CurrentLogicalIndex].Direction : _pendingDirections[CurrentLogicalIndex];
        _directionCombo.SelectedIndex = direction == 1 ? 0 : 1;
    }

    // ================= 连接 / 设备 =================

    private void OnOpenClose()
    {
        if (!_transport.IsOpen)
        {
            if (!_registry.HasUniqueDeviceIds)
            {
                MessageBox.Show(this, "CAN ID 存在重复。请在关节控制页修正并保存映射后再打开设备。", "打开设备",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            string dll = Path.Combine(Application.StartupPath, "ControlCANFD.dll");
            if (!File.Exists(dll))
            {
                MessageBox.Show(this, $"未找到 {dll}\n请确认 native/ControlCANFD.dll 已复制到输出目录。", "错误",
                    MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            int devIdx = Math.Clamp(_registry.Config.Can.DeviceIndex, 0, 15);
            int channel = Math.Clamp(_registry.Config.Can.Channel, 0, 3);
            _controller.ReloadJointLimits();
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
            if (_wholeBodyPage?.PrepareClose() == false) { ShowNotice("全身运控正在停止或保存录制，请稍后再关闭设备"); return; }
            CloseDeviceSafely();
        }
        UpdateConnectionUi();
        RefreshStatus();
    }

    /// <summary>关闭设备前先急停（否则电机将在 500ms 后以看门狗自锁方式非受控抱闸）。</summary>
    private void CloseDeviceSafely()
    {
        ++_session;
        _wholeBodyPage?.ConnectionClosing();
        _readingFirmware = false;
        if (_transport.IsOpen)
            _controller.EmergencyStop();
        _uiTimer.Stop();
        _syncTimer.Stop();
        _logFlushTimer.Stop();
        _keepalive.Stop();
        _transport.Close();
        _wholeBodyPage?.ConnectionClosed();
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
        UpdateDirectionUi();
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
        var motor = _registry.Motors[CurrentLogicalIndex];
        int devId = motor.DeviceId;
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
            if (!motor.IsHead) _wholeBodyPage?.InvalidateCalibration();
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
        var ids = _idNums.Select(number => (int)number.Value).ToArray();
        if (ids.Distinct().Count() != ids.Length)
        {
            MessageBox.Show(this, "电机的总线 ID 存在重复，请修正后再保存。", "保存映射",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        var previousIds = _registry.Motors.Select(m => m.DeviceId).ToArray();
        for (int i = 0; i < ids.Length; i++)
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
        CaptureJointProfile();
        var previousProfiles = _registry.Motors.Select(m => m.Profile).ToArray();
        int[] previousDirections = _registry.Motors.Select(m => m.Direction).ToArray();
        for (int i = 0; i < _pendingDirections.Length; i++)
        {
            _registry.Motors[i].Direction = _pendingDirections[i];
            _registry.Motors[i].Profile = _jointProfiles[i];
        }
        if (!_registry.Save())
        {
            for (int i = 0; i < previousDirections.Length; i++)
            {
                _registry.Motors[i].DeviceId = previousIds[i];
                _registry.Motors[i].Direction = previousDirections[i];
                _registry.Motors[i].Profile = previousProfiles[i];
            }
        }
        RefreshMotorDisplayNames();
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
            string expected = string.IsNullOrEmpty(m.ExpectedFirmware) ? "" : $"；预期 {m.ExpectedFirmware}";
            AppendLog($"ID {m.DeviceId} {m.Model} 软件版本：{version ?? "未读到"}{expected}");
        }
        _readingFirmware = false;
        AppendLog("通信诊断：" + _transport.GetBusDiagnostics());
        if (_transport.RxFrameCount == 0)
            AppendLog("未收到任何总线报文。适配器打开成功不代表电机通信成功；请对照官方的接口、帧协议和接线。TX仅表示驱动接收了发送请求。");
        RefreshStatus();
    }

    // ================= 状态刷新（100ms 拉取缓存） =================

    private static string JointAngleText(MotorFeedback? feedback, int direction) =>
        feedback is null ? "—" : $"{(feedback.PositionRaw == 0 ? 0 : direction * feedback.PositionDeg):F2}°";

    private void RefreshStatus()
    {
        bool open = _transport.IsOpen;
        long now = Environment.TickCount64;
        var alive = new List<int>();
        int selectedIndex = CurrentLogicalIndex;
        var motor = _registry.Motors[selectedIndex];
        int idSelected = motor.DeviceId;
        MotorFeedback? fb = null;
        for (int i = 0; i < _idNums.Length; i++)
        {
            var mappedMotor = _registry.Motors[i];
            int id = mappedMotor.DeviceId;
            bool online = open && _transport.IsDeviceAlive(id);
            var feedback = online ? _transport.GetFeedback(id) : null;
            if (i == selectedIndex) fb = feedback;
            if (online) alive.Add(id);
            UiValueCache.Text(_onlineLabels[i], online ? "● 在线" : "● 离线");
            UiValueCache.Color(_onlineLabels[i], online ? OkColor : TextSub);
            UiValueCache.Text(_motorCardAngles[i], JointAngleText(feedback, mappedMotor.Direction));
            UiValueCache.Color(_motorCardAngles[i], feedback is null ? TextSub : UiTheme.ValueBlue);
        }
        UiValueCache.Text(_busIdLabel, open ? alive.Count == 0 ? "无" : string.Join(", ", alive.Order()) : "--");
        UiValueCache.Text(_mappedIdLabel, $"头部 {string.Join("、", _registry.HeadMotors.Select(m => m.DeviceId))} · " +
            $"腰部 {string.Join("、", _registry.WaistMotors.Select(m => m.DeviceId))}");
        if (!open || now - _lastCountersUiTick >= 500)
        {
            _lastCountersUiTick = now;
            UiValueCache.Text(_rxTxLabel, open ? $"{_transport.RxFrameCount:N0} / {_transport.TxFrameCount:N0}" : "--");
            _uiValues.Tip(_tips, _rxTxLabel, $"有效反馈：{_transport.FeedbackCount:N0} 帧；异常反馈长度：{_transport.BadFeedbackCount:N0} 帧");
        }
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
            UiValueCache.Text(_posLabel, JointAngleText(fb, motor.Direction));
            _uiValues.Tip(_tips, _posLabel, $"电机角：{fb.PositionDeg:F2}°（{fb.PositionRaw} cnt）");
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
            _controller.IsWithinEnableWindow(idSelected), moving, atTarget, clearable),
            fb?.PositionDeg * motor.Direction);
    }

    private void UpdateTargetAngleRange((double Minimum, double Maximum)? limits)
    {
        UiValueCache.Enabled(_targetAngleNum, limits.HasValue);
        if (limits is not { } range)
        {
            UiValueCache.Text(_targetAngleCaption, "绝对目标角度 · 标定无效");
            return;
        }

        decimal minimum = range.Minimum == Od.MinAngle ? -178.90M : (decimal)range.Minimum;
        decimal maximum = range.Maximum == Od.MaxAngle ? 178.90M : (decimal)range.Maximum;
        if (minimum > maximum) (minimum, maximum) = ((decimal)range.Minimum, (decimal)range.Maximum);
        if (_targetAngleNum.Minimum != minimum || _targetAngleNum.Maximum != maximum)
        {
            int Scale(decimal value) => (decimal.GetBits(value)[3] >> 16) & 0xff;
            _targetAngleNum.DecimalPlaces = Math.Max(2, Math.Max(Scale(minimum), Scale(maximum)));
            _targetAngleNum.Minimum = minimum;
            _targetAngleNum.Maximum = maximum;
        }
        string rangeText = $"{minimum:G29}～{maximum:G29}°";
        UiValueCache.Text(_targetAngleCaption, $"目标角度 {rangeText}");
        _uiValues.Tip(_tips, _targetAngleCaption, $"绝对目标角度允许范围 {rangeText}");
    }

    internal void ApplyOperationState(OperationUiState state, double? currentJointAngle = null)
    {
        var motor = _registry.Motors[CurrentLogicalIndex];
        var limits = _controller.GetJointLimits(motor);
        UpdateTargetAngleRange(limits);
        double step = _jogStepDegrees;
        UiValueCache.Enabled(_enableAllButton, state.Connected && !_controller.WholeBodyOwned);
        UiValueCache.Enabled(_enableButton, state.Connected && state.FeedbackFresh && !state.Faulted && !state.Configuring && !state.Armed);
        UiValueCache.Enabled(_disableButton, state.Connected && (state.Armed || (state.FeedbackFresh && state.MotorEnabled)));
        UiValueCache.Enabled(_clearErrButton, state.Connected && state.FeedbackFresh && state.Faulted && !state.Configuring && state.CommandClearable);
        // 不再随每次点动闪成禁用色。重入、保护期和标零到位检查在操作入口及控制层执行。
        UiValueCache.Enabled(_setZeroButton, state.CanCommand && !state.ReadingFirmware);
        UiValueCache.Enabled(_moveButton, state.CanCommand && limits.HasValue);
        UiValueCache.Enabled(_returnZeroButton, state.CanCommand && MotorController.InJointRange(limits, 0));
        UiValueCache.Enabled(_jogFwdButton, state.CanCommand && currentJointAngle is double forward &&
            MotorController.InJointRange(limits, forward + step));
        UiValueCache.Enabled(_jogRevButton, state.CanCommand && currentJointAngle is double backward &&
            MotorController.InJointRange(limits, backward - step));
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
        if (_wholeBodyPage?.PrepareClose() == false) { e.Cancel = true; return; }
        if (_transport.IsOpen)
            CloseDeviceSafely();
        _transport.Dispose();
        base.OnFormClosing(e);
    }
}
