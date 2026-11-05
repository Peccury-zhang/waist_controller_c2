using waist_motor_controller.Can;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.Control;

/// <summary>只控制注册的四个节点；确认协议分支用PP报文接收主机生成的轨迹目标。</summary>
public sealed class MotorController
{
    private readonly object _gate = new();
    private readonly ICanTransport _transport;
    private readonly KeepaliveScheduler _keepalive;
    private readonly MotorRegistry _registry;
    private readonly long[] _enabledSince = new long[128];
    private readonly bool[] _calibrating = new bool[128];
    private int _generation;
    public event Action<string>? Log;

    public MotorController(ICanTransport transport, KeepaliveScheduler keepalive, MotorRegistry registry)
    {
        _transport = transport; _keepalive = keepalive; _registry = registry;
        _transport.FeedbackUpdated += (id, fb) =>
        {
            if (Configured(id) && (fb.HasError || fb.FaultCode != 0) && _keepalive.IsArmed(id))
            {
                _keepalive.SendDisabled(id);
                Log?.Invoke($"电机 {id} 故障，已撤销轨迹并发送失能：{FaultCodes.Describe(fb.FaultCode)}");
            }
        };
        _keepalive.AxisStopped += (id, reason) => Log?.Invoke($"电机 {id}：{reason}");
    }
    private bool Configured(int id) => _registry.Motors.Any(m => m.DeviceId == id);
    public bool IsJogging(int id) => _keepalive.IsMoving(id);
    public bool IsBusy(int id) { lock (_gate) return _calibrating[id]; }
    public bool IsWithinEnableWindow(int id) => _enabledSince[id] != 0 &&
        Environment.TickCount64 - _enabledSince[id] < 150;
    private string? Check(int id, bool motion = false)
    {
        if (!Configured(id)) return "只允许控制已保存映射的四个电机";
        if (!_transport.IsOpen || !_transport.IsDeviceAlive(id)) return $"电机 {id} 无新鲜反馈（500ms）";
        if (_calibrating[id]) return "标零处理中";
        var fb = _transport.GetFeedback(id);
        if (fb == null) return "未收到反馈";
        if (fb.HasError || fb.FaultCode != 0) return FaultCodes.Describe(fb.FaultCode);
        if (motion && (!fb.Enabled || !fb.BrakeReleased || !_keepalive.IsArmed(id))) return "请先通过本程序使能电机";
        if (motion && IsWithinEnableWindow(id)) return "使能后请等待150ms再运动";
        return null;
    }
    public (bool Ok, string Message) Enable(int id, double accel, double speed)
    {
        lock (_gate)
        {
            if (Check(id) is { } error) return (false, error);
            if (_keepalive.IsArmed(id)) return (false, "已使能，请勿重复使能");
            var fb = _transport.GetFeedback(id)!;
            _enabledSince[id] = Environment.TickCount64;
            if (!_keepalive.ArmHold(id, fb.PositionRaw))
            { _enabledSince[id] = 0; return (false, "使能发送失败"); }
            Log?.Invoke($"电机 {id} 使能：保持当前位置 {fb.PositionDeg:F3}°");
            return (true, "保持当前位置，使能帧已发送");
        }
    }
    public (bool Ok, string Message) EnableAll(double accel, double speed)
    {
        lock (_gate)
        {
            int[] ids = _registry.Motors.Select(m => m.DeviceId).ToArray();
            foreach (int id in ids)
                if (Check(id) is { } error) return (false, $"ID {id}：{error}；未执行全部使能");
            var enabled = new List<int>();
            var skipped = new List<int>();
            foreach (int id in ids)
            {
                if (_keepalive.IsArmed(id)) { skipped.Add(id); continue; }
                var result = Enable(id, accel, speed);
                if (!result.Ok)
                {
                    var stop = EmergencyStop();
                    return (false, $"ID {id} 使能失败：{result.Message}；已撤销全部使能。{stop.Message}");
                }
                enabled.Add(id);
            }
            string message = enabled.Count == 0 ? "四个关节均已通过本程序使能" :
                $"ID {string.Join(", ", enabled)} 使能帧已发送，保持各自当前位置";
            if (skipped.Count > 0) message += $"；跳过已使能 ID {string.Join(", ", skipped)}";
            Log?.Invoke(message);
            return (true, message);
        }
    }
    public (bool Ok, string Message) Disable(int id)
    {
        lock (_gate)
        {
            if (!Configured(id)) return (false, "节点未配置");
            ++_generation;
            _enabledSince[id] = 0;
            bool sent = _keepalive.SendDisabled(id);
            return (sent, sent ? "失能帧已发送" : "失能发送失败");
        }
    }
    public (bool Ok, string Message) ClearError(int id)
    {
        lock (_gate)
        {
            if (!Configured(id) || _calibrating[id]) return (false, "节点未配置或正在标零");
            var fb = _transport.GetFeedback(id);
            if (fb != null && fb.FaultCode != 0)
            {
                if (FaultCodes.RequiresRestart(fb.FaultCode))
                    return (false, FaultCodes.Describe(fb.FaultCode) + "；该故障按确认的代码表需重启清除");
                if (!FaultCodes.CanClearWithCommand(fb.FaultCode))
                    return (false, "代码表未定义此故障的清除方式，请确认：" + FaultCodes.Describe(fb.FaultCode));
                if ((fb.FaultCode & 0x0004) != 0 && fb.IsFirmware117 && fb.TemperatureRaw >= 850)
                    return (false, "过温故障须恢复到85°C以下后再清除");
            }
            _enabledSince[id] = 0;
            bool sent = _keepalive.SendDisabled(id, clearError: true);
            string control = _registry.Motors.First(m => m.DeviceId == id).UseFirmware117Pdo ? "20" : "08";
            return (sent, sent ? $"错误复位帧已发送（B0={control}），请检查后续反馈" : "清错发送失败");
        }
    }
    public (bool Ok, string Message) MoveToPosition(int id, double targetDeg, double accel, double speed)
    {
        lock (_gate)
        {
            if (Check(id, motion: true) is { } error) return (false, error);
            if (_keepalive.IsMoving(id)) return (false, "正在运动，请等待或先停止");
            try
            {
                if (!_keepalive.Move(id, Od.AngleToRaw(targetDeg), speed, accel)) return (false, "未使能或正在运动");
            }
            catch (ArgumentOutOfRangeException ex) { return (false, ex.Message); }
            Log?.Invoke($"电机 {id} 定位 → {targetDeg:F3}°，输出端速度 {speed:F2}rpm，加速度 {accel:F2}rpm/s");
            return (true, "位置轨迹已启动");
        }
    }
    public (bool Ok, string Message) ReturnAllToZero(double accel, double speed)
    {
        lock (_gate)
        {
            int[] configured = _registry.Motors.Select(m => m.DeviceId).ToArray();
            int[] ids = configured.Where(id => _keepalive.IsArmed(id) && _transport.GetFeedback(id)?.Enabled != false).ToArray();
            int[] skipped = configured.Except(ids).ToArray();
            string skippedMessage = skipped.Length == 0 ? "" : $"；跳过未使能关节 ID {string.Join(", ", skipped)}";
            if (ids.Length == 0) return (false, "没有通过本程序使能的关节，未发送回零指令" + skippedMessage);
            foreach (int id in ids)
            {
                if (Check(id, motion: true) is { } error) return (false, $"ID {id}：{error}；未启动全部回零");
                if (_keepalive.IsMoving(id)) return (false, $"ID {id} 正在运动，请先停止或等待完成；未启动全部回零");
            }
            try
            {
                if (!_keepalive.MoveGroup(ids, 0, speed, accel))
                    return (false, "关节状态已变化，请检查使能及运动状态；未启动全部回零");
            }
            catch (ArgumentOutOfRangeException ex) { return (false, ex.Message); }
            string message = $"关节 ID {string.Join(", ", ids)} 的 0° 位置轨迹已启动，速度 {speed:F2}rpm，加速度 {accel:F2}rpm/s" + skippedMessage;
            Log?.Invoke(message);
            return (true, message);
        }
    }
    public (bool Ok, string Message) MoveIncrement(int id, double deltaDeg, double accel, double speed)
    {
        lock (_gate)
        {
            if (Check(id, motion: true) is { } error) return (false, error);
            if (!double.IsFinite(deltaDeg) || Math.Abs(deltaDeg) < 0.005) return (false, "增量过小或无效");
            if (_keepalive.IsMoving(id)) return (false, "正在运动，请等待或先停止");
            var fb = _transport.GetFeedback(id)!;
            var previous = _keepalive.GetTarget(id);
            if (previous.HasValue && !IsAtTarget(id))
                return (false, "电机尚未到达上一目标");
            return MoveToPosition(id, fb.PositionDeg + deltaDeg, accel, speed);
        }
    }
    public (bool Ok, string Message) StopMotion(int id)
    {
        lock (_gate)
        {
            if (!Configured(id)) return (false, "节点未配置");
            _keepalive.Hold(id);
            Log?.Invoke($"电机 {id} 已停止轨迹，保持最后指令位置");
            return (true, "轨迹停止，保持位置");
        }
    }
    public bool IsAtTarget(int id)
    {
        var fb = _transport.GetFeedback(id);
        var target = _keepalive.GetTarget(id);
        return _transport.IsDeviceAlive(id) && target.HasValue && fb != null &&
            !_keepalive.IsMoving(id) && fb.TargetReached != false && Math.Abs(target.Value - fb.PositionRaw) <= 20 && Math.Abs(fb.VelocityRpm) <= 2;
    }
    public async Task<(bool Ok, string Message)> SetZeroOriginAsync(int id, double accel, double speed)
    {
        int generation;
        lock (_gate)
        {
            if (Check(id, motion: true) is { } error) return (false, error);
            if (_keepalive.IsMoving(id) || !IsAtTarget(id)) return (false, "请先停止并等待电机到位后标零");
            _calibrating[id] = true;
            generation = _generation;
            _keepalive.Disarm(id); // 坐标系改变后不再发送旧位置
        }
        try
        {
            Task<SdoResponse?> write;
            lock (_gate)
            {
                if (generation != _generation) return (false, "操作已取消");
                write = _transport.SdoRequestAsync(id, SdoMessage.WriteU32(id, Od.ZeroSet, 0, 1));
            }
            var rsp = await write.ConfigureAwait(false);
            lock (_gate)
            {
                if (generation != _generation) return (false, "操作已取消");
                if (rsp?.IsWriteAck != true) return (false, "标零未确认（超时或SDO拒绝）");
                if (!_keepalive.SendDisabled(id)) return (false, "标零后失能发送失败");
                // 保留既有写1→写0流程。确认协议第8页复位值仍写1且使能状态说明矛盾，待厂家澄清。
                write = _transport.SdoRequestAsync(id, SdoMessage.WriteU32(id, Od.ZeroSet, 0, 0));
            }
            rsp = await write.ConfigureAwait(false);
            long verificationStart = Environment.TickCount64;
            lock (_gate)
            {
                if (generation != _generation) return (false, "操作已取消");
                if (rsp?.IsWriteAck != true) return (false, "标零复位未确认");
                _transport.Send(PdoBuilder.Sync());
            }
            await Task.Delay(150).ConfigureAwait(false);
            lock (_gate)
            {
                if (generation != _generation) return (false, "操作已取消");
                var fb = _transport.GetFeedback(id);
                if (fb == null || fb.Timestamp < verificationStart || Math.Abs(fb.PositionRaw) > 2)
                    return (false, "未收到标零后的有效零位反馈，请检查电机");
                return (true, "标零及复位完成，电机保持失能；请手动使能");
            }
        }
        finally
        {
            lock (_gate)
            {
                _keepalive.SendDisabled(id);
                _enabledSince[id] = 0;
                _calibrating[id] = false;
            }
        }
    }
    public (bool Ok, string Message) EmergencyStop()
    {
        lock (_gate)
        {
            ++_generation;
            _keepalive.DisarmAll();
            bool ok = true;
            foreach (var m in _registry.Motors)
            {
                _enabledSince[m.DeviceId] = 0;
                ok &= _keepalive.SendDisabled(m.DeviceId);
            }
            string message = ok ? "四个已配置电机的失能抱闸帧已发送" : "部分失能帧发送失败，请检查硬件急停和CAN连接";
            Log?.Invoke(message);
            return (ok, message);
        }
    }
    public async Task<(bool Ok, string Message)> ChangeDeviceIdAsync(int currentId, int newId)
    {
        int generation;
        lock (_gate)
        {
            if (!Configured(currentId) || newId is < 1 or > 127 || newId == currentId)
                return (false, "仅可修改已配置节点，新ID须为1~127并与原ID不同");
            if (_registry.Motors.Any(m => m.DeviceId == newId)) return (false, "新ID已被占用");
            if (_calibrating[currentId] || !_transport.IsDeviceAlive(currentId) ||
                _keepalive.IsArmed(currentId) || _transport.GetFeedback(currentId)?.Enabled != false)
                return (false, "请确认电机在线且已失能");
            _calibrating[currentId] = true;
            generation = _generation;
        }
        try
        {
            // 探测包括未纳入控制的另外两台电机；无响应仍需现场确认ID未被占用。
            var probe = await _transport.SdoRequestAsync(newId, SdoMessage.Read(newId, Od.ActualPosition, 0), 100).ConfigureAwait(false);
            Task<SdoResponse?> write;
            lock (_gate)
            {
                if (generation != _generation) return (false, "操作已取消");
                if (probe != null) return (false, "新ID在总线上已有响应，禁止写入");
                write = _transport.SdoRequestAsync(currentId, SdoMessage.WriteU32(currentId, Od.CanNodeId, 0, newId));
            }
            var rsp = await write.ConfigureAwait(false);
            return (rsp?.IsWriteAck == true, rsp?.IsWriteAck == true ?
                "ID已写入，需电机断电重启；重启后再更新映射" : "改ID未确认（超时或SDO拒绝）");
        }
        finally { lock (_gate) _calibrating[currentId] = false; }
    }
    public bool SendSync() => _transport.Send(PdoBuilder.Sync());
}
