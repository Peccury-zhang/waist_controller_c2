using System.Diagnostics;
using System.Runtime.InteropServices;
using waist_motor_controller.Can;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.Control;

/// <summary>独立线程发送同步位置指令；同一锁内发送与撤销，防急停后旧帧重发。</summary>
public sealed class KeepaliveScheduler
{
    private readonly object _gate = new();
    private readonly PositionTrajectory?[] _axes = new PositionTrajectory?[128];
    private readonly Func<CanFrame, bool> _send;
    private readonly Func<int, bool> _safe;
    private readonly Func<int, bool> _firmware117;
    private readonly ManualResetEvent _stop = new(false);
    private Thread? _thread;
    private long _sentCount;
    public long SentCount => Interlocked.Read(ref _sentCount);
    public event Action<int, string>? AxisStopped;
    public KeepaliveScheduler(Func<CanFrame, bool> send, Func<int, bool>? safe = null, Func<int, bool>? firmware117 = null)
    { _send = send; _safe = safe ?? (_ => true); _firmware117 = firmware117 ?? (_ => false); }
    private CanFrame Build(int id, PdoCommand cmd) => PdoBuilder.SingleAxis(id, cmd, _firmware117(id));

    public void Start()
    {
        lock (_gate)
        {
            if (_thread != null) return;
            _stop.Reset();
            _thread = new Thread(Run) { IsBackground = true, Name = "JuxiePosition", Priority = ThreadPriority.AboveNormal };
            _thread.Start();
        }
    }
    public void Stop()
    {
        Thread? thread;
        lock (_gate) { _stop.Set(); thread = _thread; _thread = null; Array.Clear(_axes); }
        thread?.Join();
    }
    public bool ArmHold(int id, short position)
    {
        lock (_gate)
        {
            if (!_send(Build(id, PdoCommand.Hold(position)))) return false;
            _axes[id] = new PositionTrajectory(position);
            return true;
        }
    }
    public bool Move(int id, short target, double speed, double accel)
    {
        lock (_gate)
        {
            if (_axes[id] is not { } axis || axis.IsMoving) return false;
            axis.Move(target, speed, accel);
            return true;
        }
    }
    public void Hold(int id) { lock (_gate) if (_axes[id] is { } axis) axis.Hold(axis.CommandRaw); }
    public bool MoveGroup(IReadOnlyList<int> ids, short target, double speed, double accel)
    {
        lock (_gate)
        {
            // Validate everything before changing a trajectory; Tick cannot observe a partly started group.
            new PositionTrajectory(0).Move(target, speed, accel);
            if (ids.Count == 0 || ids.Distinct().Count() != ids.Count ||
                ids.Any(id => id is < 1 or > 127 || _axes[id] == null || _axes[id]!.IsMoving)) return false;
            foreach (int id in ids) _axes[id]!.Move(target, speed, accel);
            return true;
        }
    }
    public bool IsMoving(int id) { lock (_gate) return _axes[id]?.IsMoving ?? false; }
    public bool IsArmed(int id) { lock (_gate) return _axes[id] != null; }
    public short? GetTarget(int id) { lock (_gate) return _axes[id]?.TargetRaw; }
    public void Disarm(int id) { lock (_gate) _axes[id] = null; }
    public void DisarmAll() { lock (_gate) Array.Clear(_axes); }
    public bool SendDisabled(int id, bool clearError = false)
    {
        lock (_gate)
        {
            _axes[id] = null;
            return _send(Build(id, new PdoCommand(false, false, clearError, 0, 0)));
        }
    }

    // 发送失败立即撤销，不能继续推进未被电机接收的轨迹。
    public void Tick(double seconds)
    {
        var stopped = new List<(int Id, string Reason)>();
        lock (_gate)
        {
            for (int id = 1; id < _axes.Length; id++)
            {
                if (_axes[id] is not { } axis) continue;
                if (!_safe(id))
                {
                    _axes[id] = null;
                    _send(Build(id, PdoCommand.Disable));
                    stopped.Add((id, "反馈超时或故障，已撤销轨迹并发送失能"));
                    continue;
                }
                if (!_send(Build(id, PdoCommand.Hold(axis.Step(seconds)))))
                {
                    _axes[id] = null;
                    _send(Build(id, PdoCommand.Disable));
                    stopped.Add((id, "发送失败，已撤销轨迹并尝试失能"));
                }
                else Interlocked.Increment(ref _sentCount);
            }
        }
        foreach (var item in stopped) AxisStopped?.Invoke(item.Id, item.Reason);
    }
    private void Run()
    {
        bool resolution = OperatingSystem.IsWindows() && timeBeginPeriod(1) == 0;
        try
        {
            long previous = Stopwatch.GetTimestamp();
            while (!_stop.WaitOne(1))
            {
                long now = Stopwatch.GetTimestamp();
                Tick((now - previous) / (double)Stopwatch.Frequency);
                previous = now;
            }
        }
        finally { if (resolution) timeEndPeriod(1); }
    }
    [DllImport("winmm.dll")] private static extern uint timeBeginPeriod(uint period);
    [DllImport("winmm.dll")] private static extern uint timeEndPeriod(uint period);
}
