using waist_motor_controller.Protocol;

namespace waist_motor_controller.Control;

/// <summary>主机端梯形速度规划。每帧变化严格小于协议的 100cnt 跃迁阈值。</summary>
public sealed class PositionTrajectory
{
    public const int MaxStepRaw = 90;
    private double _position, _velocity, _maxVelocity, _acceleration;
    public short CommandRaw { get; private set; }
    public short TargetRaw { get; private set; }
    public bool IsMoving => CommandRaw != TargetRaw;

    public PositionTrajectory(short position) => Hold(position);
    public void Hold(short position)
    {
        _position = CommandRaw = TargetRaw = position;
        _velocity = 0;
    }

    public void Move(short target, double outputRpm, double outputRpmPerSec)
    {
        if (!double.IsFinite(outputRpm) || outputRpm <= 0 || outputRpm > 5 ||
            !double.IsFinite(outputRpmPerSec) || outputRpmPerSec <= 0 || outputRpmPerSec > 50)
            throw new ArgumentOutOfRangeException(nameof(outputRpm), "输出端速度 0~5rpm，加速度 0~50rpm/s（均不含0）");
        if (target < Od.MinPositionRaw || target > Od.MaxPositionRaw)
            throw new ArgumentOutOfRangeException(nameof(target));
        TargetRaw = target;
        _velocity = 0;
        _maxVelocity = outputRpm * 65536 / 60;
        _acceleration = outputRpmPerSec * 65536 / 60;
    }

    public short Step(double seconds)
    {
        // 调度延迟时减慢轨迹，不追赶丢失的时间，避免一次跳跃过大。
        double dt = Math.Clamp(seconds, 0, 0.005);
        double remaining = TargetRaw - _position;
        if (Math.Abs(remaining) < 0.00001) return CommandRaw;
        double speed = Math.Min(_maxVelocity, Math.Sqrt(2 * _acceleration * Math.Abs(remaining)));
        _velocity = Math.Min(speed, _velocity + _acceleration * dt);
        double step = Math.Min(Math.Abs(remaining), Math.Min(MaxStepRaw, _velocity * dt));
        _position += Math.Sign(remaining) * step;
        CommandRaw = checked((short)Math.Round(_position));
        return CommandRaw;
    }
}
