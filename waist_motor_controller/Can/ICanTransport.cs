using waist_motor_controller.Protocol;
namespace waist_motor_controller.Can;
public interface ICanTransport
{
    bool IsOpen { get; }
    bool Send(CanFrame frame);
    int SendBatch(IReadOnlyList<CanFrame> frames)
    {
        int sent = 0; foreach (var frame in frames) { if (!Send(frame)) break; sent++; } return sent;
    }
    MotorFeedback? GetFeedback(int deviceId);
    bool IsDeviceAlive(int deviceId, int windowMs = 500);
    event Action<int, MotorFeedback>? FeedbackUpdated;
    Task<SdoResponse?> SdoRequestAsync(int deviceId, CanFrame request, int timeoutMs = 300);
}
