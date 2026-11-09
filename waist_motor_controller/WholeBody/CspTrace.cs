using System.Text;
using WaistModel;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.WholeBody;

internal sealed class CspTrace
{
    private const int MaxDurationMs = 120_000;
    private readonly Sample[] samples = new Sample[24_000];
    private long startedAt;
    private long endedAt;
    private string? endReason;
    internal int Count { get; private set; }
    internal bool Complete => endReason is not null;

    // Controller snapshots are replaced, never mutated after publication.
    internal readonly record struct Sample(
        long TimeMs, long TargetSentMs, GroupState State, double? TrajectorySeconds, double CycleSeconds,
        short[] Target, short[] PreviousTarget, double TargetInterval, MotorFeedback[] Feedback, int[] Signs);

    internal void Start(long now) => startedAt = now;

    internal void CheckDuration(long now)
    {
        if (now - startedAt >= MaxDurationMs) Finish(now, "recording_limit");
    }

    internal void Add(Sample sample)
    {
        if (Complete) return;
        samples[Count++] = sample;
        if (Count == samples.Length) Finish(sample.TimeMs, "sample_limit");
    }

    internal void Finish(long now, string reason)
    {
        if (Complete) return;
        endedAt = now;
        endReason = reason;
    }

    internal string Save(string directory)
    {
        Directory.CreateDirectory(directory);
        string path = Path.Combine(directory, $"csp-{DateTime.Now:yyyyMMdd-HHmmss-fff}-{Environment.ProcessId}.csv");
        string pending = path + ".tmp";
        try
        {
            using (var file = new StreamWriter(pending, append: false, new UTF8Encoding(true)))
            {
                file.WriteLine("time_ms,state,trajectory_s,cycle_ms,axis,can_id,target_sent_ms,feedback_received_ms,feedback_age_ms,target_raw,actual_raw,target_deg,actual_deg,error_deg,command_output_rpm,feedback_motor_rpm,current_ma,drive_mode,enabled,brake_released,fault_code,event");
                for (int n = 0; n < Count; n++)
                {
                    var s = samples[n];
                    for (int i = 0; i < s.Feedback.Length; i++)
                    {
                        var feedback = s.Feedback[i];
                        double Joint(int raw) => Calibration.FeedbackToUrdf(i, raw, s.Signs[i]);
                        double commanded = Joint(s.Target[i]) * 180 / Math.PI;
                        double actual = Joint(feedback.PositionRaw) * 180 / Math.PI;
                        double rpm = s.TargetInterval > 0
                            ? Joint(s.Target[i] - s.PreviousTarget[i]) * 60 / (2 * Math.PI * s.TargetInterval)
                            : 0;
                        file.WriteLine(FormattableString.Invariant(
                            $"{s.TimeMs - startedAt},{s.State},{s.TrajectorySeconds:F6},{s.CycleSeconds * 1000:F3},{i + 1},{feedback.DeviceId},{s.TargetSentMs - startedAt},{feedback.Timestamp - startedAt},{s.TimeMs - feedback.Timestamp},{s.Target[i]},{feedback.PositionRaw},{commanded:F6},{actual:F6},{actual - commanded:F6},{rpm:F6},{feedback.VelocityRpm},{feedback.CurrentMa},{feedback.Mode},{Convert.ToInt32(feedback.Enabled)},{Convert.ToInt32(feedback.BrakeReleased)},0x{feedback.FaultCode:X4},"));
                    }
                }
                var footer = new string[22];
                footer[0] = (endedAt - startedAt).ToString(System.Globalization.CultureInfo.InvariantCulture);
                footer[^1] = "\"" + endReason?.Replace("\"", "\"\"") + "\"";
                file.WriteLine(string.Join(",", footer));
            }
            File.Move(pending, path);
            return path;
        }
        finally
        {
            if (File.Exists(pending)) File.Delete(pending);
        }
    }
}
