using System.Text.Json;
using System.Text.Json.Serialization;
using WaistModel;

namespace waist_motor_controller.WholeBody;

internal sealed record WholeBodyPoint(Guid Id, string Name, double HeightMm, double PitchDeg, double YawDeg,
    double ForeAftMm, double[] AxisAnglesDeg)
{
    [JsonIgnore] public TaskPose Pose => new(ForeAftMm / 1000, HeightMm / 1000, PitchDeg * Math.PI / 180, YawDeg * Math.PI / 180);

    [JsonIgnore] public bool IsZeroPose => AxisAnglesDeg.All(angle => angle == 0) &&
        DiskKinematics.Matches(Pose, new double[4]);
}

internal sealed class WholeBodyPointStore(string filePath)
{
    public WholeBodyPoint[] Load()
    {
        try
        {
            using var file = new FileStream(filePath, FileMode.Open, FileAccess.Read, FileShare.Read | FileShare.Delete);
            var points = JsonSerializer.Deserialize<WholeBodyPoint[]>(file, CalibrationStore.Options)
                ?? throw new InvalidDataException("点位文件为空");
            if (points.Any(p => p is null || !IsValid(p)) || points.Select(p => p.Id).Distinct().Count() != points.Length ||
                points.Select(p => p.Name).Distinct(StringComparer.OrdinalIgnoreCase).Count() != points.Length)
                throw new InvalidDataException("点位文件含无效或重复记录");
            return points;
        }
        catch (FileNotFoundException) { return []; }
        catch (DirectoryNotFoundException) { return []; }
        catch (JsonException e) { throw new InvalidDataException("点位文件格式无效", e); }
    }

    public WholeBodyPoint[] Save(WholeBodyPoint point, bool editing) => Update(points =>
    {
        if (!IsValid(point)) throw new ArgumentException("点位数据无效，请填写名称及有效位姿");
        int index = points.FindIndex(p => p.Id == point.Id);
        if (editing != (index >= 0)) throw new InvalidOperationException("点位列表已变化，请重新选择");
        if (points.Any(p => p.Id != point.Id && StringComparer.OrdinalIgnoreCase.Equals(p.Name, point.Name)))
            throw new ArgumentException("已有同名点位，请换一个名称");
        if (editing) points[index] = point;
        else points.Add(point);
    });

    public WholeBodyPoint[] Delete(Guid id) => Update(points =>
    {
        if (points.RemoveAll(point => point.Id == id) == 0)
            throw new InvalidOperationException("点位已删除，请重新选择");
    });

    public WholeBodyPoint[] Move(Guid id, Guid anchorId, bool after) => Update(points =>
    {
        var point = points.Find(p => p.Id == id);
        if (point is null || !points.Any(p => p.Id == anchorId))
            throw new InvalidOperationException("点位列表已变化，请重新拖动");
        if (id == anchorId) return;
        points.Remove(point);
        int destination = points.FindIndex(p => p.Id == anchorId);
        points.Insert(after ? destination + 1 : destination, point);
    });

    private static bool IsValid(WholeBodyPoint point) =>
        point.Id != Guid.Empty && !string.IsNullOrWhiteSpace(point.Name) && point.Name.Length <= 40 &&
        double.IsFinite(point.HeightMm) && double.IsFinite(point.ForeAftMm) &&
        double.IsFinite(point.PitchDeg) && double.IsFinite(point.YawDeg) &&
        point.AxisAnglesDeg is { Length: 4 } && point.AxisAnglesDeg.All(double.IsFinite);

    private WholeBodyPoint[] Update(Action<List<WholeBodyPoint>> edit)
    {
        using var writer = new Mutex(false, @"Local\WaistMotorController.Waypoints");
        try
        {
            if (!writer.WaitOne(TimeSpan.FromSeconds(1)))
                throw new IOException("另一个窗口正在保存点位，请稍后重试");
        }
        catch (AbandonedMutexException) { } // The prior writer exited. The JSON is replaced atomically.

        try
        {
            var points = Load().ToList();
            edit(points);
            Directory.CreateDirectory(Path.GetDirectoryName(filePath)!);
            string temporary = filePath + "." + Guid.NewGuid().ToString("N") + ".tmp";
            try
            {
                File.WriteAllText(temporary, JsonSerializer.Serialize(points, CalibrationStore.Options));
                File.Move(temporary, filePath, overwrite: true);
            }
            finally { File.Delete(temporary); }
            return points.ToArray();
        }
        finally { writer.ReleaseMutex(); }
    }
}
