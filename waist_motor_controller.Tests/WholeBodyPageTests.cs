using System.Reflection;
using WaistModel;
using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;
using waist_motor_controller.WholeBody;

internal static class WholeBodyPageTests
{
    public static void Run()
    {
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
        var bus = new PageBus();
        var registry = new MotorRegistry(Path.Combine(Path.GetTempPath(), Guid.NewGuid() + ".json"));
        var legacy = new MotorController(bus, new KeepaliveScheduler(bus.Send), registry);
        using var page = new WholeBodyPage(bus, registry, legacy, () => new Dictionary<int, string>(), _ => { });
        var type = typeof(WholeBodyPage);
        var inputs = (NumericUpDown[])type.GetField("target", flags)!.GetValue(page)!;
        var view = type.GetField("view", flags)!.GetValue(page)!;
        var jointOverlay = view.GetType().GetField("jointFeedback", flags)!.GetValue(view)!;
        string JointText() => (string)jointOverlay.GetType().GetProperty("Text")!.GetValue(jointOverlay)!;
        void Set(string name, object value) => type.GetField(name, flags)!.SetValue(page, value);
        object? Call(string name, params object[] args) => type.GetMethod(name, flags)!.Invoke(page, args);
        void Check(bool condition, string message) { if (!condition) throw new Exception("WBC page: " + message); }
        Check(WholeBodyPage.StateLabel(GroupState.Holding, false, true) == "保持 / 提前停止" &&
            WholeBodyPage.StateLabel(GroupState.Holding, true, false) == "保持 / 已到位" &&
            WholeBodyPage.StateLabel(GroupState.Holding, false, false) == "保持",
            "holding distinguishes an early stop, arrival, and initial CSP hold");
        var speedInput = (NumericUpDown)type.GetField("maxSpeed", flags)!.GetValue(page)!;
        var accelerationInput = (NumericUpDown)type.GetField("maxAcceleration", flags)!.GetValue(page)!;
        Check(speedInput.Value == 5 && accelerationInput.Value == 30, "motion defaults are 5 rpm and 30 rpm/s");
        speedInput.Value = 11;
        accelerationInput.Value = 62;
        Check(speedInput.Maximum == 11 && accelerationInput.Maximum == 62 &&
            speedInput.Value == 11 && accelerationInput.Value == 62, "independent speed and acceleration input limits");
        void Refresh() => Call("RefreshScene");
        decimal[] Values() => inputs.Select(n => n.Value).ToArray();
        void Edit()
        {
            Set("fillingTargets", true);
            decimal[] values = [750, 32, 1.5m, -5];
            for (int i = 0; i < 4; i++) inputs[i].Value = values[i];
            Set("fillingTargets", false);
            Set("editedTargetAxes", 3);
        }

        bus.Valid = false;
        Refresh();
        Check(inputs.All(n => !n.Enabled), "wait for feedback before editing");
        Check(inputs[0].Minimum >= 466 && JointText().Count(ch => ch == '—') == 4,
            "height uses the operating floor and the overlay shows missing feedback");
        var motion = (List<Button>)type.GetField("motion", flags)!.GetValue(page)!;
        int enabledChanges = 0;
        foreach (var control in motion.Cast<Control>().Concat(inputs))
            control.EnabledChanged += (_, _) => enabledChanges++;
        Refresh();
        Check(enabledChanges == 0, "a refresh with stale feedback never briefly enables the controls");
        bus.Valid = true;
        Refresh();
        Check(motion.All(button => button.Enabled), "valid feedback enables motion controls");
        string ExpectedAngles() => string.Join(Environment.NewLine, bus.Raw.Select((raw, i) =>
            $"M{i + 1} {registry.Motors[i].Direction * Od.RawToAngle(raw),7:F2}°"));
        Check(JointText() == ExpectedAngles(), "the 3D overlay uses the same signs as the joint controls");
        bus.Raw[0] = -100;
        Refresh();
        Check(JointText() == ExpectedAngles(), "updated feedback changes the 3D overlay");
        bus.Raw[0] = 100;
        Refresh();
        var expected = Values();
        var pose = DiskKinematics.Forward(bus.Raw.Select((raw, i) =>
            Calibration.FeedbackToUrdf(i, raw, registry.Motors[i].Direction)).ToArray());
        double[] actual = [pose.Z * 1000, pose.X * 1000, pose.Pitch * 180 / Math.PI, pose.Yaw * 180 / Math.PI];
        Check(expected.Select((v, i) => Math.Abs((double)v - actual[i]) < .006).All(x => x), "first valid feedback fills all four targets");
        Edit();
        Refresh();
        Check(inputs[0].Value == 750, "refresh preserves manual target");
        page.Visible = false;
        page.Visible = true;
        Check(Values().SequenceEqual(expected), "reopening resets targets to actual pose");

        for (int axis = 0; axis < 4; axis++)
        foreach (int direction in new[] { -1, 1 })
        {
            Edit();
            var before = Values();
            var request = (TaskPose)Call("JogRequest", axis, direction)!;
            before[axis] += direction * (axis < 2 ? 10 : 2);
            var updated = Values();
            Check(updated.Where((_, i) => axis != 0 || i != 1)
                .SequenceEqual(before.Where((_, i) => axis != 0 || i != 1)),
                "jog updates the selected input and only a height jog can reset manual X");
            double[] requested = [request.Z * 1000, request.X * 1000, request.Pitch * 180 / Math.PI, request.Yaw * 180 / Math.PI];
            Check(updated.Select((v, i) => Math.Abs((double)v - requested[i]) < 1e-8).All(x => x), "planner receives displayed values");
        }
        inputs[2].Value = inputs[3].Value = 0;
        inputs[1].Value = 55;
        inputs[0].Value = 921;
        Check(inputs[1].Value == 0, "height edit overrides previously manual X and uses the reachable center");
        inputs[1].Value = 60;
        Check(inputs[0].Value == 921, "manual X does not change height");
        inputs[0].Value = 466;
        var calibration = page.Store.Load(registry.Motors.Select(m => m.Direction).ToArray());
        var centered = MotionPlan.ResolveCenteredEndpoint(calibration, new TaskPose(0, .466, 0, 0)).Pose;
        Check(inputs[0].Value == 466 && Math.Abs((double)inputs[1].Value - centered.X * 1000) < .0001,
            "a deep fold uses the feasible X closest to center after a manual X edit");

        Edit();
        inputs[0].Value = inputs[0].Maximum;
        var boundary = Values();
        bool rejected = false;
        try { Call("JogRequest", 0, 1); }
        catch (TargetInvocationException e) when (e.InnerException is InvalidOperationException) { rejected = true; }
        Check(rejected && Values().SequenceEqual(boundary), "out of range jog leaves inputs unchanged");
        inputs[2].Value = inputs[3].Value = 0;
        inputs[1].Value = 0;
        var unreachable = (Task)Call("RunAsync", -1, 0, false, false, null!)!;
        var deadline = Environment.TickCount64 + 3000;
        while (!unreachable.IsCompleted && Environment.TickCount64 < deadline)
        {
            Application.DoEvents();
            Thread.Sleep(1);
        }
        Check(unreachable.IsCompletedSuccessfully && inputs[0].Value == inputs[0].Maximum &&
            inputs[1].Value == 0 && bus.Sent == 0, "unreachable manual X at maximum height does not execute or change height");
        page.ConnectionClosed();
        Refresh();
        Check(Values().SequenceEqual(expected), "reconnection initializes actual pose again");
        Check(bus.Sent == 0, "UI checks never send motor commands");
        CheckPointManagement();
        Console.WriteLine("PASS WBC page: feedback, jog, reconnect and saved points; simulated transport only.");
    }

    private static void CheckPointManagement()
    {
        void Check(bool condition, string message) { if (!condition) throw new Exception("WBC points: " + message); }
        string directory = Path.Combine(Path.GetTempPath(), "wbc-points-" + Guid.NewGuid().ToString("N"));
        string path = Path.Combine(directory, "waypoints.json");
        var store = new WholeBodyPointStore(path);
        var calibration = new Calibration
        {
            Motors = [new(1, -172, 172), new(2, -100, 15), new(3, -10, 143), new(4, -54, 10)],
        };
        const double degrees = 180 / Math.PI;
        var low = MotionPlan.ResolveCenteredEndpoint(calibration, new TaskPose(0, .650, 0, 0));
        Check(low.Pose.X > 0 && low.TargetRaw.Length == 4 &&
            DiskKinematics.Matches(low.Pose, low.TargetRaw.Select((raw, i) => calibration.ToJoint(i, raw)).ToArray()),
            "an off-center feasible point saves four matching encoded joint angles");
        var request = new TaskPose(0, .900, 0, 3 / degrees);
        var resolved = MotionPlan.ResolveCenteredEndpoint(calibration, request);
        var point = new WholeBodyPoint(Guid.NewGuid(), "测试点", 900, 0, 3, resolved.Pose.X * 1000,
            resolved.TargetRaw.Select((raw, i) => calibration.Signs[i] * Od.RawToAngle(raw)).ToArray());
        string assets = Path.Combine(AppContext.BaseDirectory, "assets", "robot");
        var robot = Robot.Load(Path.Combine(assets, "urdf", "urdf_g3_06.urdf"), Path.Combine(assets, "meshes"));
        var plan = MotionPlan.CreateMoveL(robot, calibration, new double[4], point.Pose, 5, 30);
        Check(DiskKinematics.Matches(point.Pose, plan.End) &&
            calibration.Encode(robot, plan.End).SequenceEqual(resolved.TargetRaw),
            "the saved Cartesian endpoint replays to the displayed four axes");
        try
        {
            Check(store.Load().Length == 0, "an absent file starts with an empty list");
            store.Save(point, editing: false);
            var restored = new WholeBodyPointStore(path).Load().Single();
            Check(restored.Id == point.Id && restored.Name == point.Name && restored.Pose == point.Pose &&
                restored.AxisAnglesDeg.SequenceEqual(point.AxisAnglesDeg), "all point fields survive restart");
            bool duplicate = false;
            try { store.Save(point with { Id = Guid.NewGuid() }, editing: false); }
            catch (ArgumentException) { duplicate = true; }
            Check(duplicate && store.Load().Length == 1, "duplicate names cannot overwrite an existing point");
            var edited = point with { Name = "编辑后" };
            store.Save(edited, editing: true);
            Check(store.Load().Single().Name == edited.Name, "edit updates the saved point by id");

            WholeBodyPoint? executed = null;
            using var panel = new WholeBodyPointsPanel(store, () => calibration, () => (point.Pose, ""),
                value => executed = value);
            static IEnumerable<Control> Descendants(Control root) =>
                root.Controls.Cast<Control>().SelectMany(child => Descendants(child).Prepend(child));
            var run = Descendants(panel).OfType<Button>().Single(b => b.Text == "执行");
            Check(!run.Enabled && Descendants(panel).OfType<Label>().Any(l => l.Text.Contains("M4")),
                "the row shows all four axes and disables execution without readiness");
            panel.SetAvailability(true, true, true);
            run.PerformClick();
            Check(executed?.Id == point.Id && executed.Pose == point.Pose, "execute uses the selected saved endpoint");

            store.Delete(point.Id);
            Check(store.Load().Length == 0, "delete persists an empty list");
            File.WriteAllText(path, "{");
            bool invalid = false;
            try { store.Save(point, editing: false); }
            catch (InvalidDataException) { invalid = true; }
            Check(invalid && File.ReadAllText(path) == "{", "an invalid point file is preserved on a failed save");
        }
        finally
        {
            File.Delete(path);
            if (Directory.Exists(directory)) Directory.Delete(directory);
        }
    }

    private sealed class PageBus : ICanTransport
    {
        public short[] Raw = [100, 3000, 6000, 3000];
        public bool Valid = true;
        public int Sent;
        public bool IsOpen => true;
        public event Action<int, MotorFeedback>? FeedbackUpdated { add { } remove { } }
        public bool Send(CanFrame frame) { Sent++; return true; }
        public MotorFeedback? GetFeedback(int id) => Valid ? new MotorFeedback
        {
            DeviceId = id, PositionRaw = Raw[id - 3], Timestamp = MotorFeedback.Now, IsFirmware117 = true,
        } : null;
        public bool IsDeviceAlive(int id, int windowMs = 500) => Valid;
        public Task<SdoResponse?> SdoRequestAsync(int id, CanFrame request, int timeoutMs = 300) => Task.FromResult<SdoResponse?>(null);
    }
}
