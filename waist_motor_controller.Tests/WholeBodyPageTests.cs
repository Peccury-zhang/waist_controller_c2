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
        var pitchLock = (CheckBox)type.GetField("keepPitch", flags)!.GetValue(page)!;
        Check(speedInput.Value == 8 && accelerationInput.Value == 60, "motion defaults are 8 rpm and 60 rpm/s");
        Check(!pitchLock.Checked, "pitch hold starts unchecked");
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
        Check(motion.All(button => button.Enabled) && inputs[2].Enabled, "valid feedback enables motion and pitch controls");
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
        CheckTrackingLayout(page, calibration);
        CheckPointManagement();
        Console.WriteLine("PASS WBC page: feedback, jog, reconnect and saved points; simulated transport only.");
    }

    private static void CheckTrackingLayout(WholeBodyPage page, Calibration calibration)
    {
        using var form = new Form
        {
            StartPosition = FormStartPosition.Manual, Location = new Point(-5000, -5000),
            ShowInTaskbar = false, ClientSize = new Size(1280, 800),
            AutoScaleDimensions = new SizeF(96, 96), AutoScaleMode = AutoScaleMode.Dpi,
        };
        form.Controls.Add(page);
        try
        {
            form.Show();
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
            var controller = (WholeBodyController)typeof(WholeBodyPage).GetField("controller", flags)!.GetValue(page)!;
            var robot = (Robot)typeof(WholeBodyPage).GetField("model", flags)!.GetValue(page)!;
            var plan = MotionPlan.CreateSmoothToZero(robot, calibration, [0, -.4, .8, -.4], 8, 60);
            typeof(WholeBodyController).GetField("plan", flags)!.SetValue(controller, plan);
            typeof(WholeBodyController).GetField("moveLDeviation", flags)!.SetValue(controller, (0.98, 0d, 0.02));
            typeof(WholeBodyPage).GetMethod("RefreshScene", flags)!.Invoke(page, null);
            Application.DoEvents();
            var status = (Label)typeof(WholeBodyPage).GetField("status", flags)!.GetValue(page)!;
            var tracking = (Label)typeof(WholeBodyPage).GetField("trackingStatus", flags)!.GetValue(page)!;
            if (!tracking.Text.Contains("位置跟踪差") || status.Text.Contains("位置跟踪差") ||
                tracking.Bounds.Top < status.Bounds.Bottom || tracking.Bottom > tracking.Parent!.ClientSize.Height ||
                tracking.Height < tracking.PreferredHeight || tracking.Width < tracking.PreferredWidth)
                throw new Exception($"WBC page: tracking text does not fit. Status={status.Bounds}, " +
                    $"tracking={tracking.Bounds}, preferred={tracking.PreferredSize}, text={tracking.Text}");
        }
        finally { form.Controls.Remove(page); }
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
            CheckScrollingAndReorder(store, calibration, point);
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

    private static void CheckScrollingAndReorder(WholeBodyPointStore store, Calibration calibration, WholeBodyPoint template)
    {
        var points = Enumerable.Range(1, 8)
            .Select(i => template with { Id = Guid.NewGuid(), Name = $"点位 {i}" }).ToArray();
        points[0] = points[0] with { Name = "最高站直点", HeightMm = 971.999999994603, ForeAftMm = 65.9999,
            PitchDeg = 0, YawDeg = 0, AxisAnglesDeg = new double[4] };
        if (!points[0].IsZeroPose || (points[0] with { YawDeg = 1 }).IsZeroPose || points[1].IsZeroPose)
            throw new Exception("WBC points: only the saved zero pose can use the shared zero trajectory");
        foreach (var point in points) store.Save(point, editing: false);
        using var form = new Form
        {
            StartPosition = FormStartPosition.Manual, Location = new Point(-5000, -5000),
            ShowInTaskbar = false, ClientSize = new Size(540, 350),
        };
        var outer = new Panel { Dock = DockStyle.Fill, AutoScroll = true };
        form.Controls.Add(outer);
        WholeBodyPoint? executed = null;
        WholeBodyPointsPanel panel = null!;
        panel = new WholeBodyPointsPanel(store, () => calibration, () => (template.Pose, ""), point =>
        {
            executed = point;
            panel.SetAvailability(false, false, false);
        }) { Bounds = new Rectangle(0, 180, 490, 305) };
        outer.Controls.Add(panel);
        outer.Controls.Add(new Button { Text = "外部按钮", Location = new Point(0, 500) });
        form.Show();
        panel.SetAvailability(true, true, true);
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
        var viewport = (Panel)typeof(WholeBodyPointsPanel).GetField("scroll", flags)!.GetValue(panel)!;
        var list = (TableLayoutPanel)typeof(WholeBodyPointsPanel).GetField("list", flags)!.GetValue(panel)!;
        var first = list.GetControlFromPosition(0, 0)!;
        if (first.Cursor != Cursors.SizeNS ||
            !first.Controls[0].Controls.OfType<Label>().Any(label => label.Text == points[0].Name))
            throw new Exception("WBC points: the card is draggable and its label has no drag icon");
        var run = first.Controls[0].Controls.OfType<Button>().Single(button => button.Text == "执行");
        if (!run.Focus()) throw new Exception("WBC points: could not focus the first execute button");
        var innerBefore = viewport.AutoScrollPosition;
        var outerBefore = outer.AutoScrollPosition;
        run.PerformClick();
        if (executed?.Id != points[0].Id || executed.Pose != points[0].Pose || !viewport.Focused ||
            viewport.AutoScrollPosition != innerBefore || outer.AutoScrollPosition != outerBefore)
            throw new Exception("WBC points: executing the first point changed the visible rows");
        panel.SetAvailability(true, true, true);

        var moved = points[1];
        typeof(WholeBodyPointsPanel).GetField("dragging", flags)!.SetValue(panel, moved.Id);
        var beforeFirst = first.PointToScreen(new Point(first.Width / 2, 4));
        var drop = new DragEventArgs(new DataObject(moved.Id.ToString()), 0, beforeFirst.X, beforeFirst.Y,
            DragDropEffects.Move, DragDropEffects.None);
        typeof(Control).GetMethod("OnDragDrop", flags)!.Invoke(viewport, [drop]);
        typeof(WholeBodyPointsPanel).GetField("dragging", flags)!.SetValue(panel, null);
        Application.DoEvents();
        var order = new[] { moved.Id, points[0].Id }.Concat(points.Skip(2).Select(p => p.Id));
        var notice = (Label)typeof(WholeBodyPointsPanel).GetField("notice", flags)!.GetValue(panel)!;
        if (drop.Effect != DragDropEffects.Move || !store.Load().Select(p => p.Id).SequenceEqual(order) ||
            notice.Text.Length != 0 ||
            !list.GetControlFromPosition(0, 0)!.Controls[0].Controls.OfType<Label>().Any(label => label.Text == moved.Name))
            throw new Exception("WBC points: dragging before the first row did not update the view and persisted order");
        store.Move(moved.Id, points[^1].Id, after: true);
        if (!store.Load().Select(p => p.Id).SequenceEqual(points.Where(p => p.Id != moved.Id).Select(p => p.Id).Append(moved.Id)))
            throw new Exception("WBC points: moving after the last row did not preserve the other points");
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
