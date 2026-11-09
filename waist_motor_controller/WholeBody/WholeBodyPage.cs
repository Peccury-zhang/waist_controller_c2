using RobotVisualization;
using WaistModel;
using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;
using waist_motor_controller.UI;
namespace waist_motor_controller.WholeBody;
public sealed class WholeBodyPage:UserControl
{
    readonly Robot model;readonly RobotView view;readonly WholeBodyController controller;readonly ICanTransport bus;readonly MotorRegistry registry;
    readonly Func<IReadOnlyDictionary<int,string>> firmware;public CalibrationStore Store {get;}=new();
    readonly NumericUpDown[] target=new NumericUpDown[4];readonly Label[] actual=new Label[4];readonly Label hint=new(){AutoSize=true};
    private readonly Label status = new() { Dock = DockStyle.Fill, Margin = Padding.Empty, AutoEllipsis = true };
    private readonly Label trackingStatus = new() { Dock = DockStyle.Fill, Margin = Padding.Empty, AutoSize = true, ForeColor = UiTheme.Muted };
    readonly NumericUpDown positionStep = new() { Minimum = .1m, Maximum = 100, Value = 10, Increment = 5, DecimalPlaces = 1 };
    readonly NumericUpDown angleStep = new() { Minimum = .1m, Maximum = 30, Value = 2, Increment = 1, DecimalPlaces = 1 };
    readonly NumericUpDown maxSpeed = NewOutputLimit(MotionPlan.MaximumOutputSpeedRpm, 8);
    readonly NumericUpDown maxAcceleration = NewOutputLimit(MotionPlan.MaximumOutputAccelerationRpmPerSec, 60);
    readonly CheckBox keepPitch = new() { Text = "保持当前俯仰", AutoSize = true };
    readonly List<Button> motion=new();readonly System.Windows.Forms.Timer timer=new(){Interval=50};readonly CancellationTokenSource lifetime=new();CancellationTokenSource? operation;
    readonly Action<string> log;
    private readonly Button recordingButton;
    private readonly Button parkingRecoveryButton;
    private readonly WholeBodyPointsPanel points;
    private readonly Label recordingStatus = new() { Dock = DockStyle.Fill, AutoEllipsis = true, TextAlign = ContentAlignment.MiddleLeft };
    private CspTrace? recordingToSave;
    private bool savingRecording;
    private bool fillingTargets;
    private bool pendingActualTargets = true;
    private bool showingParkingStatus;
    private int editedTargetAxes;
    private Calibration? targetCalibration;
    private string? workspaceVersion;
    private Label heightTitle = null!;
    private Label frontTitle = null!;
    bool planning, exiting;double[] measured=new double[4];double[]? goal;string? signature;
    public bool Busy=>planning||controller.State is GroupState.Arming or GroupState.Executing or GroupState.Stopping or GroupState.Settling or GroupState.Disarming;
    internal bool PollsFeedback => controller.PollsFeedback;
    public WholeBodyPage(ICanTransport bus,MotorRegistry registry,MotorController legacy,Func<IReadOnlyDictionary<int,string>> firmware,
        Action<string> log)
    {
        this.bus=bus;this.registry=registry;this.firmware=firmware;Dock=DockStyle.Fill;BackColor=Color.FromArgb(24,29,37);ForeColor=Color.Gainsboro;
        string root=Path.Combine(Application.StartupPath,"assets","robot");model=Robot.Load(Path.Combine(root,"urdf","urdf_g3_06.urdf"),Path.Combine(root,"meshes"));controller=new(model,new Calibration(),bus,registry,legacy);
        view = new RobotView(model, () => RunUiAction(ReadCurrent)) { Dock = DockStyle.Fill };
        controller.Log += log;
        this.log = log;
        var split=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=2,RowCount=1,Padding=new(12)};split.ColumnStyles.Add(new(SizeType.Percent,33.333f));split.ColumnStyles.Add(new(SizeType.Percent,66.667f));Controls.Add(split);
        split.Controls.Add(view, 0, 0);
        var rightHost = new BufferedPanel { Dock = DockStyle.Fill, AutoScroll = true };
        var sections = new TableLayoutPanel { Dock = DockStyle.Top, ColumnCount = 1, RowCount = 2, Padding = new(4) };
        const int poseControlsHeight = 400;
        sections.RowStyles.Add(new(SizeType.Absolute, poseControlsHeight));
        sections.RowStyles.Add(new(SizeType.Percent, 100));
        rightHost.Controls.Add(sections);
        split.Controls.Add(rightHost, 1, 0);
        void ResizeSections()
        {
            int height = Math.Max(rightHost.ClientSize.Height, (poseControlsHeight + 138) * rightHost.DeviceDpi / 96);
            if (sections.Height != height) sections.Height = height;
            rightHost.AutoScrollMinSize = new Size(0, height);
        }
        rightHost.ClientSizeChanged += (_, _) => ResizeSections();
        rightHost.DpiChangedAfterParent += (_, _) => ResizeSections();
        var controlsFrame = new SurfacePanel { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 0, 8), Padding = new(10, 6, 10, 6) };
        sections.Controls.Add(controlsFrame, 0, 0);
        var right = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 8 };
        right.RowStyles.Add(new(SizeType.AutoSize));
        foreach (int rowHeight in new[] { 116, 29, 29, 26, 42, 31, 31 })
            right.RowStyles.Add(new(SizeType.Absolute, rowHeight));
        controlsFrame.Controls.Add(right);

        var header = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 3, Margin = Padding.Empty,
            AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink };
        header.ColumnStyles.Add(new(SizeType.Percent, 100));
        header.ColumnStyles.Add(new(SizeType.Absolute, 150));
        header.ColumnStyles.Add(new(SizeType.Absolute, 150));
        header.RowStyles.Add(new(SizeType.Absolute, 22));
        header.RowStyles.Add(new(SizeType.Absolute, 22));
        header.RowStyles.Add(new(SizeType.AutoSize));
        header.Controls.Add(new Label { Text = "位姿控制", AutoSize = true, ForeColor = UiTheme.Text,
            Font = UiTheme.Font(12, FontStyle.Bold) }, 0, 0);
        status.Font = UiTheme.Font(9.5f);
        header.Controls.Add(status, 0, 1);
        trackingStatus.Font = status.Font;
        header.Controls.Add(trackingStatus, 0, 2);
        header.SetColumnSpan(trackingStatus, 3);
        parkingRecoveryButton = CommandButton("停车区退让", ActionKind.Secondary, () => _ = RunAsync(recoverParking: true));
        parkingRecoveryButton.Anchor = AnchorStyles.Right;
        parkingRecoveryButton.Size = new Size(140, 32);
        parkingRecoveryButton.Enabled = false;
        header.Controls.Add(parkingRecoveryButton, 1, 0);
        header.SetRowSpan(parkingRecoveryButton, 2);
        recordingButton = CommandButton("录制跟踪", ActionKind.Secondary, ToggleRecording);
        recordingButton.Anchor = AnchorStyles.Right;
        recordingButton.Size = new Size(140, 32);
        recordingButton.Enabled = false;
        header.Controls.Add(recordingButton, 2, 0);
        header.SetRowSpan(recordingButton, 2);
        right.Controls.Add(header, 0, 0);

        var cards = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 2, Margin = Padding.Empty };
        for (int i = 0; i < 2; i++)
        {
            cards.ColumnStyles.Add(new(SizeType.Percent, 50));
            cards.RowStyles.Add(new(SizeType.Percent, 50));
        }
        right.Controls.Add(cards, 0, 1);
        string[] names=["高度 · mm","前后 · mm","俯仰 · °","航向 · °"];string[] minus=["降低 −","后退 −","后仰 −","航向 −"],plus=["升高 +","前进 +","前倾 +","航向 +"];double[] initial=[972,66,0,0];
        for(int i=0;i<4;i++)
        {
            int axis=i;
            var card = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 2,
                Padding = new(7, 2, 7, 2), Margin = new(3), BackColor = Color.FromArgb(34, 41, 53) };
            card.RowStyles.Add(new(SizeType.Absolute, 20));
            card.RowStyles.Add(new(SizeType.Percent, 100));
            var cardHeader = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1, Margin = Padding.Empty };
            cardHeader.RowStyles.Add(new(SizeType.Percent, 100));
            cardHeader.ColumnStyles.Add(new(SizeType.Percent, 60));
            cardHeader.ColumnStyles.Add(new(SizeType.Percent, 40));
            var title = new Label { Text = names[i], Dock = DockStyle.Fill, AutoEllipsis = true, Font = UiTheme.Font(10, FontStyle.Bold), Margin = Padding.Empty };
            if (axis == 0) heightTitle = title;
            if (axis == 1) frontTitle = title;
            cardHeader.Controls.Add(title, 0, 0);
            actual[i] = new() { Text = "实际 —", Dock = DockStyle.Fill, AutoEllipsis = true, Font = UiTheme.Font(9),
                Margin = Padding.Empty, TextAlign = ContentAlignment.MiddleRight };
            cardHeader.Controls.Add(actual[i], 1, 0);
            card.Controls.Add(cardHeader, 0, 0);
            var editor = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 1, Margin = Padding.Empty };
            editor.RowStyles.Add(new(SizeType.Percent, 100));
            editor.ColumnStyles.Add(new(SizeType.Percent, 100));
            editor.ColumnStyles.Add(new(SizeType.Absolute, 67));
            editor.ColumnStyles.Add(new(SizeType.Absolute, 67));
            target[i]=new(){Minimum=i==0?(decimal)MoveLWorkspace.ReferenceMinimumMm:i==1?-2000:-180,
                Maximum=i==0?(decimal)MoveLWorkspace.ZeroHeightMm:i==1?2000:180,
                Value=(decimal)initial[i],DecimalPlaces=2,Anchor=AnchorStyles.Left|AnchorStyles.Right,
                Font=UiTheme.Font(10),Margin=new Padding(0,0,4,0)};
            editor.Controls.Add(target[i], 0, 0);
            target[i].ValueChanged += (_, _) => TargetEdited(axis);
            foreach (int direction in new[] { -1, 1 })
            {
                int delta = direction;
                var button = JogButton(direction < 0 ? minus[i] : plus[i], () => _ = RunAsync(axis, delta));
                button.Dock = DockStyle.Fill;
                motion.Add(button);
                editor.Controls.Add(button, direction < 0 ? 1 : 2, 0);
            }
            card.Controls.Add(editor, 0, 1);
            cards.Controls.Add(card, i % 2, i / 2);
        }
        var steps=new FlowLayoutPanel{Dock=DockStyle.Fill};steps.Controls.AddRange([new Label{Text="位置步长 mm",AutoSize=true},positionStep,new Label{Text="角度步长 °",AutoSize=true},angleStep]);right.Controls.Add(steps,0,2);
        var limits = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false };
        limits.Controls.AddRange([
            new Label { Text = "最大速度 rpm", AutoSize = true }, maxSpeed,
            new Label { Text = "最大加速度 rpm/s", AutoSize = true }, maxAcceleration,
        ]);
        right.Controls.Add(limits, 0, 3);
        right.Controls.Add(keepPitch, 0, 4);
        keepPitch.CheckedChanged += (_, _) =>
        {
            if (keepPitch.Checked)
            {
                fillingTargets = true;
                try { target[2].Value = Math.Clamp((decimal)(DiskKinematics.Forward(measured).Pitch * 180 / Math.PI),
                    target[2].Minimum, target[2].Maximum); }
                finally { fillingTargets = false; }
                editedTargetAxes &= ~(1 << 2);
            }
            UpdateCenteredFront();
        };
        var commands = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 1, Margin = Padding.Empty };
        commands.RowStyles.Add(new(SizeType.Percent, 100));
        for (int i = 0; i < 3; i++) commands.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / 3));
        var execute = CommandButton("执行到位姿", ActionKind.Primary, () => _ = RunAsync());
        var returnToZero = CommandButton("CSP回零位", ActionKind.Primary, () => _ = RunAsync(returnToZero: true));
        motion.Add(execute);
        motion.Add(returnToZero);
        var stop = CommandButton("停止并退出 CSP", ActionKind.Danger, StopAndExit);
        foreach (var button in new[] { execute, stop, returnToZero }) button.Dock = DockStyle.Fill;
        commands.Controls.Add(execute, 0, 0);
        commands.Controls.Add(stop, 1, 0);
        commands.Controls.Add(returnToZero, 2, 0);
        right.Controls.Add(commands, 0, 5);
        right.Controls.Add(hint, 0, 6);
        right.Controls.Add(recordingStatus, 0, 7);

        string pointsPath = Path.Combine(Path.GetDirectoryName(Store.FilePath)!, "waypoints.json");
        points = new WholeBodyPointsPanel(new WholeBodyPointStore(pointsPath), () => Store.Load(Signs()),
            CapturePointPose, point => _ = RunAsync(point: point)) { Dock = DockStyle.Fill, Margin = Padding.Empty };
        sections.Controls.Add(points, 0, 1);
        ResizeSections();
        VisibleChanged += (_, _) =>
        {
            pendingActualTargets = true;
            if (Visible) { points.Reload(); RefreshScene(); }
        };
        timer.Tick+=(_,_)=>RefreshScene();timer.Start();
    }
    private static NumericUpDown NewOutputLimit(decimal maximum, decimal initial) => new()
    {
        Minimum = MotionPlan.MinimumOutputLimit,
        Maximum = maximum,
        Value = initial,
        Increment = .1m,
        DecimalPlaces = 2,
        Width = 100,
    };

    private void ToggleRecording()
    {
        if (recordingToSave is not null || controller.RecordingComplete.HasValue)
        {
            _ = SaveRecordingAsync();
            return;
        }
        controller.StartRecording();
        recordingStatus.Text = "录制中 · 最多约 2 分钟";
        UpdateRecordingButton();
    }

    private void UpdateRecordingButton()
    {
        bool hasRecording = controller.RecordingComplete.HasValue;
        recordingButton.Text = savingRecording ? "正在保存…" : recordingToSave is not null ? "重试保存"
            : hasRecording ? "停止并保存" : "录制跟踪";
        recordingButton.Enabled = !savingRecording && (recordingToSave is not null || hasRecording
            || (bus.IsOpen && controller.State != GroupState.FaultLatched));
    }

    private async Task SaveRecordingAsync()
    {
        if (savingRecording) return;
        recordingToSave ??= controller.TakeRecording();
        if (recordingToSave is not { } capture) return;
        if (capture.Count == 0)
        {
            recordingToSave = null;
            recordingStatus.Text = "未采集到 CSP 数据，请先录制再执行";
            UpdateRecordingButton();
            return;
        }

        savingRecording = true;
        recordingStatus.Text = "正在保存跟踪曲线…";
        UpdateRecordingButton();
        try
        {
            string directory = Path.Combine(Application.StartupPath, "logs", "traces");
            string path = await Task.Run(() => capture.Save(directory));
            recordingToSave = null;
            recordingStatus.Text = $"已保存 · {Path.GetFileName(path)}";
            log($"CSP 跟踪录制已保存 · {path}");
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            recordingStatus.Text = "保存失败，数据仍在内存中，可重试保存";
            log($"CSP 跟踪录制保存失败 · {e.Message}");
        }
        finally
        {
            savingRecording = false;
            if (!IsDisposed) UpdateRecordingButton();
        }
    }

    private Button CommandButton(string label, ActionKind kind, Action action)
    {
        var button = new ActionButton
        {
            Text = label,
            Kind = kind,
            Anchor = AnchorStyles.Left | AnchorStyles.Top | AnchorStyles.Right,
            Height = 42,
            Margin = new(4),
        };
        button.Click += (_, _) => RunUiAction(action);
        return button;
    }

    private void RunUiAction(Action action)
    {
        try { action(); }
        catch (Exception e) { hint.Text = e.Message; }
    }

    private Button JogButton(string label, Action action)
    {
        var button = new ActionButton
        {
            Text = label,
            Kind = ActionKind.Jog,
            Enabled = false,
            Size = new(90, 24),
            Font = UiTheme.Font(9),
            Margin = new(1),
        };
        button.Click += (_, _) => RunUiAction(action);
        return button;
    }
    internal static string StateLabel(GroupState state, bool targetReached, bool stoppedBeforeTarget) => state switch
    {
        GroupState.Observing => "空闲",
        GroupState.Arming => "准备中",
        GroupState.Holding when targetReached => "保持 / 已到位",
        GroupState.Holding when stoppedBeforeTarget => "保持 / 提前停止",
        GroupState.Holding => "保持",
        GroupState.Executing => "执行中",
        GroupState.Stopping => "正在停止",
        GroupState.Settling when stoppedBeforeTarget => "等待停稳",
        GroupState.Settling => "等待到位",
        GroupState.Disarming => "正在退出 CSP",
        _ => "故障",
    };
    int[] Signs()=>registry.Motors.Select(m=>m.Direction).ToArray();
    private MotorFeedback[] ReadMotorFeedback()
    {
        const string unavailable = "请连接设备并等待四轴有效反馈";
        if (!bus.IsOpen) throw new InvalidOperationException(unavailable);
        var feedback = registry.Motors.Select(m => bus.GetFeedback(m.DeviceId) ?? throw new InvalidOperationException(unavailable)).ToArray();
        long now = MotorFeedback.Now;
        if (feedback.Any(axis => now - axis.Timestamp > 100 || axis.Timestamp > now || axis.HasError || axis.FaultCode != 0))
            throw new InvalidOperationException(unavailable);
        return feedback;
    }
    private static double[] ToUrdfJoints(MotorFeedback[] feedback, int[] signs) =>
        feedback.Select((axis, i) => Calibration.FeedbackToUrdf(i, axis.PositionRaw, signs[i])).ToArray();
    private double[] ReadFeedback() => ToUrdfJoints(ReadMotorFeedback(), Signs());
    static double[] Values(TaskPose p)=>[p.Z*1000,p.X*1000,p.Pitch*180/Math.PI,p.Yaw*180/Math.PI];
    void Fill(TaskPose pose)
    {
        var values = Values(pose);
        fillingTargets = true;
        try
        {
            for (int i = 0; i < 4; i++)
                target[i].Value = Math.Clamp((decimal)values[i], target[i].Minimum, target[i].Maximum);
        }
        finally { fillingTargets = false; }
    }
    void ReadCurrent()
    {
        if (Busy) throw new InvalidOperationException("请等待全身运控动作结束");
        Fill(DiskKinematics.Forward(ReadFeedback()));
        pendingActualTargets = false;
        editedTargetAxes = 0;
        UpdateFrontTitle();
        hint.Text = "当前位姿已填入目标栏";
    }

    private void TargetEdited(int axis)
    {
        if (fillingTargets) return;
        if (axis == 2 && keepPitch.Checked) keepPitch.Checked = false;
        editedTargetAxes |= 1 << axis;
        if (axis == 0) editedTargetAxes &= ~(1 << 1);
        UpdateFrontTitle();
        if (axis != 1) UpdateCenteredFront();
    }

    private void UpdateCenteredFront()
    {
        if ((editedTargetAxes & 3) != 1) return;
        if (targetCalibration is not { } calibration)
        {
            frontTitle.Text = "前后 · 未标定";
            return;
        }
        try
        {
            var request = ReadTargetPose();
            if (keepPitch.Checked) request = request with { Pitch = DiskKinematics.Forward(measured).Pitch };
            var (pose, _) = MotionPlan.ResolveCenteredEndpoint(calibration, request);
            fillingTargets = true;
            target[1].Value = Math.Clamp((decimal)(pose.X * 1000), target[1].Minimum, target[1].Maximum);
            UpdateFrontTitle();
        }
        catch (ArgumentException)
        {
            frontTitle.Text = "前后 · 无可用自动值";
        }
        finally { fillingTargets = false; }
    }

    private void UpdateFrontTitle() => frontTitle.Text = (editedTargetAxes & 3) switch
    {
        1 => "前后 · 自动 mm",
        2 or 3 => "前后 · 手填 mm",
        _ => "前后 · mm",
    };

    private void UpdateWorkspace(Calibration? calibration, string? version)
    {
        if (version == workspaceVersion) return;
        workspaceVersion = version;
        targetCalibration = calibration;
        heightTitle.Text = "高度 · mm";
        fillingTargets = true;
        try
        {
            if (calibration is not null)
            {
                target[1].Minimum = (decimal)(-1000 * calibration.TaskLimits.BackwardLimit);
                target[1].Maximum = (decimal)(1000 * calibration.TaskLimits.ForwardLimit);
                var workspace = MoveLWorkspace.FromCalibration(calibration);
                target[0].Minimum = (decimal)workspace.MinimumHeightMm;
                heightTitle.Text = $"高度 {workspace.MinimumHeightMm:F0}～{workspace.MaximumHeightMm:F0} mm";
            }
        }
        catch (ArgumentException e) { hint.Text = e.Message; }
        finally { fillingTargets = false; }
        UpdateCenteredFront();
    }

    void StopAndExit()
    {
        operation?.Cancel();
        goal = null;
        exiting = controller.Owns;
        controller.StopAndExit();
        hint.Text = exiting ? "正在停止并退出 CSP…" : "当前未占用 CSP";
    }

    private TaskPose JogRequest(int axis, int direction)
    {
        decimal next = target[axis].Value + direction * (axis < 2 ? positionStep.Value : angleStep.Value);
        if (next < target[axis].Minimum || next > target[axis].Maximum)
            throw new InvalidOperationException("目标超出输入范围，请调整目标或步长");
        target[axis].Value = next;
        return ReadTargetPose();
    }

    private TaskPose ReadTargetPose() => new((double)target[1].Value / 1000, (double)target[0].Value / 1000,
        (double)target[2].Value * Math.PI / 180, (double)target[3].Value * Math.PI / 180);

    private (TaskPose Pose, string Source) CapturePointPose()
    {
        if (Busy) throw new InvalidOperationException("请等待全身运控动作结束");
        try { return (DiskKinematics.Forward(ReadFeedback()), "已填入当前实测高度、俯仰和航向，可修改"); }
        catch (InvalidOperationException)
        {
            return (new TaskPose(0, (double)target[0].Value / 1000, (double)target[2].Value * Math.PI / 180,
                (double)target[3].Value * Math.PI / 180), "当前无实时反馈，已填入上方位姿目标，可修改");
        }
    }

    async Task RunAsync(int axis = -1, int direction = 0, bool returnToZero = false, bool recoverParking = false,
        WholeBodyPoint? point = null)
    {
        if(Busy)return;planning=true;operation?.Dispose();operation=CancellationTokenSource.CreateLinkedTokenSource(lifetime.Token);var token=operation.Token;
        try
        {
            if (controller.State == GroupState.FaultLatched)
                throw new InvalidOperationException("请先点击“停止并退出 CSP”，确认四轴失能抱闸");
            if (!recoverParking && controller.InParkingZone)
                throw new InvalidOperationException("停车区锁定，回到规划区并停稳后再执行");
            var calibration = Store.Load(Signs());
            var measuredStart = ReadFeedback();
            if (recoverParking && !controller.CanRecoverParking)
                throw new InvalidOperationException("退让需要有效的 CSP 保持指令，请先确认四轴已停稳");
            var start = controller.Owns ? controller.PlanningStart() : measuredStart;
            if (pendingActualTargets)
            {
                Fill(DiskKinematics.Forward(start));
                pendingActualTargets = false;
                editedTargetAxes = 0;
                UpdateFrontTitle();
            }
            try { calibration.Check(start); }
            catch (ArgumentException e) { throw new InvalidOperationException("起点超限 · " + e.Message, e); }
            if (!recoverParking) calibration.CheckWholeBodyStart(start);
            bool zeroMove = returnToZero || point?.IsZeroPose == true;
            TaskPose request;
            PositionPriority priority = PositionPriority.Both;
            if (recoverParking)
            {
                request = default;
                hint.Text = "正在验证低速向内退让…";
            }
            else if (zeroMove)
            {
                request = DiskKinematics.Forward(new double[4]);
                hint.Text = "正在规划 CSP 回零位 · 高 972 mm · 前后 66 mm · 俯仰 0° · 航向 0°";
            }
            else if (point is not null)
            {
                request = point.Pose;
                hint.Text = $"正在规划点位「{point.Name}」…";
            }
            else
            {
                request = axis >= 0 ? JogRequest(axis, direction) : ReadTargetPose();
                if (axis < 0 && (editedTargetAxes & 3) == 0)
                {
                    var current = DiskKinematics.Forward(start);
                    bool orientationOnly = (editedTargetAxes & 12) != 0;
                    request = request with
                    {
                        X = orientationOnly ? current.X : request.X,
                        Z = orientationOnly ? current.Z : request.Z,
                        Pitch = (editedTargetAxes & 4) != 0 ? request.Pitch : current.Pitch,
                        Yaw = (editedTargetAxes & 8) != 0 ? request.Yaw : current.Yaw,
                    };
                }
                if ((editedTargetAxes & 3) == 1) priority = PositionPriority.Height;
                hint.Text = "正在规划并验证平滑曲线…";
            }
            foreach (var input in target) input.Enabled = false;
            bool holdPitch = !zeroMove && keepPitch.Checked;
            keepPitch.Enabled = false;
            double speed = (double)maxSpeed.Value, acceleration = (double)maxAcceleration.Value;
            var plan = await Task.Run(() => recoverParking
                ? MotionPlan.CreateParkingRecovery(model, calibration, start, measuredStart, token)
                : zeroMove
                ? MotionPlan.CreateSmoothToZero(model, calibration, start, speed, acceleration, token)
                : MotionPlan.CreateSmooth(model, calibration, start, request, speed, acceleration, token, priority, holdPitch), token);
            token.ThrowIfCancellationRequested();if(Store.Load(Signs()).Fingerprint!=calibration.Fingerprint)throw new InvalidOperationException("标定已变化，请重新执行");
            var fresh=ReadFeedback();if(fresh.Where((q,i)=>Math.Abs(q-measuredStart[i])>.001).Any())throw new InvalidOperationException("规划期间起点变化，请重新执行");
            controller.SetCalibration(calibration);
            signature=calibration.Fingerprint;
            if (!controller.Owns)
            {
                if (recoverParking) throw new InvalidOperationException("CSP 保持已结束，请重新检查");
                var versions = firmware();
                long deadline = Environment.TickCount64 + 150;
                while (true)
                {
                    token.ThrowIfCancellationRequested();
                    try { controller.Arm(versions); break; }
                    catch (FeedbackNotReadyException) when (!controller.Owns && Environment.TickCount64 < deadline)
                    {
                        await Task.Delay(10, token);
                    }
                }
            }
            while(controller.State==GroupState.Arming){await Task.Delay(10,token);}
            token.ThrowIfCancellationRequested();controller.Execute(plan);goal=plan.End;
            var resolved = plan.ExpectedPose(plan.Duration) ?? DiskKinematics.Forward(goal);
            string adjustment = "";
            if (priority == PositionPriority.Height && Math.Abs(resolved.X - request.X) > .000005)
                adjustment = $"前后调整为 {resolved.X * 1000:F2} mm · ";
            if (!recoverParking)
            {
                Fill(resolved with { Yaw = DiskKinematics.Wrap(resolved.Yaw) });
                if (zeroMove) editedTargetAxes = 0;
                else if (point is not null) editedTargetAxes = 15;
                else if (axis >= 0) editedTargetAxes &= 3;
                UpdateFrontTitle();
            }
            hint.Text = recoverParking
                ? $"停车区退让 · 预计 {plan.Duration:F2}s · 速度上限 0.10 rpm"
                : zeroMove
                ? $"CSP回零位 · 高 {resolved.Z * 1000:F0} mm · 前后 {resolved.X * 1000:F0} mm · " +
                    $"俯仰 0° · 航向 0° · 预计 {plan.Duration:F2}s"
                : $"{adjustment}平滑曲线 · 预计 {plan.Duration:F2}s · " +
                    (holdPitch ? "保持俯仰" : "俯仰平滑过渡") +
                    (plan.SmoothLegCount > 1 ? $" · {plan.SmoothLegCount} 段，过渡点平滑停稳" : "");
            string limits = recoverParking ? " · 退让加速度上限 0.50 rpm/s"
                : $" · 规划速度上限 {speed:F2} rpm · 规划加速度上限 {acceleration:F2} rpm/s";
            if (plan.ExpectedPose(0) is not null)
                limits += $" · 计划关节峰值约 {plan.PlannedPeakRpm:F2} rpm";
            if (point is not null) hint.Text = $"点位「{point.Name}」 · {hint.Text}";
            log($"全身运控 · {hint.Text}{limits}"
                + $" · 起点高度 {DiskKinematics.Forward(start).Z * 1000:F2} mm · 目标高度 {resolved.Z * 1000:F2} mm" +
                $" · 前后 {resolved.X * 1000:F2} mm · 俯仰 {resolved.Pitch * 180 / Math.PI:F3}°");
        }
        catch(OperationCanceledException){if(!exiting)hint.Text="已取消当前请求";}
        catch(Exception e){hint.Text=e.Message;log("全身运控请求未执行 · " + e.Message);}
        finally
        {
            planning = false;
            for (int i = 0; i < target.Length; i++) target[i].Enabled = !Busy && (i != 2 || !keepPitch.Checked);
            keepPitch.Enabled = !Busy;
        }
    }
    void RefreshScene()
    {
        if (controller.RecordingComplete == true) _ = SaveRecordingAsync();
        UpdateRecordingButton();
        Calibration? c=null;try{c=Store.Load(Signs());}catch{}
        string? fingerprint = c?.Fingerprint;
        UpdateWorkspace(c, fingerprint);
        if(controller.State != GroupState.Disarming && controller.PollsFeedback&&(fingerprint==null||fingerprint!=signature)){operation?.Cancel();controller.Emergency();}
        var (state, targetReached, stoppedBeforeTarget) = controller.MotionStatus;
        bool parked = controller.InParkingZone;
        status.Text=$"{(c==null?"当前未标定":$"已标定 · 前后 {-c.TaskLimits.BackwardLimit*1000:F0}～{c.TaskLimits.ForwardLimit*1000:F0}mm")} · {StateLabel(state, targetReached, stoppedBeforeTarget)}";
        if (parked) status.Text += " · 停车区";
        trackingStatus.Text = controller.MoveLTracking ?? " ";
        if(!Visible)return;bool stale=false;
        double[]? anglesDeg = null;
        try
        {
            var feedback = ReadMotorFeedback();
            var signs = Signs();
            anglesDeg = feedback.Select((axis, i) => axis.PositionRaw == 0 ? 0 : signs[i] * axis.PositionDeg).ToArray();
            measured = ToUrdfJoints(feedback, signs);
            var pose = DiskKinematics.Forward(measured);
            var p = Values(pose);
            for (int i = 0; i < 4; i++) actual[i].Text = $"实际 {p[i]:F2}";
            if (pendingActualTargets && !Busy)
            {
                Fill(pose);
                editedTargetAxes = 0;
                pendingActualTargets = false;
                UpdateFrontTitle();
            }
        }
        catch { stale = true; foreach (var l in actual) l.Text = "实际 · 无反馈"; }
        bool ready = !Busy && !stale && !pendingActualTargets;
        bool calibrated = ready && c is not null && bus.IsOpen;
        bool canRun = calibrated && !parked && state is GroupState.Observing or GroupState.Holding;
        foreach (var b in motion) b.Enabled = canRun;
        points.SetAvailability(canRun, !Busy && c is not null, !Busy);
        parkingRecoveryButton.Enabled = calibrated && controller.CanRecoverParking;
        for (int i = 0; i < target.Length; i++) target[i].Enabled = ready && (i != 2 || !keepPitch.Checked);
        keepPitch.Enabled = !Busy;
        view.UpdateJointFeedback(anglesDeg);
        try{view.UpdatePose(measured,goal,stale);}catch(Exception e){controller.Stop();hint.Text=e.Message;}
        bool showParking = parked && state == GroupState.Holding;
        if (state == GroupState.FaultLatched || (exiting && state != GroupState.Observing) ||
            (!planning && (stoppedBeforeTarget || showParking || (showingParkingStatus && state == GroupState.Holding))))
            hint.Text = controller.Status;
        else if (exiting)
        {
            exiting = false;
            hint.Text = "已失能抱闸并退出 CSP";
        }
        showingParkingStatus = showParking;
    }
    public bool PrepareClose()
    {
        operation?.Cancel();
        if (controller.Owns)
        {
            StopAndExit();
            return false;
        }
        if (controller.RecordingComplete.HasValue) _ = SaveRecordingAsync();
        return !savingRecording;
    }
    internal void PrepareEmergencyStop()
    {
        operation?.Cancel();
        goal = null;
        exiting = controller.Owns;
    }

    public void ConnectionClosing(){operation?.Cancel();controller.Emergency();}
    public void ConnectionClosed(){controller.ConnectionClosed();signature=null;goal=null;exiting=false;pendingActualTargets=true;}
    public void InvalidateCalibration(){operation?.Cancel();goal=null;hint.Text="零位已变更，请核对标定角度范围";}
    public void CalibrationChanged(){operation?.Cancel();goal=null;signature=null;}
    protected override void Dispose(bool disposing){if(disposing){lifetime.Cancel();timer.Dispose();controller.Dispose();lifetime.Dispose();}base.Dispose(disposing);}
}
