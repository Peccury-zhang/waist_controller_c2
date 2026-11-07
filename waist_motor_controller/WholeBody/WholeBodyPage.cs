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
    readonly NumericUpDown[] target=new NumericUpDown[4];readonly Label[] actual=new Label[4];readonly Label status=new(){AutoSize=true},hint=new(){AutoSize=true};
    readonly NumericUpDown positionStep = new() { Minimum = .1m, Maximum = 100, Value = 10, Increment = 5, DecimalPlaces = 1 };
    readonly NumericUpDown angleStep = new() { Minimum = .1m, Maximum = 30, Value = 2, Increment = 1, DecimalPlaces = 1 };
    readonly NumericUpDown maxSpeed = NewOutputLimit();
    readonly NumericUpDown maxAcceleration = NewOutputLimit();
    readonly List<Button> motion=new();readonly System.Windows.Forms.Timer timer=new(){Interval=50};readonly CancellationTokenSource lifetime=new();CancellationTokenSource? operation;
    readonly Action<string> logFailure;
    private bool fillingTargets;
    private int editedTargetAxes;
    private MoveLWorkspace? workspace;
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
        logFailure = message => log("全身运控请求未执行 · " + message);
        var split=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=2,RowCount=1,Padding=new(12)};split.ColumnStyles.Add(new(SizeType.Percent,33.333f));split.ColumnStyles.Add(new(SizeType.Percent,66.667f));Controls.Add(split);
        split.Controls.Add(view, 0, 0);
        var right = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 7, Padding = new(16) };
        var cardsRow = new RowStyle(SizeType.Absolute, 240);
        right.RowStyles.Add(new(SizeType.Absolute, 26));
        right.RowStyles.Add(cardsRow);
        right.RowStyles.Add(new(SizeType.Absolute, 36));
        right.RowStyles.Add(new(SizeType.Absolute, 36));
        right.RowStyles.Add(new(SizeType.Absolute, 58));
        right.RowStyles.Add(new(SizeType.Absolute, 36));
        right.RowStyles.Add(new(SizeType.Percent, 100));
        split.Controls.Add(right, 1, 0);
        right.Controls.Add(status, 0, 0);
        var cardsHost = new Panel { Dock = DockStyle.Fill, AutoScroll = true, Margin = Padding.Empty };
        var cards = new TableLayoutPanel { Dock = DockStyle.Top, ColumnCount = 2, RowCount = 2 };
        for (int i = 0; i < 2; i++)
        {
            cards.ColumnStyles.Add(new(SizeType.Percent, 50));
            cards.RowStyles.Add(new(SizeType.Percent, 50));
        }
        int MinimumCardsHeight() => 240 * cardsHost.DeviceDpi / 96;
        void ResizeCards() => cards.Height = Math.Max(cardsHost.ClientSize.Height, MinimumCardsHeight());
        void FitCardsRow()
        {
            int reserved = right.RowStyles.Cast<RowStyle>()
                .Where(row => row != cardsRow && row.SizeType == SizeType.Absolute)
                .Sum(row => (int)row.Height);
            int available = Math.Max(0, right.ClientSize.Height - right.Padding.Vertical - reserved);
            int height = Math.Min(available, Math.Max(available * 2 / 3, MinimumCardsHeight()));
            if (cardsRow.Height != height) cardsRow.Height = height;
        }
        cardsHost.ClientSizeChanged += (_, _) => ResizeCards();
        cardsHost.DpiChangedAfterParent += (_, _) => ResizeCards();
        cardsHost.Controls.Add(cards);
        right.Controls.Add(cardsHost, 0, 1);
        right.Layout += (_, _) => FitCardsRow();
        FitCardsRow();
        ResizeCards();
        string[] names=["高度 · mm","前后 · mm","俯仰 · °","航向 · °"];string[] minus=["降低 −","后退 −","后仰 −","航向 −"],plus=["升高 +","前进 +","前倾 +","航向 +"];double[] initial=[972,66,0,0];
        for(int i=0;i<4;i++)
        {
            int axis=i;var card=new TableLayoutPanel{Dock=DockStyle.Fill,ColumnCount=1,RowCount=4,Padding=new(12,6,12,6),Margin=new(4),BackColor=Color.FromArgb(34,41,53)};
            var title = new Label { Text = names[i], AutoSize = true, Font = new Font(Font.FontFamily, 13, FontStyle.Bold) };
            if (axis == 0) heightTitle = title;
            if (axis == 1) frontTitle = title;
            card.Controls.Add(title,0,0);actual[i]=new(){Text="实际 —",AutoSize=true};card.Controls.Add(actual[i],0,1);
            target[i]=new(){Minimum=i==0?(decimal)MoveLWorkspace.ReferenceMinimumMm:i==1?-2000:-180,
                Maximum=i==0?(decimal)MoveLWorkspace.ZeroHeightMm:i==1?2000:180,
                Value=(decimal)initial[i],DecimalPlaces=2,Dock=DockStyle.Top};card.Controls.Add(target[i],0,2);
            target[i].ValueChanged += (_, _) => TargetEdited(axis);
            var buttons = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoSize = true };
            foreach (int direction in new[] { -1, 1 })
            {
                int delta = direction;
                var button = JogButton(direction < 0 ? minus[i] : plus[i], () => _ = RunAsync(axis, delta));
                motion.Add(button);
                buttons.Controls.Add(button);
            }
            card.Controls.Add(buttons, 0, 3);
            cards.Controls.Add(card, i % 2, i / 2);
        }
        var steps=new FlowLayoutPanel{Dock=DockStyle.Fill};steps.Controls.AddRange([new Label{Text="位置步长 mm",AutoSize=true},positionStep,new Label{Text="角度步长 °",AutoSize=true},angleStep]);right.Controls.Add(steps,0,2);
        var limits = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false };
        limits.Controls.AddRange([
            new Label { Text = "最大速度 rpm", AutoSize = true }, maxSpeed,
            new Label { Text = "最大加速度 rpm/s", AutoSize = true }, maxAcceleration,
        ]);
        right.Controls.Add(limits, 0, 3);
        var commands = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 1 };
        for (int i = 0; i < 3; i++) commands.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / 3));
        var execute = CommandButton("执行到位姿", ActionKind.Primary, () => _ = RunAsync());
        var returnToZero = CommandButton("CSP回零位", ActionKind.Primary, () => _ = RunAsync(returnToZero: true));
        motion.Add(execute);
        motion.Add(returnToZero);
        commands.Controls.Add(execute, 0, 0);
        commands.Controls.Add(CommandButton("停止并退出 CSP", ActionKind.Danger, StopAndExit), 1, 0);
        commands.Controls.Add(returnToZero, 2, 0);
        right.Controls.Add(commands, 0, 4);
        right.Controls.Add(hint, 0, 5);
        timer.Tick+=(_,_)=>RefreshScene();timer.Start();
    }
    private static NumericUpDown NewOutputLimit() => new()
    {
        Minimum = MotionPlan.MinimumOutputLimit,
        Maximum = MotionPlan.MaximumOutputLimit,
        Value = 1,
        Increment = .1m,
        DecimalPlaces = 2,
        Width = 100,
    };

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
    static string StateLabel(GroupState s)=>s switch {GroupState.Observing=>"空闲",GroupState.Arming=>"准备中",GroupState.Holding=>"保持 / 已到位",GroupState.Executing=>"执行中",GroupState.Stopping=>"正在停止",GroupState.Settling=>"等待到位",GroupState.Disarming=>"正在退出 CSP",_=>"故障"};
    int[] Signs()=>registry.Motors.Select(m=>m.Direction).ToArray();
    double[] ReadFeedback()
    {
        var f=registry.Motors.Select(m=>bus.GetFeedback(m.DeviceId)).ToArray();long now=MotorFeedback.Now;
        if(!bus.IsOpen||f.Any(x=>x==null||now-x.Timestamp>100||x.Timestamp>now||x.HasError||x.FaultCode!=0))throw new InvalidOperationException("请连接设备并等待四轴有效反馈");
        var signs = Signs();
        return f.Select((x, i) => Calibration.FeedbackToUrdf(i, x!.PositionRaw, signs[i])).ToArray();
    }
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
        editedTargetAxes = 0;
        UpdateFrontTitle();
        hint.Text = "当前位姿已填入目标栏";
    }

    private void TargetEdited(int axis)
    {
        if (fillingTargets) return;
        editedTargetAxes |= 1 << axis;
        UpdateFrontTitle();
        if (axis == 1 || (editedTargetAxes & 3) != 1 || workspace is not { } range) return;
        try
        {
            var calibration = Store.Load(Signs());
            var start = ReadFeedback();
            var pose = DiskKinematics.Forward(start);
            var request = pose with
            {
                Z = (double)target[0].Value / 1000,
                Pitch = (editedTargetAxes & 4) != 0 ? (double)target[2].Value * Math.PI / 180 : pose.Pitch,
                Yaw = (editedTargetAxes & 8) != 0 ? (double)target[3].Value * Math.PI / 180 : pose.Yaw,
            };
            range.CheckHeight(request.Z);
            double suggested = MotionPlan.SuggestCenteredX(calibration, start, request) * 1000;
            fillingTargets = true;
            target[1].Value = Math.Clamp(Math.Round((decimal)suggested, 2), target[1].Minimum, target[1].Maximum);
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException or InvalidDataException or IOException)
        {
            frontTitle.Text = "前后 · 待规划 mm";
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
        workspace = null;
        heightTitle.Text = "高度 · mm";
        fillingTargets = true;
        try
        {
            if (calibration is null) return;
            target[1].Minimum = (decimal)(-1000 * calibration.TaskLimits.BackwardLimit);
            target[1].Maximum = (decimal)(1000 * calibration.TaskLimits.ForwardLimit);
            workspace = MoveLWorkspace.FromCalibration(calibration);
            target[0].Minimum = (decimal)workspace.Value.MinimumHeightMm;
            heightTitle.Text = $"高度 · {workspace.Value.MinimumHeightMm:F0}～{workspace.Value.MaximumHeightMm:F0} mm";
        }
        catch (ArgumentException e) { hint.Text = e.Message; }
        finally { fillingTargets = false; }
    }

    void StopAndExit()
    {
        operation?.Cancel();
        goal = null;
        exiting = controller.Owns;
        controller.StopAndExit();
        hint.Text = exiting ? "正在停止并退出 CSP…" : "当前未占用 CSP";
    }

    async Task RunAsync(int axis = -1, int direction = 0, bool returnToZero = false)
    {
        if(Busy)return;planning=true;operation?.Dispose();operation=CancellationTokenSource.CreateLinkedTokenSource(lifetime.Token);var token=operation.Token;
        try
        {
            if (controller.State == GroupState.FaultLatched)
                throw new InvalidOperationException("请先点击“停止并退出 CSP”，确认四轴失能抱闸");
            var calibration=Store.Load(Signs());var start=ReadFeedback();
            try { calibration.Check(start); }
            catch (ArgumentException e) { throw new InvalidOperationException("起点超限 · " + e.Message, e); }
            TaskPose request;
            PositionPriority priority = axis switch
            {
                0 => PositionPriority.Height,
                1 => PositionPriority.ForeAft,
                -1 when !returnToZero && (editedTargetAxes & 3) == 1 => PositionPriority.Height,
                -1 when !returnToZero && (editedTargetAxes & 3) == 2 => PositionPriority.ForeAft,
                _ => PositionPriority.Both,
            };
            if (returnToZero)
            {
                request = DiskKinematics.Forward(new double[4]);
                hint.Text = "正在验证 CSP 回零路径…";
            }
            else
            {
                var t = target.Select(n => (double)n.Value).ToArray();
                var actualValues = Values(DiskKinematics.Forward(start));
                if (axis >= 0)
                {
                    t = actualValues;
                    t[axis] += direction * (double)(axis < 2 ? positionStep.Value : angleStep.Value);
                }
                else
                {
                    if ((editedTargetAxes & 3) == 0 && (editedTargetAxes & 12) != 0)
                        Array.Copy(actualValues, t, 2);
                    for (int i = 2; i < 4; i++)
                        if ((editedTargetAxes & (1 << i)) == 0) t[i] = actualValues[i];
                }
                request = new TaskPose(t[1] / 1000, t[0] / 1000, t[2] * Math.PI / 180, t[3] * Math.PI / 180);
                hint.Text = "正在规划并验证 MoveL 直线…";
            }
            foreach (var input in target) input.Enabled = false;
            double speed = (double)maxSpeed.Value, acceleration = (double)maxAcceleration.Value;
            var plan = await Task.Run(() => returnToZero
                ? MotionPlan.CreateIndependentToZero(model, calibration, start, speed, acceleration, token)
                : MotionPlan.CreateMoveL(model, calibration, start, request, speed, acceleration, token, priority), token);
            token.ThrowIfCancellationRequested();if(Store.Load(Signs()).Fingerprint!=calibration.Fingerprint)throw new InvalidOperationException("标定已变化，请重新执行");
            var fresh=ReadFeedback();if(fresh.Where((q,i)=>Math.Abs(q-start[i])>.001).Any())throw new InvalidOperationException("规划期间起点变化，请重新执行");
            controller.SetCalibration(calibration);
            signature=calibration.Fingerprint;
            if (!controller.Owns)
            {
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
            var resolved = plan.ExpectedMoveLPose(plan.Duration) ?? DiskKinematics.Forward(goal);
            string adjustment = "";
            if (priority == PositionPriority.Height && Math.Abs(resolved.X - request.X) > .000005)
                adjustment = $"前后调整为 {resolved.X * 1000:F2} mm · ";
            else if (priority == PositionPriority.ForeAft && Math.Abs(resolved.Z - request.Z) > .000005)
                adjustment = $"高度调整为 {resolved.Z * 1000:F2} mm · ";
            Fill(resolved with { Yaw = DiskKinematics.Wrap(resolved.Yaw) });
            if (axis >= 0 || returnToZero) editedTargetAxes = axis == 0 ? 1 : 0;
            UpdateFrontTitle();
            hint.Text = returnToZero
                ? $"CSP回零位 · 四轴目标 0° · 最长指令轨迹 {plan.Duration:F2}s"
                : $"{adjustment}MoveL · 预计 {plan.Duration:F2}s" +
                    (plan.MoveLLimitingConstraint is { } limit ? $" · {limit}" : "");
        }
        catch(OperationCanceledException){if(!exiting)hint.Text="已取消当前请求";}
        catch(Exception e){hint.Text=e.Message;logFailure(e.Message);}
        finally
        {
            planning = false;
            foreach (var input in target) input.Enabled = !Busy;
        }
    }
    void RefreshScene()
    {
        Calibration? c=null;try{c=Store.Load(Signs());}catch{}
        string? fingerprint = c?.Fingerprint;
        UpdateWorkspace(c, fingerprint);
        var state = controller.State;
        if(state != GroupState.Disarming && controller.PollsFeedback&&(fingerprint==null||fingerprint!=signature)){operation?.Cancel();controller.Emergency();}
        foreach(var b in motion)b.Enabled=c!=null&&!Busy&&bus.IsOpen;
        foreach (var input in target) input.Enabled = !Busy;
        var tracking = controller.MoveLTracking;
        status.Text=$"{(c==null?"当前未标定":$"已标定 · 前后 {-c.TaskLimits.BackwardLimit*1000:F0}～{c.TaskLimits.ForwardLimit*1000:F0}mm")} · {StateLabel(controller.State)}";
        if (tracking is not null) status.Text += Environment.NewLine + tracking;
        if(!Visible)return;bool stale=false;try{measured=ReadFeedback();var p=Values(DiskKinematics.Forward(measured));for(int i=0;i<4;i++)actual[i].Text=$"实际 {p[i]:F2}";}catch{stale=true;foreach(var l in actual)l.Text="实际 —（未连接/陈旧）";}
        try{view.UpdatePose(measured,goal,stale);}catch(Exception e){controller.Stop();hint.Text=e.Message;}
        if (state == GroupState.FaultLatched || (exiting && state != GroupState.Observing))
            hint.Text = controller.Status;
        else if (exiting)
        {
            exiting = false;
            hint.Text = "已失能抱闸并退出 CSP";
        }
    }
    public bool PrepareClose()
    {
        operation?.Cancel();
        if (!controller.Owns) return true;
        StopAndExit();
        return false;
    }
    internal void PrepareEmergencyStop()
    {
        operation?.Cancel();
        goal = null;
        exiting = controller.Owns;
    }

    public void ConnectionClosing(){operation?.Cancel();controller.Emergency();}
    public void ConnectionClosed(){controller.ConnectionClosed();signature=null;goal=null;exiting=false;}
    public void InvalidateCalibration(){operation?.Cancel();goal=null;hint.Text="零位已变更，请核对标定角度范围";}
    public void CalibrationChanged(){operation?.Cancel();goal=null;signature=null;}
    protected override void Dispose(bool disposing){if(disposing){lifetime.Cancel();timer.Dispose();controller.Dispose();lifetime.Dispose();}base.Dispose(disposing);}
}
