using WaistModel;
using waist_motor_controller.Protocol;
using waist_motor_controller.UI;

namespace waist_motor_controller.WholeBody;

internal sealed class WholeBodyPointsPanel : SurfacePanel
{
    private readonly WholeBodyPointStore store;
    private readonly Func<Calibration> loadCalibration;
    private readonly Func<(TaskPose Pose, string Source)> capture;
    private readonly Action<WholeBodyPoint> execute;
    private readonly TableLayoutPanel list = new() { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1, Margin = Padding.Empty };
    private readonly Label notice = new() { Dock = DockStyle.Fill, AutoEllipsis = true, ForeColor = UiTheme.Muted };
    private readonly ActionButton save;
    private readonly List<(WholeBodyPoint Point, SurfacePanel Card, Button Execute, Button Edit, Button Delete)> cards = [];
    private (bool Execute, bool Edit, bool Delete) availability;
    private Guid? selected;

    public WholeBodyPointsPanel(WholeBodyPointStore store, Func<Calibration> loadCalibration,
        Func<(TaskPose Pose, string Source)> capture, Action<WholeBodyPoint> execute)
    {
        this.store = store;
        this.loadCalibration = loadCalibration;
        this.capture = capture;
        this.execute = execute;
        Padding = new Padding(12, 8, 12, 6);

        var layout = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3 };
        layout.RowStyles.Add(new(SizeType.Absolute, 38));
        layout.RowStyles.Add(new(SizeType.Percent, 100));
        layout.RowStyles.Add(new(SizeType.Absolute, 22));
        Controls.Add(layout);
        var header = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1, Margin = Padding.Empty };
        header.RowStyles.Add(new(SizeType.Percent, 100));
        header.ColumnStyles.Add(new(SizeType.Percent, 100));
        header.ColumnStyles.Add(new(SizeType.Absolute, 150));
        header.Controls.Add(new Label { Text = "姿态点位管理", AutoSize = true, Anchor = AnchorStyles.Left,
            ForeColor = UiTheme.Text, Font = UiTheme.Font(12, FontStyle.Bold) }, 0, 0);
        save = Button("保存当前点位", ActionKind.Primary, () => EditPoint(null));
        save.Enabled = false;
        header.Controls.Add(save, 1, 0);
        layout.Controls.Add(header, 0, 0);
        var scroll = new BufferedPanel { Dock = DockStyle.Fill, AutoScroll = true, Margin = Padding.Empty };
        list.ColumnStyles.Add(new(SizeType.Percent, 100));
        scroll.Controls.Add(list);
        layout.Controls.Add(scroll, 0, 1);
        layout.Controls.Add(notice, 0, 2);
        Reload();
    }

    public void Reload() => Run(() =>
    {
        ShowPoints(store.Load());
        notice.Text = "";
    });

    public void SetAvailability(bool canExecute, bool canEdit, bool canDelete)
    {
        if (availability == (canExecute, canEdit, canDelete)) return;
        availability = (canExecute, canEdit, canDelete);
        ApplyAvailability();
    }

    private void ApplyAvailability()
    {
        save.Enabled = availability.Edit;
        foreach (var card in cards)
        {
            card.Execute.Enabled = availability.Execute;
            card.Edit.Enabled = availability.Edit;
            card.Delete.Enabled = availability.Delete;
        }
    }

    private ActionButton Button(string label, ActionKind kind, Action action)
    {
        var button = new ActionButton
        {
            Text = label, Kind = kind, Dock = DockStyle.Fill, Margin = new Padding(3, 4, 3, 4),
            Font = UiTheme.Font(9.5f, FontStyle.Bold),
        };
        button.Click += (_, _) => Run(action);
        return button;
    }

    private void Run(Action action)
    {
        try { action(); }
        catch (Exception e) when (e is ArgumentException or IOException or InvalidDataException or InvalidOperationException or UnauthorizedAccessException)
        {
            notice.Text = e.Message;
        }
    }

    private void ShowPoints(WholeBodyPoint[] points)
    {
        list.SuspendLayout();
        try
        {
            foreach (var child in list.Controls.Cast<System.Windows.Forms.Control>().ToArray()) child.Dispose();
            list.RowStyles.Clear();
            list.RowCount = 0;
            cards.Clear();
            if (!points.Any(p => p.Id == selected)) selected = null;
            if (points.Length == 0)
                list.Controls.Add(new Label { Text = "暂无点位，点击「保存当前点位」添加", AutoSize = true,
                    ForeColor = UiTheme.Muted, Margin = new Padding(6, 12, 4, 4) });
            foreach (var point in points) AddCard(point);
            ApplySelection();
            ApplyAvailability();
        }
        finally { list.ResumeLayout(true); }
    }

    private void AddCard(WholeBodyPoint point)
    {
        int Scale(int pixels) => pixels * DeviceDpi / 96;
        var card = new SurfacePanel { Dock = DockStyle.Fill, Margin = new Padding(1, 3, 5, 3), Padding = new Padding(8, 4, 8, 4) };
        var content = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 4, RowCount = 3, Margin = Padding.Empty };
        content.ColumnStyles.Add(new(SizeType.Absolute, Scale(68)));
        content.ColumnStyles.Add(new(SizeType.Percent, 100));
        content.ColumnStyles.Add(new(SizeType.Absolute, Scale(68)));
        content.ColumnStyles.Add(new(SizeType.Absolute, Scale(68)));
        content.RowStyles.Add(new(SizeType.Absolute, Scale(30)));
        content.RowStyles.Add(new(SizeType.Percent, 50));
        content.RowStyles.Add(new(SizeType.Percent, 50));
        void SelectPoint() { selected = point.Id; ApplySelection(); }
        void AddLine(string value, Color color, int row, FontStyle style = FontStyle.Regular)
        {
            var line = new Label { Text = value, ForeColor = color, Font = UiTheme.Font(9.5f, style),
                Dock = DockStyle.Fill, AutoEllipsis = true, TextAlign = ContentAlignment.MiddleLeft, Margin = Padding.Empty };
            line.Click += (_, _) => SelectPoint();
            content.Controls.Add(line, row == 0 ? 1 : 0, row);
            if (row > 0) content.SetColumnSpan(line, 4);
        }
        AddLine(point.Name, UiTheme.Text, 0, FontStyle.Bold);
        AddLine(string.Join("    ", point.AxisAnglesDeg.Select((angle, i) => $"M{i + 1} {angle:F2}°")), UiTheme.Muted, 1);
        AddLine($"高 {point.HeightMm:F2} mm    前后 {point.ForeAftMm:F2} mm    俯仰 {point.PitchDeg:F2}°    航向 {point.YawDeg:F2}°", UiTheme.Amber, 2);
        card.Click += (_, _) => SelectPoint();
        var run = Button("执行", ActionKind.Primary, () => { SelectPoint(); execute(point); });
        var edit = Button("编辑", ActionKind.SoftOrange, () => { SelectPoint(); EditPoint(point); });
        var delete = Button("删除", ActionKind.SoftDanger, () => { SelectPoint(); DeletePoint(point); });
        foreach (var button in new[] { run, edit, delete }) button.Margin = new Padding(3, 2, 3, 3);
        content.Controls.Add(run, 0, 0);
        content.Controls.Add(edit, 2, 0);
        content.Controls.Add(delete, 3, 0);
        card.Controls.Add(content);
        int row = list.RowCount++;
        list.RowStyles.Add(new(SizeType.Absolute, Scale(76)));
        list.Controls.Add(card, 0, row);
        cards.Add((point, card, run, edit, delete));
    }

    private void ApplySelection()
    {
        foreach (var card in cards)
        {
            bool active = card.Point.Id == selected;
            card.Card.FillColor = active ? Color.FromArgb(47, 42, 34) : Color.FromArgb(34, 43, 57);
            card.Card.StrokeColor = active ? UiTheme.Amber : UiTheme.Border;
            card.Card.Invalidate(true);
        }
    }

    private void EditPoint(WholeBodyPoint? existing)
    {
        var calibration = loadCalibration();
        var workspace = MoveLWorkspace.FromCalibration(calibration);
        var (pose, source) = existing is null ? capture() : (existing.Pose, "修改名称或位姿，前后位置会重新解算");
        using var dialog = new WholeBodyPointDialog(existing?.Name, pose, workspace, source, (name, requested) =>
        {
            var current = loadCalibration();
            var resolved = MotionPlan.ResolveCenteredEndpoint(current, requested);
            double degrees = 180 / Math.PI;
            var point = new WholeBodyPoint(existing?.Id ?? Guid.NewGuid(), name, resolved.Pose.Z * 1000,
                resolved.Pose.Pitch * degrees, resolved.Pose.Yaw * degrees, resolved.Pose.X * 1000,
                resolved.TargetRaw.Select((raw, i) => current.Signs[i] * Od.RawToAngle(raw)).ToArray());
            var points = store.Save(point, editing: existing is not null);
            selected = point.Id;
            ShowPoints(points);
            notice.Text = $"已保存 · {point.Name}";
        });
        dialog.ShowDialog(this);
    }

    private void DeletePoint(WholeBodyPoint point)
    {
        if (MessageBox.Show(this, $"删除点位「{point.Name}」？", "删除点位", MessageBoxButtons.OKCancel,
                MessageBoxIcon.Question, MessageBoxDefaultButton.Button2) != DialogResult.OK)
            return;
        ShowPoints(store.Delete(point.Id));
        notice.Text = $"已删除 · {point.Name}";
    }
}
