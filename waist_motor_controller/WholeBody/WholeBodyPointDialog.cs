using WaistModel;
using waist_motor_controller.UI;

namespace waist_motor_controller.WholeBody;

internal sealed class WholeBodyPointDialog : Form
{
    public WholeBodyPointDialog(string? pointName, TaskPose pose, MoveLWorkspace workspace, string source,
        Action<string, TaskPose> save)
    {
        AutoScaleDimensions = new(96, 96);
        AutoScaleMode = AutoScaleMode.Dpi;
        Text = pointName is null ? "保存当前点位" : "编辑点位";
        ClientSize = new(435, 295);
        StartPosition = FormStartPosition.CenterParent;
        FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = MinimizeBox = false;
        ShowInTaskbar = false;
        BackColor = UiTheme.Surface;
        ForeColor = UiTheme.Text;
        Font = UiTheme.Font(10);

        var fields = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 7, Padding = new Padding(16) };
        fields.ColumnStyles.Add(new(SizeType.Absolute, 96));
        fields.ColumnStyles.Add(new(SizeType.Percent, 100));
        fields.RowStyles.Add(new(SizeType.Absolute, 36));
        for (int i = 0; i < 4; i++) fields.RowStyles.Add(new(SizeType.Absolute, 34));
        fields.RowStyles.Add(new(SizeType.Percent, 100));
        fields.RowStyles.Add(new(SizeType.Absolute, 38));
        Controls.Add(fields);

        var description = new Label { Text = source, Dock = DockStyle.Fill, ForeColor = UiTheme.Muted };
        fields.Controls.Add(description, 0, 0);
        fields.SetColumnSpan(description, 2);
        var name = new TextBox { Text = pointName ?? "", MaxLength = 40, Dock = DockStyle.Top,
            BackColor = UiTheme.Input, ForeColor = UiTheme.Text };
        var height = Number((decimal)workspace.MinimumHeightMm, (decimal)workspace.MaximumHeightMm, pose.Z * 1000);
        var pitch = Number(-180, 180, pose.Pitch * 180 / Math.PI);
        var yaw = Number(-180, 180, pose.Yaw * 180 / Math.PI);
        AddField("名称", name, 1);
        AddField("高度 mm", height, 2);
        AddField("俯仰 °", pitch, 3);
        AddField("航向 °", yaw, 4);

        var error = new Label { Text = "前后位置与四轴角度在保存时自动解算。", ForeColor = UiTheme.Muted, Dock = DockStyle.Fill };
        fields.Controls.Add(error, 0, 5);
        fields.SetColumnSpan(error, 2);
        var buttons = new FlowLayoutPanel { Dock = DockStyle.Fill, FlowDirection = FlowDirection.RightToLeft, Margin = Padding.Empty };
        var confirm = new ActionButton { Text = "保存", Kind = ActionKind.Primary, Width = 92, Height = 32 };
        var cancel = new ActionButton { Text = "取消", DialogResult = DialogResult.Cancel, Width = 92, Height = 32 };
        buttons.Controls.AddRange([confirm, cancel]);
        fields.Controls.Add(buttons, 0, 6);
        fields.SetColumnSpan(buttons, 2);
        AcceptButton = confirm;
        CancelButton = cancel;
        ActiveControl = name;
        name.SelectAll();
        confirm.Click += (_, _) =>
        {
            try
            {
                string value = name.Text.Trim();
                if (value.Length == 0) throw new ArgumentException("请输入点位名称");
                save(value, new TaskPose(0, (double)height.Value / 1000,
                    (double)pitch.Value * Math.PI / 180, (double)yaw.Value * Math.PI / 180));
                DialogResult = DialogResult.OK;
            }
            catch (Exception e) when (e is ArgumentException or IOException or InvalidDataException or InvalidOperationException or UnauthorizedAccessException)
            {
                error.ForeColor = UiTheme.Amber;
                error.Text = e.Message;
            }
        };

        void AddField(string caption, System.Windows.Forms.Control input, int row)
        {
            fields.Controls.Add(new Label { Text = caption, AutoSize = true, Anchor = AnchorStyles.Left }, 0, row);
            fields.Controls.Add(input, 1, row);
        }
    }

    private static NumericUpDown Number(decimal minimum, decimal maximum, double value) => new()
    {
        Minimum = minimum, Maximum = maximum, Value = Math.Clamp((decimal)value, minimum, maximum),
        DecimalPlaces = 2, Dock = DockStyle.Top, BackColor = UiTheme.Input, ForeColor = UiTheme.Text,
    };
}
