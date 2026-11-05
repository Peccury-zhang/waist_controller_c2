namespace waist_motor_controller.UI;

public sealed class ChangeMotorIdDialog : Form
{
    private readonly NumericUpDown _currentNum = new(), _newNum = new();
    private readonly Label _conflictLabel = new();
    private readonly HashSet<int> _usedIds;
    private readonly ActionButton _ok = new() { Text = "写入新 ID", Kind = ActionKind.Primary };
    public int CurrentId => (int)_currentNum.Value;
    public int NewId => (int)_newNum.Value;

    public ChangeMotorIdDialog(int defaultCurrentId, HashSet<int> usedIds)
    {
        _usedIds = usedIds;
        SuspendLayout();
        AutoScaleDimensions = new SizeF(96, 96); AutoScaleMode = AutoScaleMode.Dpi;
        Text = "更改电机 CAN ID"; FormBorderStyle = FormBorderStyle.FixedDialog;
        MaximizeBox = MinimizeBox = false; StartPosition = FormStartPosition.CenterParent;
        ClientSize = new Size(500, 350); Font = UiTheme.Font(11); BackColor = UiTheme.Background; ForeColor = UiTheme.Text;
        var card = new SurfacePanel { Dock = DockStyle.Fill, Padding = new Padding(24), Margin = new Padding(20) };
        var grid = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 5, BackColor = UiTheme.Surface };
        grid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 145)); grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        var heading = new Label { Text = "修改总线节点地址", AutoSize = true, ForeColor = UiTheme.Text, Font = UiTheme.Font(15, FontStyle.Bold), Margin = new Padding(0, 0, 0, 16) };
        grid.Controls.Add(heading, 0, 0); grid.SetColumnSpan(heading, 2); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        NumericUpDown[] numbers = { _currentNum, _newNum };
        string[] titles = { "当前 CAN ID", "新的 CAN ID" };
        for (int i = 0; i < 2; i++)
        {
            grid.Controls.Add(new Label { Text = titles[i], AutoSize = true, ForeColor = UiTheme.Muted, Anchor = AnchorStyles.Left }, 0, i + 1);
            var n = numbers[i]; n.Minimum = 1; n.Maximum = 127; n.Value = Math.Clamp(defaultCurrentId, 1, 127);
            n.Font = UiTheme.Font(17); n.ForeColor = UiTheme.Text; n.BackColor = UiTheme.Input; n.TextAlign = HorizontalAlignment.Center;
            n.Dock = DockStyle.Fill; n.Margin = new Padding(0, 8, 0, 8); n.BorderStyle = BorderStyle.FixedSingle;
            n.ValueChanged += (_, _) => UpdateConflictHint();
            grid.Controls.Add(n, 1, i + 1); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
        }
        _conflictLabel.Dock = DockStyle.Fill; _conflictLabel.ForeColor = UiTheme.Amber; _conflictLabel.Font = UiTheme.Font(10);
        _conflictLabel.Padding = new Padding(0, 8, 0, 0);
        grid.Controls.Add(_conflictLabel, 0, 3); grid.SetColumnSpan(_conflictLabel, 2); grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var buttons = new TableLayoutPanel { ColumnCount = 2, Dock = DockStyle.Fill, Margin = Padding.Empty };
        buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50)); buttons.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50));
        var cancel = new ActionButton { Text = "取消", DialogResult = DialogResult.Cancel, Dock = DockStyle.Fill, Margin = new Padding(0, 0, 12, 0) };
        _ok.DialogResult = DialogResult.OK; _ok.Dock = DockStyle.Fill; _ok.Margin = Padding.Empty;
        buttons.Controls.Add(cancel, 0, 0); buttons.Controls.Add(_ok, 1, 0);
        grid.Controls.Add(buttons, 0, 4); grid.SetColumnSpan(buttons, 2); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
        card.Controls.Add(grid);
        var padding = new Panel { Dock = DockStyle.Fill, Padding = new Padding(20) }; padding.Controls.Add(card); Controls.Add(padding);
        AcceptButton = _ok; CancelButton = cancel; ResumeLayout(true); UpdateConflictHint();
    }
    private void UpdateConflictHint()
    {
        bool conflict = _usedIds.Contains(NewId) && NewId != CurrentId;
        bool same = NewId == CurrentId;
        _ok.Enabled = !conflict && !same;
        _conflictLabel.ForeColor = conflict ? UiTheme.Red : UiTheme.Muted;
        _conflictLabel.Text = conflict ? $"ID {NewId} 已被其他电机使用，请更换。" : same ? "请输入一个与当前 ID 不同的新地址。" : "写入成功后，需电机断电重启才会生效。";
    }
}
