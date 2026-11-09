using System.Windows.Forms;
using waist_motor_controller.Protocol;

namespace waist_motor_controller.UI;

public sealed partial class MainForm
{
    private static readonly Color BgColor = UiTheme.Background, CardColor = UiTheme.Surface,
        TileColor = UiTheme.Input, BorderColor = UiTheme.Border, AccentColor = UiTheme.Blue,
        PrimaryColor = UiTheme.Blue, DangerColor = UiTheme.Red, OkColor = UiTheme.Green,
        TextMain = UiTheme.Text, TextSub = UiTheme.Muted;
    private enum BtnKind { Normal, Primary, Success, Warning, Danger, SoftDanger, SoftOrange }
    private readonly SurfacePanel[] _motorCards = new SurfacePanel[4];
    private readonly Label[] _motorCardTitles = new Label[4];
    private readonly Label[] _motorCardAngles = new Label[4];
    private static readonly string[] JointDisplayNames = { "上部旋转", "折叠腿高", "折叠腿中", "折叠腿低" };
    private readonly ToolTip _tips = new() { ShowAlways = true, InitialDelay = 350, ReshowDelay = 100 };
    private Label _motionHint = null!;
    private Button _saveMappingButton = null!;
    private Button _returnAllZeroButton = null!;
    private Button _enableAllButton = null!;
    private readonly Panel[] _pages = new Panel[4];
    private readonly Button[] _pageButtons = new Button[4];
    private int _selectedPage = -1;
    private PageTransition _pageTransition = null!;

    private static Button NewButton(string text, int width = 140, int height = 54, BtnKind kind = BtnKind.Normal)
        => new ActionButton
        {
            Text = text, Width = width, Height = height,
            Kind = kind switch { BtnKind.Primary => ActionKind.Primary, BtnKind.Success => ActionKind.Success,
                BtnKind.Warning => ActionKind.Warning, BtnKind.Danger => ActionKind.Danger,
                BtnKind.SoftDanger => ActionKind.SoftDanger, BtnKind.SoftOrange => ActionKind.SoftOrange, _ => ActionKind.Secondary },
        };

    private static NumericUpDown NewNum(int min, int max, int value, int width)
        => new()
        {
            Minimum = min, Maximum = max, Value = Math.Clamp(value, min, max), Width = width,
            Font = UiTheme.Font(14), BackColor = TileColor, ForeColor = TextMain,
            BorderStyle = BorderStyle.None, TextAlign = HorizontalAlignment.Center,
            Margin = Padding.Empty,
        };

    private Button CreateReturnAllZeroButton()
    {
        var button = NewButton("全部回零位", kind: BtnKind.Primary);
        button.Dock = DockStyle.Fill;
        button.Click += (_, _) => RunMotorOp("全部回零位",
            () => _controller.ReturnAllToZero(JogAccel, PpProfileSpeed));
        _tips.SetToolTip(button, "仅让已通过本程序使能的映射关节回到 0°，未使能关节自动跳过。使用关节控制页的轨迹速度和加速度。");
        return button;
    }

    private ActionButton CreateEmergencyStopButton()
    {
        var button = (ActionButton)NewButton("■  急停 · 4 台电机", kind: BtnKind.Danger);
        button.Dock = DockStyle.Fill;
        button.Click += (_, _) =>
        {
            _wholeBodyPage?.PrepareEmergencyStop();
            RunMotorOp("急停", _controller.EmergencyStop);
        };
        return button;
    }

    private static Label Caption(string text, float size = 10.5F)
        => new() { Text = text, AutoSize = true, ForeColor = TextSub, BackColor = Color.Transparent,
            Font = UiTheme.Font(size), Margin = Padding.Empty };

    private static TableLayoutPanel Grid(int columns, int rows)
    {
        var grid = new BufferedTableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = columns, RowCount = rows,
            Margin = Padding.Empty, Padding = Padding.Empty, BackColor = CardColor };
        for (int i = 0; i < columns; i++) grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f / columns));
        for (int i = 0; i < rows; i++) grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100f / rows));
        return grid;
    }

    private static SurfacePanel NumberShell(NumericUpDown number)
    {
        var shell = new NumberInputShell(number) { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 12, 0) };
        number.Enter += (_, _) => { shell.StrokeColor = AccentColor; shell.Invalidate(); };
        number.Leave += (_, _) => { shell.StrokeColor = BorderColor; shell.Invalidate(); };
        return shell;
    }

    private static Panel NumberField(string title, NumericUpDown number)
    {
        var panel = new BufferedPanel { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 0, 8), BackColor = Color.Transparent };
        var label = Caption(title);
        label.Dock = DockStyle.Top; label.AutoSize = false; label.Height = 27;
        panel.Controls.Add(NumberShell(number));
        panel.Controls.Add(label);
        return panel;
    }

    private sealed class SectionCard : SurfacePanel
    {
        public Panel Header { get; }
        public Panel Content { get; }
        public SectionCard(string title, int height)
        {
            Height = height; Dock = DockStyle.Fill; Margin = new Padding(0, 0, 0, 16);
            Header = new BufferedPanel { Dock = DockStyle.Top, Height = 48, BackColor = Color.Transparent };
            Header.Controls.Add(new Label { Text = title, AutoSize = true, Location = new Point(20, 13),
                Font = UiTheme.Font(13, FontStyle.Bold), ForeColor = TextMain, BackColor = Color.Transparent });
            Content = new BufferedPanel { Dock = DockStyle.Fill, Padding = new Padding(20, 4, 20, 18), BackColor = Color.Transparent };
            Controls.Add(Content); Controls.Add(Header);
        }
    }

    private sealed class Tile : SurfacePanel
    {
        private readonly Label _caption;
        public Label ValueLabel { get; }
        public Tile(string title, string initial = "—")
        {
            Dock = DockStyle.Fill; Margin = new Padding(0, 0, 10, 10);
            FillColor = TileColor; BackColor = TileColor; StrokeColor = Color.FromArgb(38, 55, 78);
            _caption = Caption(title, 10);
            _caption.AutoSize = false;
            _caption.TextAlign = ContentAlignment.MiddleLeft;
            Controls.Add(_caption);
            ValueLabel = new Label { Text = initial, AutoSize = false, AutoEllipsis = true,
                Font = UiTheme.Font(16, FontStyle.Bold), ForeColor = TextMain, BackColor = Color.Transparent,
                TextAlign = ContentAlignment.MiddleLeft };
            Controls.Add(ValueLabel);
        }

        protected override void OnLayout(LayoutEventArgs e)
        {
            base.OnLayout(e);
            if (_caption == null || ValueLabel == null) return;
            float scale = DeviceDpi / 96f;
            int inset = (int)Math.Round(14 * scale), gap = (int)Math.Round(3 * scale);
            int captionHeight = TextRenderer.MeasureText("国Ag", _caption.Font, Size.Empty, TextFormatFlags.SingleLine).Height;
            int valueHeight = TextRenderer.MeasureText("国Ag", ValueLabel.Font, Size.Empty, TextFormatFlags.SingleLine).Height;
            int top = Math.Max(0, (ClientSize.Height - captionHeight - gap - valueHeight) / 2);
            int width = Math.Max(0, ClientSize.Width - inset * 2);
            _caption.SetBounds(inset, top, width, captionHeight);
            ValueLabel.SetBounds(inset, top + captionHeight + gap, width, valueHeight);
        }
    }

    private void BuildUi()
    {
        _motorCombo = new JointComboBox { DropDownStyle = ComboBoxStyle.DropDownList, FlatStyle = FlatStyle.Flat,
            Font = UiTheme.Font(11.5F, FontStyle.Bold), BackColor = TileColor, ForeColor = TextMain,
            Dock = DockStyle.Fill, Margin = new Padding(0, 3, 0, 0) };
        for (int i = 0; i < _registry.Motors.Count; i++) _motorCombo.Items.Add(MotorDisplayName(i));
        _motorCombo.DrawMode = DrawMode.OwnerDrawFixed;
        _motorCombo.SelectedIndex = 0;

        var root = Grid(1, 2);
        root.BackColor = BgColor; root.Padding = new Padding(20, 18, 20, 18);
        root.RowStyles.Clear();
        root.RowStyles.Add(new RowStyle(SizeType.Absolute, 48));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        var menu = new NavigationStrip();
        string[] titles = { "通信连接", "关节控制", "全身运控", "标定设备" };
        for (int i = 0; i < titles.Length; i++)
        {
            int index = i;
            var button = new NavigationButton { Text = titles[i], Margin = Padding.Empty, Dock = DockStyle.None };
            button.Click += (_, _) => SelectPage(index);
            _pageButtons[i] = button; menu.Controls.Add(button);
        }
        root.Controls.Add(menu, 0, 0);
        var host = new BufferedPanel { Dock = DockStyle.Fill, BackColor = BgColor, Margin = Padding.Empty };
        root.Controls.Add(host, 0, 1);

        var connection = Grid(1, 2); connection.BackColor = BgColor;
        connection.RowStyles.Clear();
        connection.RowStyles.Add(new RowStyle(SizeType.Absolute, 262));
        connection.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        connection.Controls.Add(BuildConnSection(), 0, 0);
        var logHost = new BufferedPanel { Dock = DockStyle.Fill, BackColor = BgColor, Margin = Padding.Empty };
        BuildLogSection(logHost);
        connection.Controls.Add(logHost, 0, 1);
        _pages[0] = CreatePage(connection, 438);

        var workspace = Grid(2, 1);
        workspace.ColumnStyles.Clear();
        workspace.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 304));
        workspace.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        workspace.BackColor = BgColor;
        var motors = BuildMappingSection(); motors.Margin = new Padding(0, 0, 16, 0);
        workspace.Controls.Add(motors, 0, 0);
        var control = BuildMotorSection(); control.Margin = Padding.Empty;
        workspace.Controls.Add(control, 1, 0);
        _pages[1] = CreatePage(workspace, 816);
        try
        {
            _wholeBodyPage = new waist_motor_controller.WholeBody.WholeBodyPage(_transport, _registry, _controller,
                () => new Dictionary<int, string>(_firmware), OnLogFromThread);
            _pages[2] = new BufferedPanel { Dock = DockStyle.Fill, BackColor = BgColor };
            _pages[2].Controls.Add(_wholeBodyPage);
        }
        catch (Exception ex)
        {
            _pages[2] = new Panel { Dock = DockStyle.Fill };
            _pages[2].Controls.Add(new Label { Dock = DockStyle.Fill, Text = "Whole body model unavailable: " + ex.Message, ForeColor = TextMain });
        }
        var calibrationPage = new waist_motor_controller.WholeBody.CalibrationPage(new WaistModel.CalibrationStore(),
            () => _wholeBodyPage?.Busy == true || _registry.Motors.Any(m => _controller.IsJogging(m.DeviceId) || _controller.IsBusy(m.DeviceId)),
            () => _registry.Motors.Select(m => m.Direction).ToArray());
        calibrationPage.Saved += () => _wholeBodyPage?.CalibrationChanged();
        _pages[3] = new BufferedPanel { Dock = DockStyle.Fill, AutoScroll = true }; _pages[3].Controls.Add(calibrationPage);
        foreach (var page in _pages) { page.Visible = false; host.Controls.Add(page); }
        _pageTransition = new PageTransition(); host.Controls.Add(_pageTransition);
        Controls.Add(root);
        SelectPage(0);
        _motorCombo.SelectedIndexChanged += (_, _) =>
        {
            UpdateMotorSelection();
            UpdateDirectionUi();
            if (_uiTimer != null) RefreshStatus();
        };
        UpdateMotorSelection();
    }

    private Panel CreatePage(TableLayoutPanel layout, int minimumHeight)
    {
        var page = new BufferedPanel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = BgColor, Margin = Padding.Empty };
        layout.Dock = DockStyle.Top; layout.Height = minimumHeight;
        page.Controls.Add(layout);
        void ResizeContent()
        {
            int height = Math.Max((int)Math.Ceiling(minimumHeight * DeviceDpi / 96d), page.ClientSize.Height);
            if (layout.Height != height) layout.Height = height;
            page.AutoScrollMinSize = new Size(0, height);
        }
        page.SizeChanged += (_, _) => ResizeContent();
        page.VisibleChanged += (_, _) => { if (page.Visible) ResizeContent(); };
        return page;
    }

    private void SelectPage(int index)
    {
        if ((uint)index >= _pages.Length) throw new ArgumentOutOfRangeException(nameof(index));
        if (_selectedPage == index) return;
        var before = _pageTransition.TakeSnapshot();
        SuspendLayout();
        for (int i = 0; i < _pages.Length; i++)
        {
            _pages[i].Visible = i == index;
            ((NavigationButton)_pageButtons[i]).Selected = i == index;
            _tips.SetToolTip(_pageButtons[i], i == index ? "当前页面" : "切换页面");
        }
        _pages[index].BringToFront();
        _selectedPage = index;
        ResumeLayout(true);
        _pageTransition.Start(before);
    }

    private void UpdateMotorSelection()
    {
        for (int i = 0; i < _motorCards.Length; i++)
        {
            if (_motorCards[i] == null) continue;
            bool selected = i == CurrentLogicalIndex;
            Color fill = selected ? Color.FromArgb(27, 53, 83) : CardColor;
            Color stroke = selected ? AccentColor : BorderColor;
            if (_motorCards[i].FillColor == fill && _motorCards[i].StrokeColor == stroke) continue;
            _motorCards[i].FillColor = fill;
            _motorCards[i].BackColor = fill;
            _motorCards[i].StrokeColor = stroke;
            _motorCards[i].Invalidate();
        }
    }

    private SectionCard BuildConnSection()
    {
        var card = new SectionCard("通信连接", 246);
        var grid = Grid(4, 3);
        grid.RowStyles.Clear();
        var connection = new Tile("连接状态", "● 未连接");
        _connStateLabel = connection.ValueLabel; _connStateLabel.Font = UiTheme.Font(13, FontStyle.Bold);
        var nodes = new Tile("在线电机");
        _busIdLabel = nodes.ValueLabel; _busIdLabel.Font = UiTheme.Font(12, FontStyle.Bold);
        var traffic = new Tile("报文计数 RX/TX");
        _rxTxLabel = traffic.ValueLabel; _rxTxLabel.Font = UiTheme.Font(11.5F, FontStyle.Bold);
        var speed = new Tile("通信速率", "1M / 5M · CANFD");
        speed.ValueLabel.Font = UiTheme.Font(10.5F, FontStyle.Bold);
        var adapter = new Tile("适配器信息", "USBCANFD-200U  ·  标准帧 · BRS  ·  终端电阻：开启");
        adapter.ValueLabel.Font = UiTheme.Font(10.5F, FontStyle.Bold);
        grid.Controls.Add(connection, 0, 0); grid.Controls.Add(nodes, 1, 0);
        grid.Controls.Add(traffic, 2, 0); grid.Controls.Add(speed, 3, 0);
        grid.Controls.Add(adapter, 0, 1); grid.SetColumnSpan(adapter, 4);

        var operations = Grid(2, 1);
        _openCloseButton = NewButton("打开设备", 140, 52, BtnKind.Primary);
        _openCloseButton.Click += (_, _) => OnOpenClose();
        _changeIdButton = NewButton("更改电机 ID", 150, 52);
        _changeIdButton.Click += (_, _) => OnChangeId();
        var connectionButtons = new[] { _openCloseButton, _changeIdButton };
        for (int i = 0; i < connectionButtons.Length; i++) { connectionButtons[i].Dock = DockStyle.Fill; operations.Controls.Add(connectionButtons[i], i, 0); }
        grid.Controls.Add(operations, 0, 2); grid.SetColumnSpan(operations, 4);
        grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 60));
        grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 52));
        card.Content.Controls.Add(grid);
        return card;
    }

    private string MotorDisplayName(int index)
        => $"{_registry.Motors[index].DeviceId}号电机-{JointDisplayNames[index]}";

    private void RefreshMotorDisplayNames()
    {
        int selectedIndex = _motorCombo.SelectedIndex;
        for (int i = 0; i < _motorCardTitles.Length; i++)
        {
            string name = MotorDisplayName(i);
            _motorCardTitles[i].Text = name;
            if (!Equals(_motorCombo.Items[i], name)) _motorCombo.Items[i] = name;
        }
        _motorCombo.SelectedIndex = selectedIndex;
        _motorCombo.Invalidate();
    }

    private SectionCard BuildMappingSection()
    {
        var card = new SectionCard("关节选择", 816);
        var content = Grid(1, 10);
        content.RowStyles.Clear();
        for (int i = 0; i < 4; i++)
        {
            int index = i;
            var motor = _registry.Motors[i];
            var tile = new SurfacePanel { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 0, 10),
                Padding = new Padding(13, 10, 13, 10), Cursor = Cursors.Hand };
            _motorCards[i] = tile;
            var header = Grid(2, 1);
            header.Dock = DockStyle.Top;
            header.Height = 31;
            header.BackColor = Color.Transparent;
            header.ColumnStyles[1].SizeType = SizeType.Absolute;
            header.ColumnStyles[1].Width = 70;
            var name = new Label { Text = MotorDisplayName(i), ForeColor = TextMain,
                Font = UiTheme.Font(11.5F, FontStyle.Bold), Dock = DockStyle.Fill, Margin = Padding.Empty,
                BackColor = Color.Transparent, AutoEllipsis = true, TextAlign = ContentAlignment.MiddleLeft };
            _motorCardTitles[i] = name;
            var angle = new Label { Text = "—", ForeColor = TextSub, Font = UiTheme.Font(10.5F, FontStyle.Bold),
                BackColor = Color.Transparent, Dock = DockStyle.Fill, Margin = Padding.Empty, TextAlign = ContentAlignment.MiddleRight };
            _motorCardAngles[i] = angle;
            header.Controls.Add(name, 0, 0);
            header.Controls.Add(angle, 1, 0);
            var bottom = new BufferedFlowLayoutPanel { Dock = DockStyle.Bottom, Height = 31, WrapContents = false, Margin = Padding.Empty, BackColor = Color.Transparent };
            var id = Caption("ID"); id.Anchor = AnchorStyles.Left; id.Margin = new Padding(0, 0, 8, 0);
            _idNums[i] = NewNum(1, 127, motor.DeviceId, 58);
            _idNums[i].Font = UiTheme.Font(12);
            _onlineLabels[i] = Caption("● 离线", 10.5F);
            _onlineLabels[i].Anchor = AnchorStyles.Left;
            _onlineLabels[i].Margin = new Padding(18, 0, 0, 0);
            bottom.Controls.AddRange(new System.Windows.Forms.Control[] { id, _idNums[i], _onlineLabels[i] });
            tile.Controls.Add(bottom); tile.Controls.Add(header);
            tile.Click += (_, _) => _motorCombo.SelectedIndex = index;
            header.Click += (_, _) => _motorCombo.SelectedIndex = index;
            name.Click += (_, _) => _motorCombo.SelectedIndex = index;
            angle.Click += (_, _) => _motorCombo.SelectedIndex = index;
            // Child controls do not bubble Click to the card, including blank space in this row.
            bottom.Click += (_, _) => _motorCombo.SelectedIndex = index;
            id.Click += (_, _) => _motorCombo.SelectedIndex = index;
            _onlineLabels[i].Click += (_, _) => _motorCombo.SelectedIndex = index;
            tile.MouseEnter += (_, _) => { if (index != CurrentLogicalIndex) { tile.StrokeColor = AccentColor; tile.Invalidate(); } };
            tile.MouseLeave += (_, _) => UpdateMotorSelection();
            _tips.SetToolTip(tile, $"{motor.Model} · 点击选择此关节");
            content.Controls.Add(tile, 0, i);
            content.RowStyles.Add(new RowStyle(SizeType.Absolute, 93));
        }
        var save = NewButton("保存映射与参数", 210, 52);
        _saveMappingButton = save;
        save.Dock = DockStyle.Fill; save.Margin = new Padding(0, 4, 0, 6);
        save.Click += (_, _) => OnSaveMapping();
        content.Controls.Add(save, 0, 4); content.RowStyles.Add(new RowStyle(SizeType.Absolute, 62));
        var version = new SurfacePanel { Dock = DockStyle.Fill, FillColor = TileColor, Padding = new Padding(12), Margin = Padding.Empty };
        var versionTitle = Caption("型号 · 版本", 9.5F);
        versionTitle.Dock = DockStyle.Top; versionTitle.Height = 24; versionTitle.AutoSize = false;
        _firmwareLabel = new Label { Text = $"{_registry.Motors[0].Model} · —", Dock = DockStyle.Fill, ForeColor = TextMain, BackColor = Color.Transparent, Font = UiTheme.Font(10.5F), TextAlign = ContentAlignment.MiddleLeft };
        version.Controls.Add(_firmwareLabel);
        version.Controls.Add(versionTitle);
        content.Controls.Add(version, 0, 5); content.RowStyles.Add(new RowStyle(SizeType.Absolute, 88));
        _returnAllZeroButton = CreateReturnAllZeroButton();
        _returnAllZeroButton.Margin = new Padding(0, 16, 0, 0);
        content.Controls.Add(_returnAllZeroButton, 0, 6); content.RowStyles.Add(new RowStyle(SizeType.Absolute, 72));
        _enableAllButton = NewButton("全部使能", kind: BtnKind.Primary);
        _enableAllButton.Dock = DockStyle.Fill; _enableAllButton.Margin = new Padding(0, 10, 0, 0);
        _enableAllButton.Click += (_, _) => RunMotorOp("全部使能", () => _controller.EnableAll(JogAccel, PpProfileSpeed));
        content.Controls.Add(_enableAllButton, 0, 7); content.RowStyles.Add(new RowStyle(SizeType.Absolute, 66));
        _eStopButton = CreateEmergencyStopButton();
        _eStopButton.Margin = new Padding(0, 10, 0, 0);
        content.Controls.Add(_eStopButton, 0, 8); content.RowStyles.Add(new RowStyle(SizeType.Absolute, 66));
        content.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        card.Content.Controls.Add(content);
        return card;
    }

    private SectionCard BuildMotorSection()
    {
        var card = new SectionCard("单关节控制", 684);
        var grid = Grid(1, 8);
        grid.RowStyles.Clear();
        var select = Grid(4, 1);
        select.MaximumSize = new Size(465, 0);
        select.ColumnStyles.Clear();
        select.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        select.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
        select.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
        select.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 76));
        var joint = Caption("关节选择：", 10);
        joint.Anchor = AnchorStyles.Left;
        joint.Margin = new Padding(0, 0, 8, 8);
        select.Controls.Add(joint, 0, 0);
        _motorCombo.Margin = new Padding(0, 3, 12, 0);
        select.Controls.Add(_motorCombo, 1, 0);
        var direction = Caption("方向系数：", 10);
        direction.Anchor = AnchorStyles.Left;
        direction.Margin = new Padding(0, 0, 8, 8);
        select.Controls.Add(direction, 2, 0);
        _directionCombo = new JointComboBox { AccessibleName = "方向系数", Dock = DockStyle.Fill,
            Font = UiTheme.Font(13, FontStyle.Bold), Margin = new Padding(0, 3, 8, 0) };
        _directionCombo.Items.AddRange(new object[] { "+", "−" });
        _directionCombo.SelectionChangeCommitted += (_, _) =>
        {
            if (!_transport.IsOpen) _pendingDirections[CurrentLogicalIndex] = _directionCombo.SelectedIndex == 0 ? 1 : -1;
        };
        _tips.SetToolTip(_directionCombo, "关闭设备后可修改方向系数，点击“保存映射与参数”生效。");
        select.Controls.Add(_directionCombo, 3, 0);
        grid.Controls.Add(select, 0, 0); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 60));

        var metrics = Grid(3, 2);
        var pos = new Tile("当前位置 · 关节角"); _posLabel = pos.ValueLabel; _posLabel.ForeColor = UiTheme.ValueBlue;
        var velocity = new Tile("当前速度 · 输入端"); _velLabel = velocity.ValueLabel;
        var current = new Tile("电流"); _curLabel = current.ValueLabel;
        var mode = new Tile("控制模式"); _modeLabel = mode.ValueLabel; _modeLabel.Font = UiTheme.Font(13, FontStyle.Bold);
        var temperature = new Tile("线圈温度"); _tempLabel = temperature.ValueLabel;
        var fault = new Tile("故障状态"); _faultLabel = fault.ValueLabel; _faultLabel.Font = UiTheme.Font(11.5F, FontStyle.Bold);
        metrics.Controls.Add(pos, 0, 0); metrics.Controls.Add(velocity, 1, 0); metrics.Controls.Add(current, 2, 0);
        metrics.Controls.Add(mode, 0, 1); metrics.Controls.Add(temperature, 1, 1); metrics.Controls.Add(fault, 2, 1);

        grid.Controls.Add(metrics, 0, 1); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 172));

        var states = new BufferedFlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false, Margin = Padding.Empty };
        string[] names = { "使能", "抱闸释放", "故障", "到位估算" };
        for (int i = 0; i < names.Length; i++)
        {
            _stateLights[i] = Caption("● " + names[i], 10.5F);
            _stateLights[i].Margin = new Padding(0, 0, 22, 0);
            states.Controls.Add(_stateLights[i]);
        }
        grid.Controls.Add(states, 0, 2); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));

        var ops = Grid(4, 1);
        ops.ColumnStyles.Clear();
        for (int i = 0; i < 4; i++) ops.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 25));
        _enableButton = NewButton("使能", kind: BtnKind.Success);
        _enableButton.Click += (_, _) => RunMotorOp("使能", () => _controller.Enable(CurrentDeviceId, JogAccel, PpProfileSpeed));
        _disableButton = NewButton("失能", kind: BtnKind.SoftDanger);
        _disableButton.Click += (_, _) => RunMotorOp("失能", () => _controller.Disable(CurrentDeviceId));
        _clearErrButton = NewButton("清除故障", kind: BtnKind.SoftOrange);
        _clearErrButton.Click += (_, _) => RunMotorOp("清错", () => _controller.ClearError(CurrentDeviceId));
        _setZeroButton = NewButton("当前位置设零", kind: BtnKind.SoftOrange);
        _setZeroButton.Click += async (_, _) => await OnSetZero();
        var buttons = new[] { _enableButton, _disableButton, _clearErrButton, _setZeroButton };
        foreach (var button in buttons) ((ActionButton)button).WhiteText = true;
        for (int i = 0; i < buttons.Length; i++) { buttons[i].Dock = DockStyle.Fill; buttons[i].Margin = new Padding(0, 0, 10, 8); ops.Controls.Add(buttons[i], i, 0); }
        grid.Controls.Add(ops, 0, 3); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 66));

        var parameters = Grid(3, 1);
        _targetAngleNum = NewNum(-179, 179, 0, 140); _targetAngleNum.DecimalPlaces = 2;
        _targetAngleNum.Minimum = -178.90M; _targetAngleNum.Maximum = 178.90M; _targetAngleNum.Increment = 0.5M;
        _jogSpeedNum = NewNum(0, 5, 1, 130); _jogSpeedNum.DecimalPlaces = 2; _jogSpeedNum.Minimum = 0.05M; _jogSpeedNum.Increment = 0.05M;
        _jogAccelNum = NewNum(0, 50, 5, 130); _jogAccelNum.DecimalPlaces = 2; _jogAccelNum.Minimum = 0.1M; _jogAccelNum.Increment = 0.1M;
        parameters.Controls.Add(NumberField("绝对目标角度 · °", _targetAngleNum), 0, 0);
        parameters.Controls.Add(NumberField("轨迹速度 · rpm", _jogSpeedNum), 1, 0);
        parameters.Controls.Add(NumberField("轨迹加速度 · rpm/s", _jogAccelNum), 2, 0);
        grid.Controls.Add(parameters, 0, 4); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 82));

        var absolute = Grid(2, 1);
        _moveButton = NewButton("运行到指定角度", kind: BtnKind.Primary);
        _moveButton.Dock = DockStyle.Fill; _moveButton.Margin = new Padding(0, 0, 12, 8);
        _moveButton.Click += (_, _) => RunMotionOp("角度定位", () => _controller.MoveToJointPosition(CurrentDeviceId, (double)_targetAngleNum.Value, JogAccel, PpProfileSpeed));
        _returnZeroButton = NewButton("运行回零位");
        _returnZeroButton.Dock = DockStyle.Fill; _returnZeroButton.Margin = new Padding(0, 0, 12, 8);
        _returnZeroButton.Click += (_, _) => RunMotionOp("当前关节回零", () => _controller.MoveToPosition(CurrentDeviceId, 0, JogAccel, PpProfileSpeed));
        _tips.SetToolTip(_returnZeroButton, "当前选中关节运行到 0°，使用当前轨迹速度与加速度。");
        absolute.Controls.Add(_moveButton, 0, 0);
        absolute.Controls.Add(_returnZeroButton, 1, 0);
        grid.Controls.Add(absolute, 0, 5); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 66));

        var jog = Grid(4, 1);
        jog.ColumnStyles.Clear();
        foreach (float width in new[] { 22F, 22F, 22F, 34F }) jog.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, width));
        _jogStepNum = NewNum(0, 30, 1, 110); _jogStepNum.DecimalPlaces = 2; _jogStepNum.Minimum = 0.01M; _jogStepNum.Increment = 0.1M;
        jog.Controls.Add(NumberField("点动增量 · °", _jogStepNum), 0, 0);
        _jogRevButton = NewButton("−  点动"); _jogFwdButton = NewButton("+  点动");
        _stopMotionButton = NewButton("■  停止 / 保持", kind: BtnKind.Warning);
        _jogRevButton.Click += (_, _) => RunMotionOp("负向增量", () => _controller.MoveJointIncrement(CurrentDeviceId, -(double)_jogStepNum.Value, JogAccel, PpProfileSpeed));
        _jogFwdButton.Click += (_, _) => RunMotionOp("正向增量", () => _controller.MoveJointIncrement(CurrentDeviceId, (double)_jogStepNum.Value, JogAccel, PpProfileSpeed));
        _stopMotionButton.Click += (_, _) => RunMotorOp("停止", () => _controller.StopMotion(CurrentDeviceId));
        var jogButtons = new[] { _jogRevButton, _jogFwdButton, _stopMotionButton };
        for (int i = 0; i < 3; i++) { jogButtons[i].Dock = DockStyle.Fill; jogButtons[i].Margin = new Padding(0, 27, i == 2 ? 0 : 10, 8); jog.Controls.Add(jogButtons[i], i + 1, 0); }
        grid.Controls.Add(jog, 0, 6); grid.RowStyles.Add(new RowStyle(SizeType.Absolute, 84));

        _motionHint = Caption("连接设备并选择电机后，可开始操作。", 10);
        _motionHint.AutoSize = false; _motionHint.Dock = DockStyle.Fill; _motionHint.Padding = new Padding(0, 4, 0, 0);
        grid.Controls.Add(_motionHint, 0, 7); grid.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        card.Content.Controls.Add(grid);
        _tips.SetToolTip(_setZeroButton, "将当前位置重新定义为零位，请先确认机械位置。");
        _tips.SetToolTip(_stopMotionButton, "停止生成运动轨迹，并保持最后指令位置。");
        return card;
    }

    private void BuildLogSection(Panel host)
    {
        var card = new SectionCard("通信日志", 48) { Dock = DockStyle.Top, Margin = Padding.Empty };
        var toggle = NewButton("显示日志  ▾", 108, 34);
        toggle.Name = "ToggleLogButton";
        toggle.Location = new Point(112, 7);
        toggle.Font = UiTheme.Font(10.5F, FontStyle.Bold);
        var clear = NewButton("清空日志", 100, 34);
        clear.Name = "ClearLogButton";
        clear.Location = new Point(228, 7);
        clear.Font = toggle.Font;
        card.Header.Controls.AddRange(new System.Windows.Forms.Control[] { toggle, clear });

        _logBox = new TextBox { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, WordWrap = false,
            Dock = DockStyle.Fill, Font = new Font("Consolas", 10), BackColor = BgColor, ForeColor = TextSub,
            BorderStyle = BorderStyle.None, Margin = new Padding(12) };
        var logSurface = new SurfacePanel { Dock = DockStyle.Fill, FillColor = BgColor, BackColor = BgColor,
            Padding = new Padding(12), Margin = Padding.Empty };
        logSurface.Controls.Add(_logBox);
        card.Content.Controls.Add(logSurface);
        card.Content.Visible = false;
        clear.Click += (_, _) => _logBox.Clear();

        bool expanded = false;
        double reveal = 0, start = 0;
        var clock = new System.Diagnostics.Stopwatch();
        var animation = new System.Windows.Forms.Timer { Interval = 15 };
        void ResizeCard()
        {
            int headerHeight = card.Header.Height;
            card.Height = headerHeight + (int)Math.Round(reveal * Math.Max(0, host.ClientSize.Height - headerHeight));
        }
        animation.Tick += (_, _) =>
        {
            double progress = Math.Clamp(clock.Elapsed.TotalMilliseconds / 200, 0, 1);
            double eased = progress * progress * (3 - 2 * progress);
            reveal = start + ((expanded ? 1 : 0) - start) * eased;
            ResizeCard();
            if (progress < 1) return;
            animation.Stop();
            card.Content.Visible = expanded;
        };
        toggle.Click += (_, _) =>
        {
            expanded = !expanded;
            toggle.Text = expanded ? "收起日志  ▴" : "显示日志  ▾";
            if (expanded) card.Content.Visible = true;
            start = reveal;
            clock.Restart();
            animation.Start();
        };
        host.Controls.Add(card);
        host.SizeChanged += (_, _) => ResizeCard();
        card.Header.SizeChanged += (_, _) => ResizeCard();
        card.Disposed += (_, _) => animation.Dispose();
    }
}
