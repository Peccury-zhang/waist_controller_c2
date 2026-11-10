using System.Globalization;
using WaistModel;
using waist_motor_controller.Control;
namespace waist_motor_controller.WholeBody;
public sealed class CalibrationPage:UserControl
{
    readonly CalibrationStore store;readonly Func<bool> busy;readonly Func<int[]> signs;readonly TextBox[,] values=new TextBox[4,2];
    readonly NumericUpDown forward=new(){Minimum=0,Maximum=350,Value=350,DecimalPlaces=1},backward=new(){Minimum=0,Maximum=500,Value=500,DecimalPlaces=1};
    readonly Label status=new(){AutoSize=true};public event Action? Saved;
    public CalibrationPage(CalibrationStore store, MotorRegistry registry, Func<bool> busy)
    {
        this.store=store;this.busy=busy;signs=()=>registry.WaistMotors.Select(m=>m.Direction).ToArray();Dock=DockStyle.Fill;AutoScroll=true;BackColor=Color.FromArgb(24,29,37);ForeColor=Color.Gainsboro;
        var layout=new TableLayoutPanel{Dock=DockStyle.Top,AutoSize=true,ColumnCount=3,Padding=new(24)};layout.ColumnStyles.Add(new(SizeType.Percent,40));layout.ColumnStyles.Add(new(SizeType.Percent,30));layout.ColumnStyles.Add(new(SizeType.Percent,30));Controls.Add(layout);
        void Row(System.Windows.Forms.Control a,System.Windows.Forms.Control? b=null,System.Windows.Forms.Control? c=null){int row=layout.RowCount++;layout.RowStyles.Add(new(SizeType.Absolute,44));layout.Controls.Add(a,0,row);if(b!=null)layout.Controls.Add(b,1,row);if(c!=null)layout.Controls.Add(c,2,row);foreach(var item in new[]{a,b,c})if(item!=null)item.Dock=DockStyle.Fill;}
        Row(new Label{Text="标定设备",Font=new Font(Font.FontFamily,18,FontStyle.Bold)},status);
        Row(new Label{Text="角度与关节控制页一致（°）"},new Label{Text="最小角度"},new Label{Text="最大角度"});
        string[] names=["1 · 顶部旋转","2 · 上部俯仰","3 · 中部弯曲","4 · 最底部弯曲"];
        for(int i=0;i<4;i++){for(int j=0;j<2;j++)values[i,j]=new TextBox{Margin=new(6,8,6,8)};Row(new Label{Text=names[i]},values[i,0],values[i,1]);}
        Row(new Label{Text="前向允许距离（mm，≤350）"},forward);Row(new Label{Text="后向允许距离（mm，≤500）"},backward);
        var save=new Button{Text="确认标定",AutoSize=true};Row(save);var path=new Label{Text=store.FilePath,AutoSize=false,AutoEllipsis=true,Font=new Font(Font.FontFamily,8)};Row(path);layout.SetColumnSpan(path,3);
        save.Click+=(_,_)=>{try{if(busy())throw new InvalidOperationException("运动/规划期间不能保存标定");var c=new Calibration{Signs=signs(),Motors=Enumerable.Range(0,4).Select(i=>new MotorLimit(i+1,Parse(values[i,0]),Parse(values[i,1]))).ToArray(),TaskLimits=new(){ForwardLimit=(double)forward.Value/1000,BackwardLimit=(double)backward.Value/1000}};store.Save(c);status.Text="已标定 · 已覆盖保存";Saved?.Invoke();}catch(Exception e){status.Text=e.Message;}};
        try{var c=store.Load(signs());for(int i=0;i<4;i++){var m=c.Motors.Single(m=>m.LogicalId==i+1);values[i,0].Text=m.MinAngleDeg.ToString(CultureInfo.CurrentCulture);values[i,1].Text=m.MaxAngleDeg.ToString(CultureInfo.CurrentCulture);}forward.Value=(decimal)(c.TaskLimits.ForwardLimit*1000);backward.Value=(decimal)(c.TaskLimits.BackwardLimit*1000);status.Text="已标定";}catch{status.Text="当前未标定";}

        var headStatus = new Label { AutoSize = true };
        Row(new Label { Text = "头部关节标定", Font = new Font(Font.FontFamily, 15, FontStyle.Bold) }, headStatus);
        Row(new Label { Text = "关节角度范围（°）" }, new Label { Text = "最小角度" }, new Label { Text = "最大角度" });
        var headRows = registry.HeadMotors.Select(motor =>
        {
            var title = new Label();
            var range = motor.HeadLimits!;
            var minimum = AngleBox(range.Minimum);
            var maximum = AngleBox(range.Maximum);
            Row(title, minimum, maximum);
            return (Motor: motor, Title: title, Minimum: minimum, Maximum: maximum);
        }).ToArray();
        void UpdateHeadNames()
        {
            foreach (var row in headRows)
                row.Title.Text = $"{row.Motor.DeviceId}号电机-{row.Motor.Name} ({row.Motor.HeadJoint})";
        }
        UpdateHeadNames();
        VisibleChanged += (_, _) => { if (Visible) UpdateHeadNames(); };
        var saveHead = new Button { Text = "保存头部角度范围", AutoSize = true };
        Row(saveHead);
        saveHead.Click += (_, _) =>
        {
            try
            {
                if (busy()) throw new InvalidOperationException("运动/规划期间不能保存标定");
                var updated = headRows.Select(row => new HeadAngleRange(Parse(row.Minimum), Parse(row.Maximum))).ToArray();
                foreach (var range in updated) range.Validate();
                var previous = headRows.Select(row => row.Motor.HeadLimits).ToArray();
                for (int i = 0; i < headRows.Length; i++) headRows[i].Motor.HeadLimits = updated[i];
                if (!registry.Save())
                {
                    for (int i = 0; i < headRows.Length; i++) headRows[i].Motor.HeadLimits = previous[i];
                    throw new InvalidOperationException("头部角度范围保存失败，请查看通信日志");
                }
                headStatus.Text = "头部角度范围已保存";
            }
            catch (Exception e) when (e is ArgumentException or InvalidOperationException) { headStatus.Text = e.Message; }
        };
    }

    static TextBox AngleBox(double angle) => new() { Text = angle.ToString(CultureInfo.CurrentCulture), Margin = new(6, 8, 6, 8) };
    static double Parse(TextBox box)=>double.TryParse(box.Text,NumberStyles.Float,CultureInfo.CurrentCulture,out var v)&&double.IsFinite(v)?v:throw new ArgumentException("请填写有效的角度数值");
}

