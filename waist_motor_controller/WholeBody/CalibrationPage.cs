using System.Globalization;
using WaistModel;
namespace waist_motor_controller.WholeBody;
public sealed class CalibrationPage:UserControl
{
    readonly CalibrationStore store;readonly Func<bool> busy;readonly Func<int[]> signs;readonly TextBox[,] values=new TextBox[4,2];
    readonly NumericUpDown forward=new(){Minimum=0,Maximum=350,Value=350,DecimalPlaces=1},backward=new(){Minimum=0,Maximum=500,Value=500,DecimalPlaces=1};
    readonly Label status=new(){AutoSize=true};public event Action? Saved;
    public CalibrationPage(CalibrationStore store,Func<bool> busy,Func<int[]> signs)
    {
        this.store=store;this.busy=busy;this.signs=signs;Dock=DockStyle.Fill;BackColor=Color.FromArgb(24,29,37);ForeColor=Color.Gainsboro;
        var layout=new TableLayoutPanel{Dock=DockStyle.Top,AutoSize=true,ColumnCount=3,Padding=new(24)};layout.ColumnStyles.Add(new(SizeType.Percent,40));layout.ColumnStyles.Add(new(SizeType.Percent,30));layout.ColumnStyles.Add(new(SizeType.Percent,30));Controls.Add(layout);
        void Row(System.Windows.Forms.Control a,System.Windows.Forms.Control? b=null,System.Windows.Forms.Control? c=null){int row=layout.RowCount++;layout.RowStyles.Add(new(SizeType.Absolute,52));layout.Controls.Add(a,0,row);if(b!=null)layout.Controls.Add(b,1,row);if(c!=null)layout.Controls.Add(c,2,row);foreach(var item in new[]{a,b,c})if(item!=null)item.Dock=DockStyle.Fill;}
        Row(new Label{Text="标定设备",Font=new Font(Font.FontFamily,18,FontStyle.Bold)},status);
        Row(new Label{Text="角度与关节控制页一致（°）"},new Label{Text="最小角度"},new Label{Text="最大角度"});
        string[] names=["1 · 顶部旋转","2 · 上部俯仰","3 · 中部弯曲","4 · 最底部弯曲"];
        for(int i=0;i<4;i++){for(int j=0;j<2;j++)values[i,j]=new TextBox{Margin=new(6,8,6,8)};Row(new Label{Text=names[i]},values[i,0],values[i,1]);}
        Row(new Label{Text="前向允许距离（mm，≤350）"},forward);Row(new Label{Text="后向允许距离（mm，≤500）"},backward);
        var save=new Button{Text="确认标定",AutoSize=true};Row(save);var path=new Label{Text=store.FilePath,AutoSize=false,AutoEllipsis=true,Font=new Font(Font.FontFamily,8)};Row(path);layout.SetColumnSpan(path,3);
        save.Click+=(_,_)=>{try{if(busy())throw new InvalidOperationException("运动/规划期间不能保存标定");var c=new Calibration{Signs=signs(),Motors=Enumerable.Range(0,4).Select(i=>new MotorLimit(i+1,Parse(values[i,0]),Parse(values[i,1]))).ToArray(),TaskLimits=new(){ForwardLimit=(double)forward.Value/1000,BackwardLimit=(double)backward.Value/1000}};store.Save(c);status.Text="已标定 · 已覆盖保存";Saved?.Invoke();}catch(Exception e){status.Text=e.Message;}};
        try{var c=store.Load(signs());for(int i=0;i<4;i++){var m=c.Motors.Single(m=>m.LogicalId==i+1);values[i,0].Text=m.MinAngleDeg.ToString(CultureInfo.CurrentCulture);values[i,1].Text=m.MaxAngleDeg.ToString(CultureInfo.CurrentCulture);}forward.Value=(decimal)(c.TaskLimits.ForwardLimit*1000);backward.Value=(decimal)(c.TaskLimits.BackwardLimit*1000);status.Text="已标定";}catch{status.Text="当前未标定";}
    }
    static double Parse(TextBox box)=>double.TryParse(box.Text,NumberStyles.Float,CultureInfo.CurrentCulture,out var v)&&double.IsFinite(v)?v:throw new ArgumentException("请填写全部八个有效角度数值");
}

