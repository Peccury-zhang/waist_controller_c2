using System.Numerics;
namespace WaistModel;

/// <summary>Conservative low-speed model checks. Dimensions must be verified against the actual assembly.</summary>
public sealed class WorkspaceSafety
{
    public double LinkRadius { get; set; } = 0.035;
    public double TorsoRadius { get; set; } = 0.14;
    public double Clearance { get; set; } = 0.015;
    public double GroundZ { get; set; } = 0;
    public double ChassisTop { get; set; } = 0.15;
    public double SupportHalfX { get; set; } = 0.23;
    public double SupportHalfY { get; set; } = 0.218;
    public double SupportMargin { get; set; } = 0.04;
    public void Validate()
    {
        double[] values=[LinkRadius,TorsoRadius,Clearance,GroundZ,ChassisTop,SupportHalfX,SupportHalfY,SupportMargin];
        if(values.Any(x=>!double.IsFinite(x))||LinkRadius<=0||TorsoRadius<=0||Clearance<0||SupportMargin<=0||SupportHalfX<=SupportMargin||SupportHalfY<=SupportMargin||ChassisTop<=GroundZ)throw new InvalidDataException("无效碰撞/支撑参数");
    }
    public void Check(Robot robot,double[] q)
    {
        Validate();var fk=robot.Forward(q);Vector3 P(string link)=>fk[link].Translation;
        var a=P("bady_link_1");var b=P("bady_link_2");var c=P("bady_link_3");var d=P("bady_link_4");var h=P("head_link_2");
        if(Math.Min(b.Z,c.Z)-LinkRadius<GroundZ+Clearance)throw new ArgumentException("折叠连杆接近地面");
        if(Math.Min(d.Z,h.Z)-TorsoRadius<ChassisTop+Clearance)throw new ArgumentException("上身接近底盘/地面");
        // Lower link and upper body are non-adjacent; adjacent links intentionally excluded.
        if(SegmentDistance(a,b,d,h)<LinkRadius+TorsoRadius+Clearance)throw new ArgumentException("下部连杆与上身碰撞裕量不足");
        double mass=0;Vector3 center=Vector3.Zero;
        foreach(var item in robot.Inertias){mass+=item.Mass;center+=Vector3.Transform(item.Center,fk[item.Link])*(float)item.Mass;}
        if(mass<=0)throw new ArgumentException("模型缺少质量信息");center/=(float)mass;
        if(Math.Abs(center.X)>SupportHalfX-SupportMargin||Math.Abs(center.Y)>SupportHalfY-SupportMargin)throw new ArgumentException("静态质心投影超出支撑裕量");
    }
    static double SegmentDistance(Vector3 p1,Vector3 q1,Vector3 p2,Vector3 q2)
    {
        var d1=q1-p1;var d2=q2-p2;var r=p1-p2;double a=Vector3.Dot(d1,d1),e=Vector3.Dot(d2,d2),f=Vector3.Dot(d2,r),s,t;
        if(a<=1e-12&&e<=1e-12)return Vector3.Distance(p1,p2);
        if(a<=1e-12){s=0;t=Math.Clamp(f/e,0,1);}else{double c=Vector3.Dot(d1,r);if(e<=1e-12){t=0;s=Math.Clamp(-c/a,0,1);}else{double b=Vector3.Dot(d1,d2),denom=a*e-b*b;s=denom!=0?Math.Clamp((b*f-c*e)/denom,0,1):0;t=(b*s+f)/e;if(t<0){t=0;s=Math.Clamp(-c/a,0,1);}else if(t>1){t=1;s=Math.Clamp((b-c)/a,0,1);}}}
        return Vector3.Distance(p1+d1*(float)s,p2+d2*(float)t);
    }
}
