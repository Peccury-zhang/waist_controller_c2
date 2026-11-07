using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using WaistModel;
using Visual = WaistModel.Visual;

namespace RobotVisualization;
public sealed class RobotView : System.Windows.Forms.UserControl
{
    readonly System.Windows.Forms.Integration.ElementHost host = new() { Dock=System.Windows.Forms.DockStyle.Fill };
    readonly Viewport3D viewport = new();
    readonly PerspectiveCamera camera = new() { FieldOfView=40, UpDirection=new(0,0,1), NearPlaneDistance=0.01, FarPlaneDistance=100 };
    readonly List<(Visual Visual,GeometryModel3D Actual,GeometryModel3D Target)> parts = new();
    readonly Robot robot;
    double azimuth=-1.1,elevation=0.3,distance=3.2; System.Windows.Point? drag;
    public RobotView(Robot model, Action? readCurrentPose = null)
    {
        robot=model; Controls.Add(host);
        var border=new Border { Background=new SolidColorBrush(Color.FromRgb(32,38,47)),Child=viewport };
        var sceneLayout = new Grid();
        sceneLayout.Children.Add(border);
        var cameraTools = new StackPanel
        {
            Orientation = Orientation.Horizontal,
            HorizontalAlignment = System.Windows.HorizontalAlignment.Left,
            VerticalAlignment = System.Windows.VerticalAlignment.Top,
            Margin = new System.Windows.Thickness(8),
        };
        cameraTools.Children.Add(CameraButton("复位视角", () => ResetView()));
        cameraTools.Children.Add(CameraButton("侧视", () => ResetView(true)));
        if (readCurrentPose is not null)
            cameraTools.Children.Add(CameraButton("当前位姿", readCurrentPose));
        sceneLayout.Children.Add(cameraTools);
        host.Child = sceneLayout;
        viewport.Camera=camera;
        var scene=new Model3DGroup(); scene.Children.Add(new AmbientLight(Color.FromRgb(130,130,130))); scene.Children.Add(new DirectionalLight(Colors.White,new(-1,1,-2)));
        foreach(var visual in robot.Visuals)
        {
            var mesh=ReadStl(visual.Mesh); mesh.Freeze();
            var actual=new GeometryModel3D(mesh,Material(visual.Color.X,visual.Color.Y,visual.Color.Z,1));
            var target=new GeometryModel3D(mesh,Material(0.2f,0.75f,1,0.18f));
            actual.BackMaterial=actual.Material; target.BackMaterial=target.Material;
            scene.Children.Add(actual); parts.Add((visual,actual,target));
        }
        // Ground reference grid in URDF metres.
        for(int i=-5;i<=5;i++)
        {
            AddStrip(scene,new(-0.6,i*0.1,0),new(0.6,i*0.1,0),0.002,Material(0.3f,0.35f,0.4f,1));
            AddStrip(scene,new(i*0.1,-0.6,0),new(i*0.1,0.6,0),0.002,Material(0.3f,0.35f,0.4f,1));
        }
        AddStrip(scene,new(0,0,0.003),new(0.4,0,0.003),0.006,Material(1,0.2f,0.2f,1));
        AddStrip(scene,new(0,0,0.003),new(0,0.4,0.003),0.006,Material(0.2f,1,0.2f,1));
        // Draw transparent target after opaque actual geometry.
        foreach(var p in parts) scene.Children.Add(p.Target);
        viewport.Children.Add(new ModelVisual3D { Content=scene });
        border.MouseDown+=(_,e)=> { drag=e.GetPosition(border); border.CaptureMouse(); };
        border.MouseUp+=(_,_)=> { drag=null; border.ReleaseMouseCapture(); };
        border.MouseMove+=(_,e)=> { if(drag is not {} d) return; var p=e.GetPosition(border); azimuth-=(p.X-d.X)*0.01; elevation=Math.Clamp(elevation+(p.Y-d.Y)*0.01,-1.2,1.2); drag=p; UpdateCamera(); };
        border.MouseWheel+=(_,e)=> { distance=Math.Clamp(distance*Math.Exp(-e.Delta/1200.0),0.5,12); UpdateCamera(); };
        UpdateCamera(); UpdatePose(new double[4],null,false);
    }
    private static Button CameraButton(string text, Action onClick)
    {
        var button = new Button
        {
            Content = text,
            Width = 78,
            Height = 28,
            FontSize = 13,
            Margin = new System.Windows.Thickness(0, 0, 6, 0),
            Background = new SolidColorBrush(Color.FromRgb(55, 65, 80)),
            BorderBrush = Brushes.SlateGray,
            Foreground = Brushes.White,
        };
        button.Click += (_, _) => onClick();
        return button;
    }

    static void AddStrip(Model3DGroup scene,Point3D a,Point3D b,double width,Material material)
    {
        var offset=Vector3D.CrossProduct(b-a,new Vector3D(0,0,1));offset.Normalize();offset*=width/2;
        var mesh=new MeshGeometry3D{Positions=new Point3DCollection{a-offset,a+offset,b+offset,b-offset},TriangleIndices=new Int32Collection{0,1,2,0,2,3}};
        mesh.Freeze();scene.Children.Add(new GeometryModel3D(mesh,material){BackMaterial=material});
    }
    public void SaveSnapshot(string path)
    {
        var element=(System.Windows.FrameworkElement)host.Child;
        element.UpdateLayout();
        var bitmap=new System.Windows.Media.Imaging.RenderTargetBitmap(Math.Max(1,(int)element.ActualWidth),Math.Max(1,(int)element.ActualHeight),96,96,PixelFormats.Pbgra32);
        bitmap.Render(element);var encoder=new System.Windows.Media.Imaging.PngBitmapEncoder();encoder.Frames.Add(System.Windows.Media.Imaging.BitmapFrame.Create(bitmap));using var output=File.Create(path);encoder.Save(output);
    }
    public void ResetView(bool side=false) { azimuth=side ? -Math.PI/2 : -1.1; elevation=0.3; distance=3.2; UpdateCamera(); }
    void UpdateCamera() { var p=new Point3D(distance*Math.Cos(elevation)*Math.Cos(azimuth),distance*Math.Cos(elevation)*Math.Sin(azimuth),0.65+distance*Math.Sin(elevation)); camera.Position=p; camera.LookDirection=new Point3D(0,0,0.65)-p; }
    static Material Material(float r,float g,float b,float a) { var m=new DiffuseMaterial(new SolidColorBrush(Color.FromScRgb(a,r,g,b))); m.Freeze(); return m; }
    public void UpdatePose(double[] actual,double[]? target,bool stale)
    {
        var fk=robot.Forward(actual); var goal=target==null ? null : robot.Forward(target);
        foreach(var p in parts)
        {
            p.Actual.Transform=Transform(p.Visual,fk[p.Visual.Link]);
            p.Actual.Material=stale ? Material(0.45f,0.45f,0.45f,1) : Material(p.Visual.Color.X,p.Visual.Color.Y,p.Visual.Color.Z,1); p.Actual.BackMaterial=p.Actual.Material;
            p.Target.Transform=Transform(p.Visual,(goal??fk)[p.Visual.Link]); p.Target.Material=goal==null ? null : Material(0.2f,0.75f,1,0.18f); p.Target.BackMaterial=p.Target.Material;
        }
    }
    static MatrixTransform3D Transform(Visual v,System.Numerics.Matrix4x4 world)
    {
        var m=System.Numerics.Matrix4x4.CreateScale(v.Scale)*v.Origin*world;
        return new(new Matrix3D(m.M11,m.M12,m.M13,m.M14,m.M21,m.M22,m.M23,m.M24,m.M31,m.M32,m.M33,m.M34,m.M41,m.M42,m.M43,m.M44));
    }
    public static MeshGeometry3D ReadStl(string path)
    {
        var mesh=new MeshGeometry3D(); using var stream=File.OpenRead(path); using var reader=new BinaryReader(stream);
        if(stream.Length>=84)
        {
            stream.Position=80; uint count=reader.ReadUInt32();
            if(stream.Length==84+50L*count)
            {
                for(uint i=0;i<count;i++) { reader.ReadBytes(12); for(int v=0;v<3;v++) { float x=reader.ReadSingle(),y=reader.ReadSingle(),z=reader.ReadSingle(); if(!float.IsFinite(x)||!float.IsFinite(y)||!float.IsFinite(z)) throw new InvalidDataException("STL 非有限顶点"); mesh.Positions.Add(new(x,y,z)); mesh.TriangleIndices.Add(mesh.Positions.Count-1); } reader.ReadUInt16(); }
                return mesh;
            }
        }
        foreach(var line in File.ReadLines(path)) { var fields=line.Trim().Split((char[]?)null,StringSplitOptions.RemoveEmptyEntries); if(fields.Length==4&&fields[0]=="vertex") { var a=fields.Skip(1).Select(x=>double.Parse(x,CultureInfo.InvariantCulture)).ToArray(); if(a.Any(x=>!double.IsFinite(x))) throw new InvalidDataException("STL 非有限顶点"); mesh.Positions.Add(new(a[0],a[1],a[2])); mesh.TriangleIndices.Add(mesh.Positions.Count-1); } }
        if(mesh.Positions.Count==0||mesh.Positions.Count%3!=0) throw new InvalidDataException("无效 STL"); return mesh;
    }
}
