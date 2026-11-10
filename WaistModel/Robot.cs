using System.Globalization;
using System.Numerics;
using System.Security.Cryptography;
using System.Xml.Linq;

namespace WaistModel;

public sealed record Joint(string Name, string Parent, string Child, string Type, Matrix4x4 Origin, Vector3 Axis, double Lower, double Upper);
public sealed record Inertia(string Link,double Mass,Vector3 Center);
public sealed record Visual(string Link, string Mesh, Matrix4x4 Origin, Vector3 Scale, Vector4 Color);
public sealed class Robot
{
    public static readonly string[] AxisNames = ["bady_joint_4", "bady_joint_3", "bady_joint_2", "bady_joint_1"];
    public string Hash { get; }
    public IReadOnlyList<Inertia> Inertias { get; private set; } = [];
    public IReadOnlyList<Joint> Joints { get; }
    public IReadOnlyList<Visual> Visuals { get; }
    public IReadOnlySet<string> Links { get; }
    public Joint[] Axes => AxisNames.Select(n => Joints.Single(j => j.Name == n)).ToArray();
    private Robot(string hash, List<Joint> joints, List<Visual> visuals, HashSet<string> links)
    { Hash = hash; Joints = joints.AsReadOnly(); Visuals = visuals.AsReadOnly(); Links = links; }
    static double Number(string s) { var d = double.Parse(s, CultureInfo.InvariantCulture); if (!double.IsFinite(d)) throw new InvalidDataException("非有限 URDF 值"); return d; }
    static Vector3 Vector(string? s, Vector3 fallback) => s == null ? fallback : ParseVector(s);
    static Vector3 ParseVector(string s) { var a = s.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(Number).ToArray(); if (a.Length != 3) throw new InvalidDataException("URDF 向量长度"); return new((float)a[0], (float)a[1], (float)a[2]); }
    static Matrix4x4 Origin(XElement? e)
    {
        var xyz = Vector(e?.Attribute("xyz")?.Value, Vector3.Zero); var rpy = Vector(e?.Attribute("rpy")?.Value, Vector3.Zero);
        // System.Numerics uses row vectors: transpose of T * Rz * Ry * Rx.
        return Matrix4x4.CreateRotationX(rpy.X) * Matrix4x4.CreateRotationY(rpy.Y) * Matrix4x4.CreateRotationZ(rpy.Z) * Matrix4x4.CreateTranslation(xyz);
    }
    public static Robot Load(string file, string meshRoot)
    {
        var root = XDocument.Load(file).Root ?? throw new InvalidDataException("空 URDF");
        var links = root.Elements("link").Select(x => (string)x.Attribute("name")!).ToHashSet();
        if (links.Count != root.Elements("link").Count()) throw new InvalidDataException("重复 link");
        var raw = root.Elements("joint").ToArray(); var removed = new HashSet<string> { "left_base", "righr base", "right_base" };
        int previous; do { previous = removed.Count; foreach (var j in raw) if (removed.Contains((string)j.Element("parent")!.Attribute("link")!)) removed.Add((string)j.Element("child")!.Attribute("link")!); } while (previous != removed.Count);
        links.ExceptWith(removed); var joints = new List<Joint>();
        foreach (var j in raw)
        {
            string child = (string)j.Element("child")!.Attribute("link")!; if (removed.Contains(child)) continue;
            string name = (string)j.Attribute("name")!, parent = (string)j.Element("parent")!.Attribute("link")!, type = (string)j.Attribute("type")!;
            if (!links.Contains(parent) || !links.Contains(child)) throw new InvalidDataException("悬空 joint");
            if (type is not ("fixed" or "revolute" or "continuous")) throw new InvalidDataException("不支持的 joint " + type);
            var lim = j.Element("limit");
            double low = type == "revolute" ? Number(lim?.Attribute("lower")?.Value ?? throw new InvalidDataException("缺少下限")) : 0;
            double high = type == "revolute" ? Number(lim?.Attribute("upper")?.Value ?? throw new InvalidDataException("缺少上限")) : 0;
            var axis = Vector(j.Element("axis")?.Attribute("xyz")?.Value, Vector3.UnitX);
            if (low > high || type != "fixed" && axis.Length() < 1e-6) throw new InvalidDataException("无效限位/轴");
            joints.Add(new(name,parent,child,type,Origin(j.Element("origin")),type == "fixed" ? Vector3.Zero : Vector3.Normalize(axis),low,high));
        }
        if (joints.Select(j => j.Name).Distinct().Count() != joints.Count || joints.Select(j => j.Child).Distinct().Count() != joints.Count) throw new InvalidDataException("重复关节或多父节点");
        var ordered = new List<Joint>(); var seen = new HashSet<string> { "base_link" }; var remaining = new List<Joint>(joints);
        while (remaining.Count > 0) { var next = remaining.Where(j => seen.Contains(j.Parent) && !seen.Contains(j.Child)).ToArray(); if (next.Length == 0) throw new InvalidDataException("URDF 环或非单根"); foreach (var j in next) { ordered.Add(j); seen.Add(j.Child); remaining.Remove(j); } }
        if (!seen.SetEquals(links)) throw new InvalidDataException("孤立 link");
        var visuals = new List<Visual>(); meshRoot = Path.GetFullPath(meshRoot);
        foreach (var link in root.Elements("link").Where(l => links.Contains((string)l.Attribute("name")!))) foreach (var v in link.Elements("visual"))
        {
            var mesh = v.Element("geometry")?.Element("mesh") ?? throw new InvalidDataException("只支持 mesh visual");
            var uri = (string)mesh.Attribute("filename")!;
            const string prefix = "package://具身折叠（URDF）4/meshes/";
            if (!uri.StartsWith(prefix, StringComparison.Ordinal)) throw new InvalidDataException("未知 mesh 包 " + uri);
            var path = Path.GetFullPath(Path.Combine(meshRoot, uri[prefix.Length..]));
            if (!path.StartsWith(meshRoot + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase) || !File.Exists(path)) throw new InvalidDataException("缺失或越界 mesh " + path);
            var color = v.Element("material")?.Element("color")?.Attribute("rgba")?.Value.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(Number).ToArray() ?? [0.75,0.78,0.82,1];
            if (color.Length != 4) throw new InvalidDataException("无效颜色");
            visuals.Add(new((string)link.Attribute("name")!,path,Origin(v.Element("origin")),Vector(mesh.Attribute("scale")?.Value,Vector3.One),new((float)color[0],(float)color[1],(float)color[2],(float)color[3])));
        }
        var robot = new Robot(Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(file))),ordered,visuals,links);
        robot.Inertias = root.Elements("link").Where(l => links.Contains((string)l.Attribute("name")!) && l.Element("inertial") != null).Select(l => new Inertia((string)l.Attribute("name")!, Number(l.Element("inertial")!.Element("mass")!.Attribute("value")!.Value), Origin(l.Element("inertial")!.Element("origin")).Translation)).ToArray();
        if (robot.Inertias.Any(i => i.Mass < 0)) throw new InvalidDataException("负质量");
        _ = robot.Axes; return robot;
    }
    public void Check(double[] q)
    {
        if (q.Length != 4) throw new ArgumentException("需要四轴");
        var axes = Axes;
        for (int i=0;i<4;i++) if (!double.IsFinite(q[i]) || q[i] < axes[i].Lower || q[i] > axes[i].Upper)
            throw new ArgumentOutOfRangeException(axes[i].Name,$"逻辑轴 {i+1} 目标 {q[i]:F6} rad 超出 URDF [{axes[i].Lower}, {axes[i].Upper}]");
    }
    public Dictionary<string,Matrix4x4> Forward(double[] q, IReadOnlyDictionary<string, double>? extraJointAngles = null)
    {
        var result = new Dictionary<string,Matrix4x4> { ["base_link"] = Matrix4x4.Identity };
        foreach (var j in Joints)
        {
            int i = Array.IndexOf(AxisNames, j.Name);
            double angle = i >= 0 ? q[i] : extraJointAngles?.GetValueOrDefault(j.Name) ?? 0;
            var motion = j.Type == "fixed" ? Matrix4x4.Identity : Matrix4x4.CreateFromAxisAngle(j.Axis, (float)angle);
            result[j.Child] = motion * j.Origin * result[j.Parent];
        }
        return result;
    }
}
