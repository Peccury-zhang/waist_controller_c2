using System.Reflection;
using System.Text;
using System.Text.Json;
using waist_motor_controller.Can;
using waist_motor_controller.Control;
using waist_motor_controller.Protocol;
using waist_motor_controller.UI;

internal static class Program
{
    [System.Runtime.InteropServices.DllImport("dwmapi.dll", ExactSpelling = true)]
    private static extern int DwmGetWindowAttribute(nint hwnd, int attribute, out int value, int size);
    [System.Runtime.InteropServices.DllImport("user32.dll")]
    private static extern nint SendMessage(nint hwnd, int message, nint wParam, nint lParam);
    [System.Runtime.InteropServices.DllImport("user32.dll")]
    private static extern bool PrintWindow(nint hwnd, nint deviceContext, uint flags);
    [System.Runtime.InteropServices.DllImport("user32.dll", EntryPoint = "GetWindowLongPtrW")]
    private static extern nint GetWindowStyle(nint hwnd, int index);
    [System.Runtime.InteropServices.DllImport("uxtheme.dll", CharSet = System.Runtime.InteropServices.CharSet.Unicode)]
    private static extern int SetWindowTheme(nint hwnd, string application, string classes);

    static int count;
    static void Assert(bool condition, string message)
    { if (!condition) throw new Exception(message); count++; }
    static CanFrame Response(int id, params byte[] data) => new(0x580u + (uint)id, data);
    static SdoResponse Ack(CanFrame f) => SdoMessage.Parse(Response((int)(f.Id - 0x600),
        0x60, f.Data[1], f.Data[2], f.Data[3], 0, 0, 0, 0));

    [STAThread]
    static int Main(string[] args)
    {
        try
        {
            if (args.Contains("--capture")) { Capture().GetAwaiter().GetResult(); return 0; }
            if (args.Contains("--ui-smoke")) { UiSmoke(); return 0; }
            if (args.Contains("--frame-smoke")) { FrameSmoke(); return 0; }
            if (args.Contains("--render")) { Render(args); return 0; }
            Run().GetAwaiter().GetResult();
            Console.WriteLine($"PASS: {count} assertions; no hardware opened.");
            return 0;
        }
        catch (Exception ex) { Console.Error.WriteLine(ex); return 1; }
    }

    static async Task Run()
    {
        Assert(SdoMessage.Read(3, 0x100A, 0).IsFd && SdoMessage.Read(3, 0x100A, 0).Brs, "SDO follows official CANFD+BRS configuration");
        Assert(PdoBuilder.Sync().IsFd && PdoBuilder.Sync().Brs && PdoBuilder.Sync().Id == 0x80, "sync follows CANFD+BRS configuration");
        Assert(CanFrame.TryNormalizeReceivedId(0x20000303, true, out uint normalized) && normalized == 0x303, "actual FD driver flag preserved as data");
        Assert(CanFrame.TryNormalizeReceivedId(0x20000583, true, out normalized) && normalized == 0x583, "actual SDO driver flag");
        Assert(!CanFrame.TryNormalizeReceivedId(0x20000303, false, out _), "classic error frame rejected");
        Assert(!CanFrame.TryNormalizeReceivedId(0x80000303, true, out _), "extended frames rejected");
        Assert(!CanFrame.TryNormalizeReceivedId(0x40000303, true, out _), "remote frames rejected");
        Assert(!CanFrame.TryNormalizeReceivedId(0x20001303, true, out _), "out-of-range standard ID rejected without alias");
        var hold = PdoBuilder.SingleAxis(3, PdoCommand.Hold(-1000));
        Assert(hold.Id == 0x103 && hold.IsFd && hold.Brs && hold.Length == 7, "single-axis frame");
        Assert(hold.Data.SequenceEqual(new byte[] { 0xD0, 0xFC, 0x18, 0, 0, 0, 0 }), "new position fields");
        var clear = PdoBuilder.SingleAxis(4, new PdoCommand(false, false, true, 0, 0));
        Assert(clear.Data[0] == 8, "reset bit3");
        Assert(PdoBuilder.SingleAxis(5, PdoCommand.Disable).Data.All(x => x == 0), "disable");
        Assert(Od.AngleToRaw(90) == 16384 && Od.AngleToRaw(-90) == -16384, "angle conversion");
        foreach (double bad in new[] { -180d, 180d, double.NaN, double.PositiveInfinity })
        {
            bool rejected = false;
            try { Od.AngleToRaw(bad); } catch (ArgumentOutOfRangeException) { rejected = true; }
            Assert(rejected, "out-of-range angle rejected");
        }

        var fb = FeedbackParser.Parse(4, new CanFrame(0x304,
            new byte[] { 0xC0, 0x00, 0xFF, 0x9C, 0xFF, 0xE5, 0x78, 0x81, 0x0A, 0x01, 0x2C, 0 }, true));
        Assert(fb.PositionRaw == -16384 && fb.PositionDeg == -90, "feedback position");
        Assert(fb.VelocityRpm == -100 && fb.CurrentMa == -27, "feedback signed values");
        Assert(fb.Mode == 1 && fb.Enabled && fb.BrakeReleased && fb.HasError, "feedback B6 bits");
        Assert(fb.FaultCode == 0x0A81 && fb.TemperatureRaw == 300, "feedback fault and temperature offsets");
        Assert(FaultCodes.Describe(0x0800).Contains("电流跟踪误差超差"), "confirmed R120 current error bit");
        Assert(FaultCodes.Describe(0x0200).Contains("位置跃迁"), "jump error bit");
        using (var fixture = JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Fixtures", "r120-faults.json"))))
        {
            Assert(fixture.RootElement.GetProperty("Sheet").GetString() == "ZYR120&120MAX报错逻辑", "correct model worksheet");
            var entries = fixture.RootElement.GetProperty("Entries").EnumerateArray().ToArray();
            Assert(FaultCodes.Entries.Count == entries.Length, "all assigned R120 fault rows covered");
            foreach (var entry in entries)
            {
                ushort code = entry.GetProperty("Code").GetUInt16();
                string name = entry.GetProperty("Name").GetString()!;
                var match = FaultCodes.Entries.Single(e => e.Code == code);
                Assert(match.Name == name, $"fault name agrees with sheet row {entry.GetProperty("Row")}");
                bool restart = entry.GetProperty("ClearMethod").GetString()!.Contains("重启");
                Assert(FaultCodes.RequiresRestart(code) == restart, "restart policy agrees with worksheet");
                Assert(FaultCodes.CanClearWithCommand(code) == !restart, "command clearing agrees with worksheet");
            }
        }
        Assert(FaultCodes.Describe(0x1000).Contains("位置跟踪误差过大"), "position tracking bit covered");
        Assert(FaultCodes.Describe(0x2002).Contains("欠压") && FaultCodes.Describe(0x2002).Contains("未定义"), "unknown bits preserved with known errors");
        Assert(!FaultCodes.CanClearWithCommand(0x2000), "no fabricated reset policy for unassigned bits");
        bool wrongLength = false;
        try { FeedbackParser.Parse(4, new CanFrame(0x304, new byte[16])); }
        catch (ArgumentException) { wrongLength = true; }
        Assert(wrongLength, "legacy feedback rejected");

        foreach (var item in new (byte Cmd, int Size)[] { (0x4F, 1), (0x4B, 2), (0x47, 3), (0x43, 4) })
        {
            var response = SdoMessage.Parse(Response(3, item.Cmd, 0x64, 0x60, 0, 1, 2, 3, 4));
            Assert(response.Data.Length == item.Size, "SDO expedited length");
            Assert(SdoMessage.Matches(SdoMessage.Read(3, 0x6064, 0), response), "SDO index matched");
            Assert(!SdoMessage.Matches(SdoMessage.Read(3, 0x606C, 0), response), "unrelated SDO ignored");
        }
        // 真机抓取：603F返回0000；错误偏移曾将温度高字节误报为0100。
        foreach (var item in new (int Id, string Hex, short Position, short Temperature)[] {
            (3, "00A00000FFE9000001130210", 160, 275),
            (4, "FFD9000000080000011D0210", -39, 285),
            (5, "FFDC0000000A000001000000", -36, 256),
            (6, "FFED0000FFD00000011D0000", -19, 285) })
        {
            var actual = FeedbackParser.Parse(item.Id, new CanFrame((uint)(0x300 + item.Id), Convert.FromHexString(item.Hex), true), firmware117: true);
            Assert(actual.FaultCode == 0 && !actual.HasError, "live frame has no false encoder fault");
            Assert(actual.PositionRaw == item.Position && actual.TemperatureRaw == item.Temperature, "live firmware117 offsets");
            Assert(!actual.Enabled && !actual.BrakeReleased, "live motors initially disabled");
            Assert(actual.TargetReached == (item.Id is 3 or 4), "actual firmware target-reached bit4");
        }
        var firmwareHold = PdoBuilder.SingleAxis(3, PdoCommand.Hold(160), firmware117: true);
        Assert(firmwareHold.Data.SequenceEqual(Convert.FromHexString("C200A007D00005")), "firmware117 enable hold matches original protocol");
        Assert(PdoBuilder.SingleAxis(3, new PdoCommand(false, false, true, 0, 0), true).Data[0] == 0x20, "firmware117 reset bit");
        Assert(PdoBuilder.SingleAxis(3, PdoCommand.Disable, true).Data.All(v => v == 0), "firmware117 disable");
        var abort = SdoMessage.Parse(Response(3, 0x80, 0x31, 0x25, 0, 0, 0, 2, 6));
        Assert(abort.IsAbort && abort.AbortCode == 0x06020000, "abort code");
        foreach (string version in new[] { "V1.17.12", "V1.17.13" })
        {
            byte[] text = Encoding.ASCII.GetBytes(version + "\0\0\0"); // screenshot: length 0B
            int phase = 0;
            var read = await SdoStringReader.ReadAsync(4, Od.SoftwareVersion, request =>
            {
                byte[] data;
                if (phase == 0)
                {
                    Assert(request.Data[0] == 0x40 && request.Data[1] == 0x0A && request.Data[2] == 0x10, "version object");
                    data = new byte[] { 0x41, 0x0A, 0x10, 0, (byte)text.Length, 0, 0, 0 };
                }
                else
                {
                    Assert(request.Data[0] == (phase == 1 ? 0x60 : 0x70), "segment toggle");
                    data = new byte[8];
                    data[0] = phase == 1 ? (byte)0 : (byte)0x17;
                    Array.Copy(text, (phase - 1) * 7, data, 1, phase == 1 ? 7 : 4);
                }
                phase++;
                return Task.FromResult<SdoResponse?>(SdoMessage.Parse(Response(4, data)));
            });
            Assert(read == version && phase == 3, "segmented firmware decoded");
        }

        foreach (short target in new short[] { 20000, -20000, 0 })
        {
            var path = new PositionTrajectory(-1000);
            path.Move(target, 5, 10);
            int steps = 0;
            while (path.IsMoving && steps++ < 100000)
            {
                short previous = path.CommandRaw;
                short next = path.Step(steps % 3 == 0 ? 0.2 : 0.001);
                Assert(Math.Abs(next - previous) <= PositionTrajectory.MaxStepRaw, "position step below100");
                Assert(next >= Math.Min(-1000, (int)target) && next <= Math.Max(-1000, (int)target), "no overshoot");
            }
            Assert(path.CommandRaw == target, "trajectory arrives");
            path.Hold(path.CommandRaw);
            Assert(path.Step(1) == target, "hold stable");
        }

        string temp = Path.Combine(Path.GetTempPath(), "waist-tests-" + Guid.NewGuid());
        Directory.CreateDirectory(temp);
        try
        {
            var defaults = AppConfig.CreateDefault();
            Assert(defaults.Motors.Select(m => m.DeviceId).SequenceEqual(new[] { 3, 4, 5, 6 }), "new IDs");
            string config = Path.Combine(temp, "motors.json");
            defaults.SchemaVersion = 1;
            for (int i = 0; i < 4; i++) defaults.Motors[i].DeviceId = 11 + i;
            File.WriteAllText(config, JsonSerializer.Serialize(defaults));
            var registry = new MotorRegistry(config);
            Assert(registry.Motors.Select(m => m.DeviceId).SequenceEqual(new[] { 3, 4, 5, 6 }), "legacy IDs migrated");
            Assert(File.Exists(config + ".pre-v2.bak") && registry.Config.SchemaVersion == 2, "migration backup");
            defaults.Motors[0].DeviceId = 21;
            File.WriteAllText(config, JsonSerializer.Serialize(defaults));
            Assert(new MotorRegistry(config).Motors[0].DeviceId == 21, "custom ID preserved");

            var fake = new FakeCan();
            var frames = new List<CanFrame>();
            var scheduler = new KeepaliveScheduler(f => { frames.Add(f); return fake.Send(f); });
            var controller = new MotorController(fake, scheduler, registry);
            Assert(!controller.Enable(1, 5, 1).Ok, "unconfigured ID rejected");
            fake.Alive = false;
            Assert(!controller.Enable(3, 5, 1).Ok, "stale feedback prevents enable");
            fake.Alive = true;
            Assert(!controller.MoveToPosition(3, 10, 5, 1).Ok, "motion requires local enable");
            Assert(controller.Enable(3, 5, 1).Ok && frames[^1].ReadBe16(1) == 0, "enable holds feedback");
            Assert(!controller.MoveToPosition(3, 10, 5, 1).Ok, "enable window enforced");
            await Task.Delay(160);
            Assert(controller.MoveIncrement(3, 1, 5, 1).Ok, "positive increment");
            Assert(scheduler.GetTarget(3) == Od.AngleToRaw(1), "increment relative to feedback");
            Assert(!controller.MoveIncrement(3, 1, 5, 1).Ok, "busy increment rejected");
            controller.StopMotion(3);
            Assert(!scheduler.IsMoving(3), "stop holds");
            Assert(controller.MoveIncrement(3, -1, 5, 1).Ok && scheduler.GetTarget(3) == Od.AngleToRaw(-1), "negative increment");
            controller.StopMotion(3);
            Assert(controller.MoveToPosition(3, 20, 5, 1).Ok, "absolute move");
            for (int i = 0; i < 20; i++) scheduler.Tick(0.001);
            int before = frames.Count;
            var eStop = controller.EmergencyStop();
            Assert(eStop.Ok && frames.Count == before + 4, "four emergency frames");
            Assert(frames.Skip(before).Select(f => f.Id).SequenceEqual(new uint[] { 0x103, 0x104, 0x105, 0x106 }), "only four IDs stopped");
            scheduler.Tick(0.1);
            Assert(frames.Count == before + 4, "no enabled frame after emergency stop");
            Assert(frames.All(f => f.Id != 0x200), "no motor broadcast");

            Assert(controller.Enable(3, 5, 1).Ok, "re-enable after stop");
            fake.FailSend = true;
            scheduler.Tick(0.01);
            Assert(!scheduler.IsArmed(3), "send failure disarms");
            fake.FailSend = false;

            int faultStart = frames.Count;
            fake.Fault = 0x0800;
            Assert(!controller.ClearError(3).Ok && frames.Count == faultStart, "restart-only fault does not send futile clear");
            fake.Fault = 0x2000;
            Assert(!controller.ClearError(3).Ok, "unknown fault not guessed clearable");
            fake.Fault = 0x0004; fake.Temperature = 850;
            Assert(!controller.ClearError(3).Ok, "overtemperature cooldown required");
            fake.Temperature = 840;
            Assert(controller.ClearError(3).Ok, "overtemperature clear permitted after cooldown");
            fake.Fault = 0;
            Assert(controller.Enable(3, 5, 1).Ok, "enable for calibration");
            await Task.Delay(160);
            fake.Reached = false;
            Assert(!controller.IsAtTarget(3), "hardware not-reached overrides near target estimate");
            fake.Reached = true;
            Assert(controller.IsAtTarget(3), "hardware reached still requires physical target check");
            fake.Reached = null;
            fake.SdoRequests.Clear();
            var zero = await controller.SetZeroOriginAsync(3, 5, 1);
            Assert(zero.Ok, "zero succeeds with fresh feedback");
            Assert(fake.SdoRequests.Count == 2 && fake.SdoRequests[0].ReadLe32(4) == 1 &&
                fake.SdoRequests[1].ReadLe32(4) == 0, "zero writes1 then0");
            Assert(!scheduler.IsArmed(3), "zero leaves disabled");

            Assert(controller.Enable(3, 5, 1).Ok, "enable for cancellation");
            await Task.Delay(160);
            var pending = new TaskCompletionSource<SdoResponse?>(TaskCreationOptions.RunContinuationsAsynchronously);
            fake.Pending = pending;
            fake.SdoRequests.Clear();
            Task<(bool Ok, string Message)> operation = controller.SetZeroOriginAsync(3, 5, 1);
            controller.EmergencyStop();
            pending.SetResult(Ack(fake.SdoRequests[0]));
            var cancelled = await operation;
            Assert(!cancelled.Ok && fake.SdoRequests.Count == 1, "emergency cancels delayed calibration");
            Assert(!scheduler.IsArmed(3), "cancel never re-enables");
            fake.Pending = null;

            var groupRegistry = new MotorRegistry(config); // Includes a custom CAN ID, not hardcoded 3~6.
            int[] groupIds = groupRegistry.Motors.Select(m => m.DeviceId).ToArray();
            var groupCan = new FakeCan();
            var groupFrames = new List<CanFrame>();
            var groupScheduler = new KeepaliveScheduler(f => { groupFrames.Add(f); return groupCan.Send(f); });
            var groupController = new MotorController(groupCan, groupScheduler, groupRegistry);
            for (int i = 0; i < groupIds.Length; i++) groupCan.Positions[groupIds[i]] = (short)((i % 2 == 0 ? 1 : -1) * (1000 + i * 100));
            Assert(!groupController.ReturnAllToZero(5, 1).Ok && groupFrames.Count == 0, "all-zero never auto-enables motors");
            foreach (int id in groupIds) Assert(groupController.Enable(id, 5, 1).Ok, "enable mapped joint for group test");
            Assert(!groupController.ReturnAllToZero(5, 1).Ok, "all-zero respects enable protection period");
            await Task.Delay(160);
            bool Unchanged() => groupIds.All(id => groupScheduler.GetTarget(id) == groupCan.Positions[id]);
            groupCan.OfflineIds.Add(groupIds[^1]);
            Assert(!groupController.ReturnAllToZero(5, 1).Ok && Unchanged(), "one offline joint rejects entire group without changing any target");
            groupCan.OfflineIds.Clear(); groupCan.Fault = 0x0100;
            Assert(!groupController.ReturnAllToZero(5, 1).Ok && Unchanged(), "fault prevents group motion");
            groupCan.Fault = 0;
            Assert(!groupController.ReturnAllToZero(5, 99).Ok && Unchanged(), "invalid group profile cannot partly start");
            Assert(groupController.MoveToPosition(groupIds[^1], 10, 5, 1).Ok, "start existing individual trajectory");
            Assert(!groupController.ReturnAllToZero(5, 1).Ok && groupScheduler.GetTarget(groupIds[0]) == groupCan.Positions[groupIds[0]], "busy joint prevents other joints starting");
            groupController.StopMotion(groupIds[^1]);
            groupScheduler.ArmHold(1, 321);
            groupFrames.Clear();
            Assert(groupController.ReturnAllToZero(5, 1).Ok && groupIds.All(id => groupScheduler.GetTarget(id) == 0), "all mapped joints receive zero targets");
            Assert(groupScheduler.GetTarget(1) == 321 && groupCan.SdoRequests.Count == 0, "all-zero does not change other joints or calibrate origins");
            Assert(!groupController.ReturnAllToZero(5, 1).Ok, "repeated all-zero during motion rejected");
            for (int i = 0; i < 10000 && groupIds.Any(groupScheduler.IsMoving); i++) groupScheduler.Tick(0.005);
            Assert(groupIds.All(id => !groupScheduler.IsMoving(id)), "all group trajectories arrive at zero");
            foreach (int id in groupIds) Assert(groupFrames.Last(f => f.Id == 0x100u + id).ReadBe16(1) == 0, "final position frame is zero");
            groupController.EmergencyStop();
            Assert(groupIds.All(id => !groupScheduler.IsArmed(id)), "emergency stop cancels all-zero group");
            Assert(groupController.Enable(groupIds[0], 5, 1).Ok && groupController.Enable(groupIds[2], 5, 1).Ok, "enable only two of the four mapped joints");
            await Task.Delay(160);
            groupCan.OfflineIds.Add(groupIds[1]); // An unarmed/offline joint must not block enabled joints.
            groupFrames.Clear();
            var partialHome = groupController.ReturnAllToZero(5, 1);
            Assert(partialHome.Ok && partialHome.Message.Contains("跳过") && partialHome.Message.Contains(groupIds[1].ToString()), "partial all-zero reports skipped joints");
            Assert(groupScheduler.GetTarget(groupIds[0]) == 0 && groupScheduler.GetTarget(groupIds[2]) == 0, "only enabled subset receives zero targets");
            Assert(!groupScheduler.IsArmed(groupIds[1]) && !groupScheduler.IsArmed(groupIds[3]), "disabled joints remain disabled");
            groupScheduler.Tick(0.005);
            Assert(!groupFrames.Any(f => f.Id == 0x100u + groupIds[1] || f.Id == 0x100u + groupIds[3]), "no frames sent to skipped mapped joints");
            groupController.EmergencyStop();
            groupFrames.Clear();
            Assert(!groupController.ReturnAllToZero(5, 1).Ok && groupFrames.Count == 0, "no enabled joints produces no commands");
            groupCan.OfflineIds.Clear();
            groupCan.OfflineIds.Add(groupIds[^1]);
            Assert(!groupController.EnableAll(5, 1).Ok && groupFrames.Count == 0, "all-enable prechecks all mapped joints before sending");
            groupCan.OfflineIds.Clear(); groupCan.Fault = 0x0100;
            Assert(!groupController.EnableAll(5, 1).Ok && groupFrames.Count == 0, "all-enable does not override faults");
            groupCan.Fault = 0;
            Assert(groupController.EnableAll(5, 1).Ok && groupIds.All(groupScheduler.IsArmed), "all-enable arms all four mapped joints");
            Assert(groupFrames.Count == 4 && groupFrames.Select(f => (int)(f.Id - 0x100)).SequenceEqual(groupIds), "all-enable only sends to mapped joints");
            foreach (int id in groupIds) Assert(groupScheduler.GetTarget(id) == groupCan.Positions[id], "all-enable holds each feedback position");
            Assert(groupController.EnableAll(5, 1).Ok && groupFrames.Count == 4, "repeated all-enable skips armed joints without resending");
            groupController.EmergencyStop(); groupCan.FailSend = true;
            Assert(!groupController.EnableAll(5, 1).Ok && groupIds.All(id => !groupScheduler.IsArmed(id)), "all-enable send failure leaves no armed trajectories");

            var receive = new CanFdTransport();
            var process = typeof(CanFdTransport).GetMethod("ProcessFrame", BindingFlags.Instance | BindingFlags.NonPublic)!;
            void Inject(uint id, byte length, byte[] data) => process.Invoke(receive, new object[] { id, length, data });
            Inject(0x303, 12, new byte[12]);
            Assert(receive.GetFeedback(3) != null, "transport accepts new layout");
            Inject(0x303, 16, new byte[16]);
            Assert(receive.BadFeedbackCount == 1, "transport rejects old layout");
            Inject(0x583, 2, new byte[2]); // truncated SDO must not kill receive loop
            Inject(0x083, 2, new byte[] { 0x02, 0x08 });
            Assert(receive.GetFeedback(3)?.FaultCode == 0x0802, "EMCY faults invalidate cache");

            bool safe = true;
            var safetyFrames = new List<CanFrame>();
            var safety = new KeepaliveScheduler(f => { safetyFrames.Add(f); return true; }, _ => safe);
            safety.ArmHold(6, 0);
            safety.Move(6, 100, 1, 5);
            safe = false; safety.Tick(0.001);
            Assert(!safety.IsArmed(6) && safetyFrames[^1].Data[0] == 0, "feedback loss disables");

            var entered = new ManualResetEventSlim();
            var release = new ManualResetEventSlim();
            bool block = false;
            var concurrentFrames = new List<CanFrame>();
            var concurrent = new KeepaliveScheduler(f =>
            {
                if (block && f.Data[0] != 0) { entered.Set(); release.Wait(); }
                concurrentFrames.Add(f); return true;
            });
            concurrent.ArmHold(3, 0); block = true;
            Task tick = Task.Run(() => concurrent.Tick(0.001));
            Assert(entered.Wait(1000), "tick entered sender");
            Task disarm = Task.Run(() => concurrent.SendDisabled(3));
            await Task.Delay(30);
            Assert(!disarm.IsCompleted, "disable serialized with in-flight frame");
            release.Set();
            await Task.WhenAll(tick, disarm);
            concurrent.Tick(0.1);
            Assert(concurrentFrames[^1].Data[0] == 0 && !concurrent.IsArmed(3), "last concurrent frame disabled");
        }
        finally { Directory.Delete(temp, true); }
    }

    static async Task Capture()
    {
        using var can = new CanFdTransport();
        can.Log += Console.WriteLine;
        var seen = new HashSet<uint>();
        can.RawFrameReceived += f =>
        {
            if (f.Id >= 0x303 && f.Id <= 0x306 && seen.Add(f.Id))
                Console.WriteLine($"FEEDBACK 0x{f.Id:X3} {Convert.ToHexString(f.Data)}");
        };
        can.ConfigureFirmware117(new[] {3, 4, 5, 6});
        if (!can.Open(0, 0, out string error)) throw new Exception(error);
        can.Send(PdoBuilder.Sync());
        await Task.Delay(200);
        foreach (int id in new[] {3, 4, 5, 6})
            foreach (ushort index in new ushort[] {0x603F, 0x6041, 0x6061, 0x2662})
            {
                var response = await can.SdoRequestAsync(id, SdoMessage.Read(id, index, 0));
                Console.WriteLine($"SDO {id} {index:X4}: cmd={response?.Command:X2} data={Convert.ToHexString(response?.Data ?? Array.Empty<byte>())}");
            }
        Console.WriteLine(can.GetBusDiagnostics());
    }

    static void UiSmoke()
    {
        ApplicationConfiguration.Initialize();
        using var form = new MainForm();
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic;
        var selection = (ComboBox)typeof(MainForm).GetField("_motorCombo", flags)!.GetValue(form)!;
        var cards = (Array)typeof(MainForm).GetField("_motorCards", flags)!.GetValue(form)!;
        var onClick = typeof(System.Windows.Forms.Control).GetMethod("OnClick", flags)!;
        for (int i = 0; i < cards.Length; i++)
        {
            var card = (System.Windows.Forms.Control)cards.GetValue(i)!;
            var row = card.Controls.OfType<FlowLayoutPanel>().Single();
            var idInput = row.Controls.OfType<NumericUpDown>().Single();
            decimal originalId = idInput.Value;
            // Test each separately: row blank space used to swallow clicks beside the online label.
            var areas = new[] { card, row }.Concat(card.Controls.OfType<Label>()).Concat(row.Controls.OfType<Label>());
            foreach (var area in areas)
            {
                selection.SelectedIndex = (i + 1) % cards.Length;
                onClick.Invoke(area, new object[] { EventArgs.Empty });
                Assert(selection.SelectedIndex == i, $"card {i + 1}: {area.GetType().Name} selects matching joint");
            }
            Assert(idInput.Value == originalId, "selecting card regions does not change CAN ID");
        }
        selection.SelectedIndex = 0;
        var navigation = (Button[])typeof(MainForm).GetField("_pageButtons", flags)!.GetValue(form)!;
        var pages = (Panel[])typeof(MainForm).GetField("_pages", flags)!.GetValue(form)!;
        var target = (NumericUpDown)typeof(MainForm).GetField("_targetAngleNum", flags)!.GetValue(form)!;
        var logBox = (TextBox)typeof(MainForm).GetField("_logBox", flags)!.GetValue(form)!;
        selection.SelectedIndex = 2; target.Value = 12; logBox.AppendText("navigation persistence check");
        string logBefore = logBox.Text;
        for (int round = 0; round < 3; round++)
            for (int i = 0; i < navigation.Length; i++)
            {
                onClick.Invoke(navigation[i], new object[] { EventArgs.Empty });
                Assert((int)typeof(MainForm).GetField("_selectedPage", flags)!.GetValue(form)! == i, "navigation selects requested page");
                Assert(selection.SelectedIndex == 2 && target.Value == 12 && logBox.Text == logBefore, "page switching preserves joint, parameters and logs");
                Assert(!pages[i].IsDisposed && ((NavigationButton)navigation[i]).Selected, "page remains alive and corresponding tab stays selected");
            }
        onClick.Invoke(navigation[0], new object[] { EventArgs.Empty });
        selection.SelectedIndex = 0;
        string[] buttonFields = { "_enableButton", "_disableButton", "_clearErrButton", "_setZeroButton", "_moveButton", "_jogFwdButton", "_jogRevButton", "_stopMotionButton", "_returnZeroButton" };
        var buttons = buttonFields.Select(name => (Button)typeof(MainForm).GetField(name, flags)!.GetValue(form)!).ToArray();
        var ready = new OperationUiState(true, true, false, false, true, true, false, false, false, true);
        form.ApplyOperationState(ready);
        int switches = 0;
        foreach (var b in buttons) b.EnabledChanged += (_, _) => switches++;
        for (int i = 0; i < 1000; i++)
        {
            form.ApplyOperationState(ready with { Moving = i % 2 == 0, AtTarget = i % 3 == 0, InEnableWindow = i % 4 == 0 });
            Assert(buttons[3].Enabled && buttons[4].Enabled && buttons[5].Enabled && buttons[6].Enabled && buttons[8].Enabled, "short motion leaves control buttons visually stable");
        }
        Assert(switches == 0, "zero EnabledChanged events across 1000 simulated motion refreshes");
        form.ApplyOperationState(ready with { Armed = false, MotorEnabled = false });
        Assert(!buttons[1].Enabled && buttons[0].Enabled, "unenabled motor offers enable but not disable");
        form.ApplyOperationState(ready with { Armed = false });
        Assert(buttons[1].Enabled, "fresh enabled feedback allows disable even without local arm");
        form.ApplyOperationState(ready with { FeedbackFresh = false });
        Assert(buttons[1].Enabled, "local armed motor can still be disabled after feedback loss");
        form.ApplyOperationState(ready with { Connected = false });
        Assert(!buttons[1].Enabled, "closed connection disables the disable button");
        form.ApplyOperationState(ready with { Faulted = true, CommandClearable = false });
        Assert(!buttons[4].Enabled && !buttons[5].Enabled && !buttons[2].Enabled && !buttons[8].Enabled, "fault still disables unsafe commands");
        form.ApplyOperationState(ready with { FeedbackFresh = false });
        Assert(!buttons[4].Enabled && !buttons[5].Enabled && !buttons[8].Enabled, "lost feedback still disables motion");
        form.ApplyOperationState(ready with { Configuring = true });
        Assert(!buttons[4].Enabled && !buttons[3].Enabled, "configuration locks commands");
        using var label = new Label();
        int textChanges = 0, colorChanges = 0;
        label.TextChanged += (_, _) => textChanges++; label.ForeColorChanged += (_, _) => colorChanges++;
        for (int i = 0; i < 1000; i++) { UiValueCache.Text(label, "0.00°"); UiValueCache.Color(label, Color.CornflowerBlue); }
        Assert(textChanges == 1 && colorChanges == 1, "unchanged telemetry does not update properties repeatedly");
        using var number = new NumericUpDown { Minimum = -1, Maximum = 1, DecimalPlaces = 2, Increment = 0.25M };
        using var shell = new NumberInputShell(number) { Size = new Size(300, 60) };
        var arrows = shell.Controls.OfType<Button>().ToDictionary(b => b.Name);
        onClick.Invoke(arrows["Increment"], new object[] { EventArgs.Empty });
        Assert(number.Value == 0.25M, "wide arrow uses configured decimal increment");
        for (int i = 0; i < 10; i++) onClick.Invoke(arrows["Decrement"], new object[] { EventArgs.Empty });
        Assert(number.Value == -1, "wide arrow respects numeric lower bound");
        number.Enabled = false;
        Assert(arrows.Values.All(b => !b.Enabled), "wide arrows follow disabled numeric state");
        Console.WriteLine($"PASS: UI smoke ({count} assertions); 1000 motion refreshes, 0 button enable/disable transitions; no hardware opened.");
    }

    static void FrameSmoke()
    {
        ApplicationConfiguration.Initialize();
        Directory.CreateDirectory("tmp");
        using var form = new MainForm { ShowInTaskbar = false, StartPosition = FormStartPosition.Manual, Location = new Point(-5000, -5000) };
        form.Show(); Application.DoEvents();
        void Check(string phase)
        {
            Application.DoEvents();
            Assert((GetWindowStyle(form.Handle, -16).ToInt64() & 0x00C00000) == 0, $"{phase}: no system caption style");
            Assert(form.PointToScreen(Point.Empty) == form.Location && form.ClientSize == form.Size, $"{phase}: no native caption or border inset; origin={form.PointToScreen(Point.Empty)}, location={form.Location}, client={form.ClientSize}, size={form.Size}");
            using var image = new Bitmap(form.Width, form.Height);
            using (var graphics = Graphics.FromImage(image))
            {
                nint context = graphics.GetHdc();
                try { Assert(PrintWindow(form.Handle, context, 0), $"{phase}: full window capture succeeds"); }
                finally { graphics.ReleaseHdc(context); }
            }
            // DrawToBitmap omits native chrome, so use PrintWindow to catch the reported overlay.
            Assert(image.GetPixel(form.Width / 2, 12) == UiTheme.Background, $"{phase}: native caption does not cover custom title area");
            image.Save(Path.GetFullPath($"tmp/ui-frame-{phase}.png"));
        }
        Check("normal");
        foreach (bool classic in new[] { false, true })
        {
            if (classic) SetWindowTheme(form.Handle, "", "");
            for (int i = 0; i < 4; i++)
            {
                SendMessage(form.Handle, 0x86, 0, 0);
                SendMessage(form.Handle, 0x86, 1, 0);
                SendMessage(form.Handle, 0x85, 1, 0);
                form.Text = "yclab折叠腰控制台 · 重绘检查";
                form.Text = "yclab折叠腰控制台";
                form.ClientSize = new Size(1280 + i * 20, 900);
            }
            Check(classic ? "classic" : "reactivated");
        }
        form.WindowState = FormWindowState.Minimized; Application.DoEvents();
        form.WindowState = FormWindowState.Normal; Application.DoEvents();
        Check("restored");
        form.Close();
        Console.WriteLine($"PASS: custom frame ({count} assertions); full-window capture, activation, resize, classic fallback, minimize/restore; no hardware opened.");
    }

    static void Render(string[] args)
    {
        ApplicationConfiguration.Initialize();
        foreach (var size in new[] { new Size(1280, 900), new Size(1920, 1080) })
        {
            using var form = new MainForm();
            form.StartPosition = FormStartPosition.Manual; form.Location = new Point(-5000, -5000);
            form.ShowInTaskbar = false; form.Show(); form.ClientSize = size; Application.DoEvents();
            if (OperatingSystem.IsWindowsVersionAtLeast(10, 0, 22000))
            {
                var frame = WindowFrameTheme.Apply(form.Handle);
                Assert(frame.Dark == 0 && frame.Border == 0 && frame.Caption == 0 && frame.Text == 0,
                    $"DWM accepts dark frame and custom colors: {frame}");
                Assert(DwmGetWindowAttribute(form.Handle, 20, out int dark, sizeof(int)) == 0 && dark == 1, "native frame has dark mode enabled");
                Console.WriteLine("PASS: native DWM dark caption, border and light title text; no hardware opened.");
            }
            Assert(form.Text == "yclab折叠腰控制台" && form.Icon != null, "custom window title and logo loaded");
            var titleBar = form.Controls.OfType<WindowTitleBar>().Single();
            Assert(titleBar.Controls.OfType<Label>().Single().Font.SizeInPoints == 13, "window title uses requested compact font");
            Assert(titleBar.Controls.OfType<PictureBox>().Single().Width >= 40 * form.DeviceDpi / 96, "window logo uses enlarged display area");
            nint Hit(int x, int y)
            {
                var screenPoint = form.PointToScreen(new Point(x, y));
                return SendMessage(form.Handle, 0x84, 0, (nint)((screenPoint.X & 0xffff) | (screenPoint.Y & 0xffff) << 16));
            }
            Assert(Hit(200, titleBar.Height / 2) == 2, "enlarged caption preserves native dragging and double-click hit area");
            Assert(Hit(1, 1) == 13 && Hit(form.ClientSize.Width - 1, form.ClientSize.Height - 1) == 17, "window corners retain resizing hit areas");
            Assert(Hit(form.ClientSize.Width - 30, titleBar.Height / 2) == 1, "window action buttons remain clickable client controls");
            var selection = (ComboBox)typeof(MainForm).GetField("_motorCombo", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            var cards = (Array)typeof(MainForm).GetField("_motorCards", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            typeof(System.Windows.Forms.Control).GetMethod("OnClick", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(cards.GetValue(2), new object[] { EventArgs.Empty });
            Assert(selection.SelectedIndex == 2, "motor card selects matching control target");
            selection.SelectedIndex = 0;
            if (args.Contains("--demo"))
            {
                var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                void Label(string name, string value) { ((Label)typeof(MainForm).GetField(name, flags)!.GetValue(form)!).Text = value; }
                Label("_connStateLabel", "● 适配器已打开");
                ((Label)typeof(MainForm).GetField("_connStateLabel", flags)!.GetValue(form)!).ForeColor = Color.FromArgb(68, 218, 170);
                Label("_busIdLabel", "3, 4, 5, 6");
                Label("_rxTxLabel", "188657 / 99437"); Label("_posLabel", "12.50°");
                Label("_velLabel", "0 RPM"); Label("_curLabel", "−27 mA"); Label("_modeLabel", "PP 轮廓位置");
                Label("_tempLabel", "27.5 °C"); Label("_faultLabel", "无故障");
                ((Label)typeof(MainForm).GetField("_faultLabel", flags)!.GetValue(form)!).ForeColor = Color.FromArgb(68, 218, 170);
                Label("_firmwareLabel", "R120 · V1.17.12");
                Label("_motionHint", "已就绪 · 点动每点击一次，增减指定角度。");
                foreach (var name in new[] { "_disableButton", "_setZeroButton", "_moveButton", "_returnZeroButton", "_jogFwdButton", "_jogRevButton", "_stopMotionButton", "_eStopButton", "_enableAllButton", "_changeIdButton" })
                    ((Button)typeof(MainForm).GetField(name, flags)!.GetValue(form)!).Enabled = true;
                ((Button)typeof(MainForm).GetField("_openCloseButton", flags)!.GetValue(form)!).Text = "关闭设备";
                ((Button)typeof(MainForm).GetField("_enableButton", flags)!.GetValue(form)!).Enabled = false;
                ((Button)typeof(MainForm).GetField("_saveMappingButton", flags)!.GetValue(form)!).Enabled = false;
                foreach (var number in (NumericUpDown[])typeof(MainForm).GetField("_idNums", flags)!.GetValue(form)!) number.Enabled = false;
                var lights = (Label[])typeof(MainForm).GetField("_stateLights", flags)!.GetValue(form)!;
                lights[0].ForeColor = Color.FromArgb(68, 218, 170); lights[1].ForeColor = Color.FromArgb(64, 134, 247); lights[3].ForeColor = Color.FromArgb(255, 195, 93); lights[3].Text = "● 到位";
                foreach (var label in (Label[])typeof(MainForm).GetField("_onlineLabels", flags)!.GetValue(form)!)
                { label.Text = "● 在线"; label.ForeColor = Color.FromArgb(68, 218, 170); }
            }
            Application.DoEvents();
            var navigation = (Button[])typeof(MainForm).GetField("_pageButtons", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            var pages = (Panel[])typeof(MainForm).GetField("_pages", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            var emergency = (Button)typeof(MainForm).GetField("_eStopButton", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            var homeAll = (Button)typeof(MainForm).GetField("_returnAllZeroButton", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            Assert(emergency.Parent == homeAll.Parent && emergency.Top > homeAll.Bottom, "emergency stop is placed below all-zero on joint page");
            for (int pageIndex = 0; pageIndex < pages.Length; pageIndex++)
            {
                if (pageIndex == 0)
                {
                    navigation[0].PerformClick(); Application.DoEvents();
                    Assert(pages[0].Visible && navigation.All(b => b.Visible), "clicking current page keeps its content and all four buttons visible");
                }
                navigation[pageIndex].PerformClick(); Application.DoEvents();
                Assert(pages.Where((page, i) => page.Visible != (i == pageIndex)).Count() == 0, "only selected page is visible");
                Assert(navigation.All(b => b.Visible), "navigation stays visible on every page");
                Assert(navigation.Select((b, i) => ((NavigationButton)b).Selected == (i == pageIndex)).All(value => value), "only current page button stays selected");
                if (pageIndex == 1)
                {
                    var flags = BindingFlags.Instance | BindingFlags.NonPublic;
                    var number = (NumericUpDown)typeof(MainForm).GetField("_targetAngleNum", flags)!.GetValue(form)!;
                    number.Value = 12.50M;
                    navigation[pageIndex].PerformClick(); Application.DoEvents();
                    Assert(pages[1].Visible, "clicking selected joint button never collapses page content");
                    navigation[pageIndex].PerformClick(); Application.DoEvents();
                    Assert(pages[1].Visible && number.Value == 12.50M, "repeated selection preserves content and numeric input");
                    foreach (string name in new[] { "_targetAngleNum", "_jogSpeedNum", "_jogAccelNum", "_jogStepNum" })
                    {
                        var input = (NumericUpDown)typeof(MainForm).GetField(name, flags)!.GetValue(form)!;
                        input.PerformLayout();
                        var edit = input.Controls.OfType<TextBox>().Single();
                        Assert(edit.Left == 0 && edit.Right >= input.Parent!.ClientSize.Width, "native numeric editor covers entire visible viewport");
                        Assert(input.Controls.Cast<System.Windows.Forms.Control>().Where(c => c is not TextBox).All(c => c.Left >= input.Parent!.ClientSize.Width), "native spinner is fully clipped outside the visible viewport");
                    }
                }
                var transition = (PageTransition)typeof(MainForm).GetField("_pageTransition", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
                var settle = System.Diagnostics.Stopwatch.StartNew();
                while (transition.Visible && settle.ElapsedMilliseconds < 1500) { Application.DoEvents(); Thread.Sleep(10); }
                Assert(!transition.Visible, "page transition finishes and releases input overlay");
                Assert(navigation.All(b => b.Bounds.Y == navigation[0].Top && b.Height == navigation[0].Height), "navigation buttons stay aligned and equal height after selection");
                Assert(pages[pageIndex].Visible, "selecting a button leaves selected content visible");
                using var image = new Bitmap(form.ClientSize.Width, form.ClientSize.Height);
                form.DrawToBitmap(image, new Rectangle(Point.Empty, image.Size));
                string prefix = args.Contains("--demo") ? "ui-preview" : "ui-offline";
                string suffix = pageIndex == 0 ? "" : new[] { "", "-joints", "-whole-body", "-calibration" }[pageIndex];
                string path = Path.GetFullPath($"tmp/{prefix}{suffix}-{size.Width}.png");
                Directory.CreateDirectory(Path.GetDirectoryName(path)!); image.Save(path); Console.WriteLine(path);
                if (pageIndex == 1 && size.Width == 1920)
                {
                    var content = pages[1].Controls[0];
                    using var full = new Bitmap(content.Width, content.Height);
                    content.DrawToBitmap(full, new Rectangle(Point.Empty, full.Size));
                    full.Save(Path.GetFullPath("tmp/ui-joints-full.png"));
                }
            }
            // Interrupted transitions must settle on the latest request and release their input overlay.
            foreach (int index in new[] { 1, 2, 0, 3, 1 }) navigation[index].PerformClick();
            Application.DoEvents();
            var animation = (PageTransition)typeof(MainForm).GetField("_pageTransition", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(form)!;
            Assert(animation.Visible, "visible navigation starts a transition");
            var animationWait = System.Diagnostics.Stopwatch.StartNew();
            while (animation.Visible && animationWait.ElapsedMilliseconds < 1500) { Application.DoEvents(); Thread.Sleep(10); }
            Assert(!animation.Visible && pages[1].Visible && pages.Where((p, i) => i != 1).All(p => !p.Visible), "rapid navigation settles on latest requested page");
            selection.DroppedDown = true; Application.DoEvents();
            Assert(selection.DroppedDown, "custom joint selector opens native dropdown");
            selection.SelectedIndex = 3;
            selection.DroppedDown = false; Application.DoEvents();
            Assert(selection.SelectedIndex == 3, "custom joint selector preserves selection behavior");
            form.Close();
        }
        using var dialog = new ChangeMotorIdDialog(3, new HashSet<int> { 3, 4, 5, 6 });
        dialog.ShowInTaskbar = false; dialog.StartPosition = FormStartPosition.Manual; dialog.Location = new Point(-5000, -5000);
        dialog.Show(); Application.DoEvents();
        using var dialogImage = new Bitmap(dialog.Width, dialog.Height); dialog.DrawToBitmap(dialogImage, new Rectangle(Point.Empty, dialogImage.Size));
        dialogImage.Save(Path.GetFullPath("tmp/ui-id-dialog.png"));
        dialog.Close();
        RenderNavigationHover();
        RenderButtonStates();
    }

    static void RenderNavigationHover()
    {
        using var host = new Form { ClientSize = new Size(420, 80), ShowInTaskbar = false,
            StartPosition = FormStartPosition.Manual, Location = new Point(-5000, -5000) };
        var button = new NavigationButton { Text = "关节控制", TabStop = false };
        host.Controls.Add(button); host.Show(); Application.DoEvents();
        var bounds = button.Bounds;
        Bitmap Snapshot() { var bmp = new Bitmap(button.Width, button.Height); button.DrawToBitmap(bmp, button.ClientRectangle); return bmp; }
        void Pump(int milliseconds)
        {
            var wait = System.Diagnostics.Stopwatch.StartNew();
            while (wait.ElapsedMilliseconds < milliseconds) { Application.DoEvents(); Thread.Sleep(2); }
        }
        using var normal = Snapshot();
        var baseline = normal.GetPixel(20, 10);
        typeof(NavigationButton).GetMethod("OnMouseEnter", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(button, new object[] { EventArgs.Empty });
        Pump(40);
        using var partial = Snapshot();
        Assert(partial.GetPixel(20, 10) == baseline && partial.GetPixel(20, button.Height - 10) != baseline,
            "hover highlights bottom before top");
        partial.Save(Path.GetFullPath("tmp/ui-navigation-hover-partial.png"));
        Pump(220);
        using var full = Snapshot();
        Assert(full.GetPixel(20, 10) != baseline && full.GetPixel(20, 10) == full.GetPixel(20, button.Height - 10),
            "hover highlight eventually fills button");
        bool whiteText = false;
        for (int x = 60; x < full.Width - 60 && !whiteText; x++)
            for (int y = 15; y < full.Height - 15; y++)
                if (full.GetPixel(x, y).ToArgb() == Color.White.ToArgb()) { whiteText = true; break; }
        Assert(whiteText && button.Bounds == bounds, "navigation has white text and hover never changes bounds");
        full.Save(Path.GetFullPath("tmp/ui-navigation-hover-full.png"));
        typeof(NavigationButton).GetMethod("OnMouseLeave", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(button, new object[] { EventArgs.Empty });
        Pump(220);
        using var restored = Snapshot();
        Assert(restored.GetPixel(20, 10) == baseline && restored.GetPixel(20, button.Height - 10) == baseline,
            "mouse leave smoothly restores dark background");
        button.Selected = true;
        using var selected = Snapshot();
        var selectedColor = selected.GetPixel(20, 10);
        Assert(selectedColor != baseline, "selected tab keeps distinct fill");
        Assert(selectedColor.GetBrightness() > full.GetPixel(20, 10).GetBrightness(), "selected tab stays brighter than hover");
        Assert(selected.GetPixel(20, button.Height - 2).ToArgb() != NavigationStrip.AccentColor.ToArgb(), "selected tab bottom remains open to content");
        typeof(NavigationButton).GetMethod("OnMouseEnter", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(button, new object[] { EventArgs.Empty });
        Pump(220);
        typeof(NavigationButton).GetMethod("OnMouseLeave", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(button, new object[] { EventArgs.Empty });
        Pump(220);
        using var selectedAfterLeave = Snapshot();
        Assert(selectedAfterLeave.GetPixel(20, 10) == selectedColor, "selected button remains lit after hover leaves");
        button.Selected = false;
        using var deselected = Snapshot();
        Assert(deselected.GetPixel(20, 10) == baseline, "previous selection returns to normal brightness");
        host.Close();
        Console.WriteLine("PASS: navigation hover fills from bottom to top, white text, fixed size and mouse-leave restoration");
    }

    static void RenderButtonStates()
    {
        var type = typeof(MainForm).Assembly.GetType("waist_motor_controller.UI.ActionButton")!;
        var kindProperty = type.GetProperty("Kind")!;
        using var host = new Form { ClientSize = new Size(1220, 570), ShowInTaskbar = false, StartPosition = FormStartPosition.Manual, Location = new Point(-5000, -5000), BackColor = Color.FromArgb(12, 19, 31), AutoScaleMode = AutoScaleMode.None };
        var stage = new Panel { Dock = DockStyle.Fill, BackColor = host.BackColor };
        host.Controls.Add(stage);
        string[] names = { "默认", "鼠标悬停", "按下", "禁用" };
        string[] kinds = { "Primary", "Secondary", "Success", "Danger", "Warning" };
        for (int column = 0; column < names.Length; column++)
            stage.Controls.Add(new Label { Text = names[column], Left = 20 + column * 300, Top = 15, Width = 270, Height = 30, Font = new Font("Microsoft YaHei UI", 12), ForeColor = Color.White });
        var all = new List<Button>();
        for (int row = 0; row < kinds.Length; row++)
            for (int column = 0; column < 4; column++)
            {
                var b = (Button)Activator.CreateInstance(type)!;
                b.Text = new[] { "运行到指定角度", "当前位置设零", "使能电机", "急停 · 4台电机", "停止 / 保持" }[row];
                kindProperty.SetValue(b, Enum.Parse(kindProperty.PropertyType, kinds[row]));
                b.SetBounds(20 + column * 300, 60 + row * 100, 270, 78); b.TabStop = false;
                stage.Controls.Add(b); all.Add(b);
            }
        host.Show(); Application.DoEvents();
        Color Sample(Button b)
        {
            using var bmp = new Bitmap(b.Width, b.Height); b.DrawToBitmap(bmp, b.ClientRectangle); return bmp.GetPixel(24, 24);
        }
        for (int row = 0; row < kinds.Length; row++)
        {
            var normal = all[row * 4]; var hover = all[row * 4 + 1]; var pressed = all[row * 4 + 2]; var disabled = all[row * 4 + 3];
            type.GetMethod("OnMouseEnter", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(hover, new object[] { EventArgs.Empty });
            type.GetMethod("OnKeyDown", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(pressed, new object[] { new KeyEventArgs(Keys.Space) });
            disabled.Enabled = false;
            Assert(Sample(normal) != Sample(hover), "button hover visibly highlights");
            Assert(Sample(normal) != Sample(pressed), "button keyboard press feedback");
            var before = Sample(disabled);
            type.GetMethod("OnMouseEnter", BindingFlags.Instance | BindingFlags.NonPublic)!.Invoke(disabled, new object[] { EventArgs.Empty });
            Assert(before == Sample(disabled), "disabled button has no false hover feedback");
        }
        using var image = new Bitmap(host.ClientSize.Width, host.ClientSize.Height);
        stage.DrawToBitmap(image, new Rectangle(Point.Empty, image.Size)); image.Save(Path.GetFullPath("tmp/ui-button-states.png"));
        host.Close(); Console.WriteLine("PASS: hover, press and disabled states for all button variants");
    }

    sealed class FakeCan : ICanTransport
    {
        public bool IsOpen => true;
        public bool Alive = true, FailSend;
        public readonly HashSet<int> OfflineIds = new();
        public readonly Dictionary<int, short> Positions = new();
        public ushort Fault;
        public short Temperature = 300;
        public bool? Reached;
        public event Action<int, MotorFeedback>? FeedbackUpdated { add { } remove { } }
        public List<CanFrame> SdoRequests = new();
        public TaskCompletionSource<SdoResponse?>? Pending;
        public bool Send(CanFrame f) => !FailSend;
        public MotorFeedback GetFeedback(int id) => new()
        {
            DeviceId = id, PositionRaw = Positions.GetValueOrDefault(id), Enabled = true, BrakeReleased = true, Mode = 1,
            Timestamp = Environment.TickCount64, FaultCode = Fault, HasError = Fault != 0,
            TemperatureRaw = Temperature, IsFirmware117 = true, TargetReached = Reached,
        };
        public bool IsDeviceAlive(int id, int windowMs = 500) => Alive && !OfflineIds.Contains(id);
        public Task<SdoResponse?> SdoRequestAsync(int id, CanFrame f, int timeoutMs = 300)
        {
            SdoRequests.Add(f);
            return Pending?.Task ?? Task.FromResult<SdoResponse?>(Ack(f));
        }
    }
}
