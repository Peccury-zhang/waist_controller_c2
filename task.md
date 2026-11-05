# waist_controller_c2 设计方案（C# 上位机：4 轴巨蟹电机躯干控制）

> 目标：用 C# WinForms（.NET 10, x64）重写躯干（腰部）电机上位机，通过 ZLG USB-CANFD 适配器
> 以 CANFD 总线控制 4 台巨蟹一体化关节模组（3×R120MAX + 1×R120）。
> 工程位置：`waist_controller_c2/waist_motor_controller`（已有 WinForms 空骨架）。

---

## 1. 系统背景与约束

| 项 | 内容 |
|---|---|
| 电机 | 巨蟹一体化通讯模组 R120MAX×3、R120×1，固件协议见《头部关节模组使用协议简易说明书》 |
| 总线 | ZLG USB-CANFD（Windows x64 DLL），CANFD 标准帧，仲裁段 1M / 数据段 5M |
| 在线设备 | 共 6 个节点；本软件只管理 4 个：ID 11=腰部旋转（偏航），ID 12/13/14=折叠腰 上/中/下；ID 9/10 忽略（但其反馈帧会在总线上出现，解析层必须能容忍） |
| 逻辑电机名 | 电机1-偏航、电机2-折叠上部、电机3-折叠中部、电机4-折叠下部（名称固定，ID 可变，需映射并持久化） |
| 参考实现 | `qyh_waist_controller`（Qt 版）：协议帧构造、看门狗 keepalive、反馈解析均已被真机验证，本方案沿用其协议层结论并修正其架构问题 |

### 1.1 协议关键事实（设计依据，均出自协议文档/字典表/报错表/Qt 版验证）

> **文档优先级（用户确认）**：故障代码以《报错逻辑&故障代码V1.5-ZY.xlsx》`R102&R120&120MAX报错逻辑` 表为准；SDO/PDO 字典对象以《巨蟹CANopem字典表-V1.1.xlsx》`字典表`/`CANFD自定义PDO` 页为准。两者与说明书 PDF 冲突处，一律以 xlsx 为准。

1. **上电时序**：boot 300ms → App 初始化约 2s → 上传 `0x700+ID`（data=0x00）心跳；上伺服后须延时 100ms 才可向新位置转动；抱闸开/闭各有 100ms 延时。
2. **电机看门狗**：电机使能后若 **500ms 内未收到控制帧（0x100+ID / 0x200）即自锁**（抱闸吸合）。因此上位机对已使能电机必须以 ≤100ms 周期重发控制帧（keepalive）。可用 SDO 写 `0x2650=1` 关闭看门狗（同时会关闭机上限位保护，默认不关）。
3. **SDO**：请求 ID=`0x600+ID`，响应 ID=`0x580+ID`，DLC=8；指令代码 0x23(4B 写)/0x2B(2B 写)/0x2F(1B 写)/0x40(读)；**数据小端（低位在前）**；响应 0x60=写成功、0x80=abort。
4. **自定义 PDO 单轴控制帧**：ID=`0x100+ID`，DLC=7；Byte0 控制字（bit7 使能、bit6 抱闸、bit5 清错、bit4~1 模式、bit0 预留）；Byte1-2 目标参数1、Byte3-4 目标参数2、Byte5-6 前馈/轮廓速度；**16bit 字段全部大端（高位在前）**。
   - 模式编码（PDO 控制字）：0x01 轮廓位置 PP、0x02 轮廓速度 PV、0x03 CSP、0x04 CSV、0x05 电流环、0x06 MIT、0x07 力矩环。
   - 位置模式：参数1=输出端目标位置 int16，[-32768,32767]↔(-180°,180°)；参数2=加减速 [0,20000] RPM/S；参数3(Byte5-6)=输出端轮廓速度 RPM。
   - 速度模式：参数1=电机端目标速度 int16 RPM；参数2=加减速 RPM/S。
5. **多控广播帧**：ID=`0x200`，DLC=64；[0..55] 为 8 个 7 字节单轴分包，[56..63] 为对应 8 个执行器 ID；未用分包补 0。一帧同时控制最多 8 电机，并触发各电机反馈。
6. **反馈帧**：ID=`0x300+ID`，大端：[0-1]负载端位置 int16、[2-3]电机端速度 int16 RPM、[4-5]Iq 电流 int16 mA、[6-7]错误码 u16、[8-9]线圈温度 int16(0.1°C)。**两种长度（字典表 CANFD自定义PDO 页）**：
   - DLC=12：[10]=控制模式（0x01 PP…0x06 MIT、0x07 力矩环）、[11]=状态位（bit7 使能、bit6 抱闸释放、bit5 报错、bit4 到位）；
   - DLC=16（带力矩传感器版）：[10-11]=力矩 int16（0.05Nm）、[12-14]=预留（速度端编码器位置 int24）、[15]=**高4bit 状态位 + 低4bit 模式反馈**（位定义同上）。
   → 解析器必须按帧长分支（len==12 / len==16），其余长度丢弃并计数告警。
   触发方式：① 同步帧 `0x80`（DLC0）；② 多控帧；③ 单轴帧。→ 上位机用周期同步帧轮询反馈。另：模组上电完成发 `0x700+ID`（data=0x00），模组报错时发紧急帧 `0x080+ID`（EMCY，ID=1 即 0x081），接收层应记录日志。
7. **位置换算**：`deg = raw * 360.0 / 65536.0`；`raw = round(deg * 65536 / 360)`（int16 限幅）。SDO 读 `0x6064` 返回 int32（小端），`deg ≈ raw / 182.04`。
8. **SDO 与 PDO 的模式编号不同（极易踩坑）**：SDO `0x6060`：1=PP、**3=PV**、8=CSP、9=CSV、10=CST；PDO 控制字：1=PP、**2=PV**、3=CSP、4=CSV、5=电流。本方案主控走 PDO，编号以 PDO 为准，常量表分开定义。
9. **CiA402 控制字（SDO 0x6040）**：0x06 准备好(抱闸吸合) → 0x07 松抱闸 → 0x0F 使能激活；0x4F 触发 PP 绝对运动；0x80 清错；0x00 下使能。状态字 0x6041：0x0060 上电初始、0x0021 准备好、0x0023 松抱闸、0x0027 使能激活、0x0427 运行到位、0x0008 报错。
10. **常用字典对象**（以字典表 xlsx 为准）：0x2530 CAN ID（改后需重启生效）、0x2531 零位标定（写 1；**字典表注明"当前代码允许使能态设置零位，故障态不允许"**；写 2=扭矩传感器零漂标定，不用）、0x2650 看门狗开关、0x6064 实际位置(int32 cnt)、0x6041 状态字、0x6061 模式显示、0x607A 目标位置、0x6081 轮廓速度（**输出端，1~40 RPM，默认 10**——说明书写的 0~30 以字典表为准）、0x6083/0x6084 加减速（**字典表 1~200 RPM/S；说明书示例却用 2000 且称 0~10000，矛盾——主控走 PDO 帧内加减速字段 [0,20000] 不受影响，SDO 备选序列需按字典表限幅并真机验证**）、0x60FF 目标速度、0x603F 错误码、0x6078 实际电流、0x6079 母线电压(0.1V)、0x6080 电机最大转速(默认 2000RPM)、0x2540 CAN 波特率（0=CAN500K/1=CAN1M/2=FD1M+1M/3=FD1M+2M/4=FD1M+5M，本期不改不用）、0x2539/0x1010 Flash 保存（仅 PI 参数与最大电流需保存，改ID/标零不需要）、0x1017 心跳时间（`2B 17 10 00`，0=关闭，断电恢复默认，本期不用）。
11. **故障代码**（R120 系列，以《报错逻辑&故障代码V1.5-ZY.xlsx》`R102&R120&120MAX报错逻辑` 表为准）：0x0001 过压(重启清)、0x0002 欠压(重启清)、0x0004 过温(恢复到 85° 以下指令清；触发：线圈NTC>95° 或驱动器NTC>100° 持续 100ms)、0x0008 堵转(重启清)、0x0010 过载(重启清)、0x0020 速度跟踪误差过大(重启清)、0x0040 正限位(指令清，向限位反方向可运行)、0x0080 负限位(指令清，向限位反方向可运行)、0x0100 输出端编码器异常(重启清)、0x0200 位置跃迁过大(指令清，伺服在当前位置不响应指令)、0x0400 电流采样错误(重启清)、0x0800 电流跟踪误差超差(重启清)、0x1000 位置跟踪误差过大(重启清)、0x2000 高编与低编速度校核(重启清)。⚠️ 说明书（PDF 故障表）另有一份旧版代码含义（0x0020=电流采样错误、0x0400=上电电角度初始化失败等），与 V1.5 xlsx 不一致，一律以 xlsx 为准（本机电机即 R120/R120MAX，Qt 版亦按 xlsx 实现）。

### 1.2 Qt 版（qyh_waist_controller）的经验与教训

**沿用（已真机验证）**：
- 主控通道 = 自定义 PDO 单轴帧（0x100+ID），使能/移动/保持都用它；SDO 只做配置类操作（改 ID、标零、看门狗、扫描）。
- **keepalive 机制**：使能/移动后把该帧登记为 keepalive 帧，100ms 周期重发，喂电机 500ms 看门狗；失能/报错/标零前必须清除 keepalive。
- **使能前先缓存反馈**：使能帧的目标位置取当前反馈位置（hold），避免使能瞬间跳变；无反馈缓存时禁止使能/移动。
- 标零前先发失能帧并清 keepalive（坐标系平移会导致旧目标变成非零目标而跳动）。
- 收到报错反馈立即停该电机 keepalive。
- 接收线程同时轮询 TYPE_CAN 与 TYPE_CANFD 两个队列。

**修正（Qt 版的问题）**：
- Qt 版 UI 与协议逻辑全部堆在 MainWindow/CanFdDevice，C# 版严格分层（Native/Can/Protocol/Control/UI），协议层纯静态、无状态、可单测。
- Qt 版每帧 `QueuedConnection` 抛信号刷 UI，帧多时 UI 消息洪水；C# 版改为**接收线程只写缓存，UI 定时器 50ms 拉取刷新**。
- Qt 版轮廓参数是全局静态变量；C# 版参数属于 UI 控件/配置对象，显式传参。
- Qt 版 SDO 用开环 `singleShot` 定时序列；C# 版 SDO 读写做成 **请求-响应配对（TaskCompletionSource + 超时）**。
- 新增 Qt 版没有的**点动（jog）安全收敛**：松开按钮必须发 PV 速度 0 帧并把 keepalive 帧替换为速度 0 帧（既停转又不触发看门狗自锁）；窗口失焦/按键抬起/鼠标离开均要收敛。

---

## 2. 总体架构

```
┌────────────────────────────── UI 层 (WinForms) ──────────────────────────────┐
│ MainForm(滚动面板+3个GroupBox+日志框)   ChangeMotorIdDialog(图1样式)          │
└──────────────┬───────────────────────────────────────────────┬───────────────┘
               │ 用户操作                                       │ 50ms 拉取状态
┌──────────────▼───────────────────────────────────────────────▼───────────────┐
│ 控制层 Control                                                                │
│  MotorRegistry(名称↔ID映射+json持久化)   KeepaliveScheduler(100ms 重发)       │
│  MotorController(使能/失能/清错/标零/移动/点动/急停 时序状态机)                 │
└──────────────┬───────────────────────────────────────────────┬───────────────┘
               │ 构帧/解析                                      │ 反馈缓存
┌──────────────▼───────────────────────────────────────────────▼───────────────┐
│ 协议层 Protocol（纯静态、无状态）                                             │
│  PdoBuilder(0x100单轴/0x200多控/0x80同步)   SdoMessage(0x600请求/0x580响应)    │
│  FeedbackParser(0x300)   ObjectDictionary(索引常量)   FaultCodes(错误码表)    │
└──────────────┬───────────────────────────────────────────────┬───────────────┘
               │ Send(CanFrame)                                 │ event FrameReceived
┌──────────────▼───────────────────────────────────────────────▼───────────────┐
│ 传输层 Can：CanFdTransport（打开/关闭/接收线程/发送锁/反馈缓存/SDO响应路由）   │
├──────────────────────────────────────────────────────────────────────────────┤
│ Native 层：ZlgCanNative（P/Invoke + 结构体，ControlCANFD.dll x64）            │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 2.1 线程模型

| 线程 | 职责 |
|---|---|
| UI 线程 | 控件交互；50ms `System.Windows.Forms.Timer` 从缓存拉取反馈刷新状态标签 |
| 接收线程 | 后台线程循环：`ZCAN_GetReceiveNum(TYPE_CANFD/CAN)` → 批量 `ZCAN_ReceiveFD/Receive` → 解析：更新反馈缓存 / 完成 SDO 等待 / 触发 FrameReceived 事件（仅日志用） |
| keepalive 线程池定时器 | `System.Threading.Timer` 100ms：重发所有已登记 keepalive 帧（发送走传输层内部锁） |
| 反馈轮询定时器 | 50ms 发同步帧 0x80（与 keepalive 合并到一个定时器里交错发送亦可） |

所有 CAN 发送统一经 `CanFdTransport.Send()`（内部 `lock`），保证帧不被并发拆碎。
UI 更新一律 `BeginInvoke` 或直接由 UI 定时器拉缓存，禁止接收线程直接碰控件。

### 2.2 工程结构与文件清单

```
waist_motor_controller/
├── waist_motor_controller.csproj     # 修改：PlatformTarget=x64；dll 作为 Content 拷贝
├── native/ControlCANFD.dll           # 新增：从 qyh_waist_controller/canfd/windows/二次开发库V1.21/x64/ 复制
├── Native/ZlgCanNative.cs            # 新增：P/Invoke 与结构体（移植官方 c#_example_vs2022_v1.4 的 zlgcan.cs 精简版）
├── Can/CanFrame.cs                   # 新增：帧模型(id/len/data/isFd/flags) + 大端/小端读写助手
├── Can/CanFdTransport.cs             # 新增：打开/关闭/接收线程/发送/反馈缓存/SDO 响应路由
├── Protocol/ObjectDictionary.cs      # 新增：字典索引与模式编码常量（SDO 与 PDO 两套模式号分开）
├── Protocol/PdoBuilder.cs            # 新增：单轴帧/多控帧/同步帧构造
├── Protocol/SdoMessage.cs            # 新增：SDO 请求构造与响应解析（读/写/确认/abort）
├── Protocol/FeedbackParser.cs        # 新增：0x300 反馈解析 → MotorFeedback（按 len==12/16 分支）
├── Protocol/MotorFeedback.cs         # 新增：反馈数据类（位置/速度/电流/错误码/温度/模式/状态位/力矩?）
├── Protocol/FaultCodes.cs            # 新增：错误码位 → 中文名 + 清除方式
├── Control/MotorRegistry.cs          # 新增：4 逻辑电机 名称↔ID 映射 + config/motors.json 持久化
├── Control/KeepaliveScheduler.cs     # 新增：keepalive 帧登记/清除/100ms 重发
├── Control/MotorController.cs        # 新增：使能/失能/清错/标零/移动/点动/急停/扫描 时序
├── UI/MainForm.cs / .Designer.cs     # 修改：主界面（滚动面板 + 分组框）
├── UI/ChangeMotorIdDialog.cs         # 新增：更改电机 ID 对话框（图1样式）
└── config/motors.json                # 运行时生成：ID 映射与速度环默认参数
```

需复制的二进制：仅 `ControlCANFD.dll`（x64）。C# 走 P/Invoke，不需要 .lib/.h。
csproj 增加：

```xml
<PlatformTarget>x64</PlatformTarget>
<ItemGroup>
  <None Update="native\ControlCANFD.dll">
    <CopyToOutputDirectory>PreserveNewest</CopyToOutputDirectory>
  </None>
</ItemGroup>
```

---

## 3. UI 设计（主界面）

`MainForm` = `Panel(Dock=Fill, AutoScroll=true)` 内纵向堆叠 `FlowLayoutPanel(FlowDirection=TopDown, WrapContents=false, AutoSize=true)`，分组框按行排列、超出一屏自动出垂直滚动条（实现提示：FlowLayoutPanel 内 GroupBox 不会自动占满宽度，需统一设置 Width 或改用单列 TableLayoutPanel）。顺序如下：

### 3.1 框栏一「CANFD连接」

- 显示项（Label 行）：
  - 连接状态：`未连接 / 已连接`（带颜色圆点：灰/绿）；
  - 总线参数（即“总线ID”信息）：设备类型 `USBCANFD-200U(type=41)`（固定显示）、**设备索引与通道为 NumericUpDown（默认 0/0，存于 json，打开设备前可改）**、波特率 `仲裁1M/数据5M`、终端电阻 `开`；
  - 收/发帧计数（调试用）。
- 按钮：
  - **[打开设备]/[关闭设备]**（同一按钮切换文案）：打开 = `ZCAN_OpenDevice(41,0,0)` → `GetIProperty` 设置 `0/canfd_standard=0`、`0/canfd_abit_baud_rate=1000000`、`0/canfd_dbit_baud_rate=5000000`、`0/initenal_resistance=1` → `ZCAN_InitCAN(TYPE_CANFD)` → `ZCAN_StartCAN` → 启动接收线程与定时器；任一步失败回滚并弹窗提示。关闭 = **先执行 5.6 急停**（广播失能 + DisarmAll，否则电机将在 500ms 后以看门狗自锁方式非受控抱闸）→ 停定时器 → `ResetCAN/ReleaseIProperty/CloseDevice`。
  - **[更改电机ID]**：打开 `ChangeMotorIdDialog`（见 3.4）。
  - **[急停]**（红色大按钮）：见 5.6。
- 附加（调试用，放本框栏右下角小按钮）：[扫描总线ID]（SDO 遍历 1..127 读 0x6064，列出在线 ID，结果写入日志并可用于回填映射框栏）。

### 3.2 框栏二「电机ID映射」

4 行，每行：`Label(电机名固定)` + `NumericUpDown(ID, 1..127)` + `Label(在线状态：最近 500ms 内是否收到该 ID 反馈，绿/灰)`。
- 电机名固定：电机1-偏航 / 电机2-折叠上部 / 电机3-折叠中部 / 电机4-折叠下部；默认 ID 11/12/13/14。
- [保存映射] 按钮：写 `config/motors.json`；ID 修改立即生效于控制层（同时清除该电机 keepalive 与反馈缓存引用）。
- 启动时自动加载 json；无文件则用默认值并生成文件。

### 3.3 框栏三「单电机控制」（左右二分）

**左半：状态区**
- `ComboBox` 选择逻辑电机（4 个名称）。
- 状态标签（50ms 刷新，取自该电机当前 ID 的反馈缓存）：
  当前位置 `xx.x° (raw)`、当前速度 `xxx RPM`、控制模式 `PP/PV/...`、电流 `xxx mA`、
  错误码 `0x0000 无 / 0x0008 堵转(重启清除)`、线圈温度 `xx.x °C`、
  （16 字节力矩版反馈时）力矩 `x.xx Nm`，
  状态位：使能/抱闸释放/报错/到位 四个小指示灯。
- 无反馈缓存时显示 `--（未收到反馈）`。

**右半：控制区**
- 按钮行：[使能] [失能] [清除错误] [设置为原点]。
- 位置行：`NumericUpDown 目标角度(-180.00..180.00, 步进0.5, 2位小数)` + [移动到位置]。
- 点动组（GroupBox「速度环」，仿图2）：
  - `速度 NumericUpDown` 默认 **250** rpm（范围 1..2000；超出电机最大转速 0x6080（默认 2000RPM）时驱动器自动限幅）；
  - `加减速 NumericUpDown` 默认 **2000** rpm/s（范围 1..20000）；
  - 两个大按钮 [⟲ 反转] [⟳ 正转]：**按下(MouseDown)开始转，松开(MouseUp/MouseLeave/KeyUp)停止**；支持键盘按住（KeyDown 重复触发需去重）。

### 3.4 更改电机 ID 对话框（图1样式）

- 两行输入：`Current device ID:`（NumericUpDown 1..127，默认取框栏三当前选中电机的映射 ID，可改）、`New device ID:`（1..127，且与当前总线上其它 ID 不冲突的提示）。
- [OK]：SDO 写 `0x2530`（帧见 5.5），等待 0x580+cur 的 0x60 确认（超时 300ms 提示失败）；成功后提示"**新 ID 需电机断电重启后生效**"，并询问是否立即更新映射框栏中对应电机的 ID（仅更新界面与 json 建议值，不自动保存）。
- [Cancel]：关闭不做任何操作。

### 3.5 框栏四「日志」（附加，同样按行排在最后）

多行只读 TextBox + 清空按钮；记录：TX/RX 原始帧（ID+hex）、SDO 解码（位置/状态字/写确认/abort）、keepalive 启停、故障告警。**高频帧节流**：同步帧、0x300 反馈、keepalive 重发帧不逐帧记录（每 2s 汇总一条计数），否则 50ms×6 路反馈会刷屏；非映射 ID（如 9/10）的反馈仅计数不展示。日志限量 2000 行环形裁剪。

---

## 4. 传输层与 Native 层设计

### 4.1 P/Invoke（ZlgCanNative.cs）

直接移植官方 C# 样例（`c#_example_vs2022_v1.4/ZLGCAN/zlgcan.cs`）中需要的部分，保持结构体布局不变：
- 结构体：`CANFD`、`ZCAN_CHANNEL_INIT_CONFIG`(Explicit _union)、`canfd_frame`(data byte[64] ByValArray)、`can_frame`、`ZCAN_TransmitFD_Data`、`ZCAN_ReceiveFD_Data`、`ZCAN_Transmit_Data`、`ZCAN_Receive_Data`、`IProperty`(3 个 Cdecl 函数指针委托：SetValue/GetValue/GetPropertys)。
- 函数：`ZCAN_OpenDevice / ZCAN_CloseDevice / GetIProperty / ReleaseIProperty / ZCAN_InitCAN / ZCAN_StartCAN / ZCAN_ResetCAN / ZCAN_Transmit / ZCAN_TransmitFD / ZCAN_GetReceiveNum / ZCAN_Receive / ZCAN_ReceiveFD`。
- 常量：`TYPE_CAN=0, TYPE_CANFD=1, ZCAN_USBCANFD_200U=41, STATUS_OK=1`；CANFD BRS flag=`0x01`（发送 FD 帧置 `frame.flags=CANFD_BRS` 以启用 5M 数据段）。
- **注意**：`IProperty` 委托实例必须保存为字段防止 GC 回收回调指针。

### 4.2 CanFdTransport

- `bool Open(out string err)`：按 3.1 顺序调用；`Open` 成功后启动接收线程、keepalive 定时器、同步轮询定时器。
- `void Close()`：停定时器 → 停接收线程(join) → ResetCAN → ReleaseIProperty → CloseDevice。
- `bool Send(CanFrame f)`：加锁；`f.IsFd ? ZCAN_TransmitFD : ZCAN_Transmit`（Marshal.AllocHGlobal + StructureToPtr，用后立即 Free）。
- 接收线程循环（10ms 空闲休眠）：
  1. 读 TYPE_CANFD 队列与 TYPE_CAN 队列（批量 ≤200）；
  2. 每帧：若 ID∈(0x300,0x37F] 且 len∈{12,16} → `FeedbackParser`（内部按 len 分支：12 字节版 [10]模式/[11]状态位；16 字节力矩版 [10-11]力矩(0.05Nm)、[15]高4bit状态+低4bit模式）→ 写入 `FeedbackCache[deviceId]`（ConcurrentDictionary 或锁保护数组）；若 ID∈(0x580,0x5FF] → 交给 `SdoPendingQueue` 完成等待；若 ID∈[0x081,0x0FF] → 记录紧急报错帧（EMCY）日志；若 ID∈(0x700,0x77F] → 记录节点上线（boot）日志；其余仅触发 `FrameReceived` 事件（日志）。
- `Task<SdoResponse> SdoRequestAsync(CanFrame req, int timeoutMs=300)`：登记 TCS（key=响应ID）→ 发送 → 超时 abort；**同一设备 ID 的 SDO 请求串行排队**（响应仅按 0x580+ID 匹配，同 ID 并发请求会串扰）。供标零/改ID/扫描/读状态字使用。
- 反馈缓存对外只读：`MotorFeedback? GetFeedback(int deviceId)`。

---

## 5. 协议层与控制层设计

### 5.1 帧构造常量（PdoBuilder）

单轴帧（ID=0x100+devId, DLC=7, FD+BRS）：
```
B0 = (enable?0x80:0) | (brakeRelease?0x40:0) | (clearErr?0x20:0) | (mode<<1)
B1,B2 = 目标参数1（大端 int16）   // PP:位置raw；PV:速度rpm
B3,B4 = 目标参数2（大端 u16）     // PP/PV:加减速 rpm/s
B5,B6 = 前馈（大端 u16）          // PP:输出端轮廓速度 rpm；PV:0
```
多控帧（ID=0x200, DLC=64, FD+BRS）：slot_i = 单轴 7 字节 @ [7i..7i+6]；[56+i]=devId_i；其余 0。
同步帧（ID=0x80, DLC=0, 经典CAN）。
发送帧类型约定（与 Qt 版真机验证一致）：**单轴帧与多控帧用 CANFD(BRS) 帧发送；SDO 帧与同步帧用经典 CAN 帧发送**——说明书 SDO 全部示例为 8 字节经典帧，两种帧型电机均可接收，切勿混淆帧类型与字节序。

角度↔raw：`raw = clamp(round(deg*65536/360), -32768, 32767)`；`deg = raw*360/65536`。

### 5.2 反馈轮询

连接建立后每 50ms 发一次同步帧 0x80 → 4 电机各回 0x300+ID。
（ID 9/10 的反馈同样会到，解析层按 ID 缓存、UI 只读映射 ID，天然免疫。）

### 5.3 keepalive（KeepaliveScheduler）

- `Arm(devId, CanFrame)` / `Disarm(devId)` / `DisarmAll()`；100ms 定时器遍历重发已登记帧。
- 登记时机：使能成功、移动成功、点动按下/松开（速度0帧）。
- 清除时机：失能、标零流程开始、急停、收到该电机报错反馈（hasError 或 faultCode≠0）、关闭设备、映射 ID 变更。清除错误帧为一次性发送，不触碰 keepalive 状态（报错反馈到达时该电机 keepalive 已自动停止）。
- 与 Qt 版一致：单轴 keepalive 与广播 keepalive 互斥；本方案只用单轴 keepalive（4 电机各自一帧，100ms 共 4 帧，总线负载可忽略）。

### 5.4 单电机操作时序（MotorController）

**使能**：
1. 无该电机反馈缓存 → 拒绝并提示"先等待反馈/点击扫描"；
2. 有报错 → 拒绝并提示先清错（重启清除类故障提示断电重启）；
3. 构造 PP hold 帧：`enable=1, brake=1, mode=PP, target=当前反馈raw, 加减速=界面值, 轮廓速度=config.ppProfileSpeedRpm`；
4. 发送 + `Arm`；记录使能时刻 T0；
5. **T0+150ms 内禁止移动/点动**（协议：上伺服后 100ms 才可转；UI 按钮短暂禁用实现）。

**失能**：发 `B0=0` 的单轴帧（其余字节 0）→ `Disarm`。

**清除错误**：发 `B0=0x20` 单轴帧（一次性，不 Arm）；若故障属"重启清除"类，日志与 UI 提示需断电重启。

**设置为原点**（严格按说明书"零位标定步骤"五步执行。说明书标题注"设置时电机需要在失能状态"，但其五步流程是使能运行到位置后先标定再去使能；字典表又注明"当前代码允许使能状态下设置零位，但故障状态不允许"。本实现采用使能态标定 + 先停 keepalive 的安全组合）：
1. 前置：电机已使能并运行到目标零位（可用“移动到位置”完成）。若点击按钮时电机处于**失能态**（位置已就位），可直接从第 3 步开始执行（Qt 版真机验证过的失能态标零路径），完成第 4~5 步后提示用户手动重新使能；
2. `Disarm` 该电机 keepalive（**必须先停**：标定后坐标系平移，旧目标帧若被重发会让电机跳向非零目标——Qt 版注释中的真机教训）；
3. SDO 写 `0x2531=1`（`23 31 25 00 01 00 00 00`），等 0x60 确认；
4. 发同步帧并读反馈，确认当前位置=0，标定成功；
5. 失能（抱闸吸合），等 150ms，再次发送同一帧作为"标定复位指令"（说明书第4步）；
6. 重新使能（hold 目标=0），恢复 keepalive 与正常控制；全流程该电机按钮禁用。

**移动到位置（PP）**：
1. 校验：已使能（反馈 enabled=1）、无报错、过 T0+150ms、目标∈[-180,180]；
2. 构造 PP 帧：target=目标raw；加减速=界面“加减速”值（RPM/S）；轮廓速度=config.ppProfileSpeedRpm（默认 8，**输出端 RPM，范围 1~40（字典表 0x6081）**；勿与点动的电机端速度 250rpm 混淆）；
3. 发送一次（电机即开始规划运动）+ 将该帧 `Arm` 为 keepalive（Qt 版真机验证：PP 模式下重发同目标帧不会重复触发规划，仅喂看门狗）；
4. 到位判定用反馈 bit4（targetReached）在 UI 显示。
> 备选方案（若真机发现 keepalive 重发引起 PP 抖动）：移动改走 SDO 序列
> `0x6060=1 → 0x6083/0x6084 加减速（按字典表限 1~200 RPM/S，与说明书示例 2000 矛盾处真机验证） → 0x6081 速度（1~40） → 0x607A 目标 → 0x6040=0x4F→0x0F`，
> keepalive 改发“当前位置 hold 帧”。代码上 PdoBuilder/SdoMessage 均已具备，切换成本=一个方法。

**点动正/反转（PV）**：
- 按下：校验同移动；构造 PV 帧 `mode=0x02, 参数1=±speed(rpm), 参数2=加减速`；发送 + `Arm`（keepalive 持续重发=持续旋转）。
- 松开（MouseUp / MouseLeave / KeyUp / 窗体 Deactivate / 急停）：构造 PV 帧 `参数1=0` 发送并替换 keepalive 帧 → 电机按设定减速度停转且保持使能、不触发自锁。
- 按下期间禁用[移动到位置]/[使能]/[失能]等按钮防冲突。

### 5.5 SDO 操作表（SdoMessage + ObjectDictionary）

| 操作 | 请求帧（ID=0x600+devId, DLC8, 小端） | 响应/说明 |
|---|---|---|
| 读实际位置 | `40 64 60 00 00 00 00 00` | `0x580+id: 43 64 60 00 xx xx xx xx` int32 小端，deg=raw/182.04 |
| 读实际电流 | `40 78 60 00 ...` | int16 小端 mA（0x6078） |
| 读母线电压 | `40 79 60 00 ...` | u32 小端 0.1V（0x6079） |
| 读状态字 | `40 41 60 00 ...` | 解码 CiA402 状态 |
| 读模式显示 | `40 61 60 00 ...` | 1PP/3PV/8CSP/9CSV/10CST（SDO 编号） |
| 改电机 ID | `23 30 25 00 {newId} 00 00 00` | 0x60 确认；**重启生效** |
| 设零位 | `23 31 25 00 01 00 00 00` | 失能态或使能态均可（故障态不允许），时序见 5.4 |
| 关看门狗(调试) | `23 50 26 00 01 00 00 00` | 同时关限位保护，默认不用 |
| 清错(SDO 备路) | `2B 40 60 00 80 00 00 00` | 主路用 PDO bit5 |

**总线扫描**：对 ID=1..127 依次 `SdoRequestAsync(读0x6064, 超时15ms)`，任何响应（含 abort）即证明该 ID 在线；进度写日志，结果列表供映射框栏回填。

### 5.6 急停

1. 立即结束点动状态；
2. `DisarmAll()`（**先清 keepalive，防止定时器在失能后又重发使能帧**）；
3. 发 1 帧多控广播 0x200：4 个 slot 全为失能帧（B0=0），[56..59]=当前映射 4 个 ID（一帧覆盖 4 电机，最快）；若某逻辑电机未映射 ID 则补发单轴失能帧兜底；
4. UI 状态刷新、日志记录"急停"。
急停不尝试清错、不关设备，保持连接以便恢复。

### 5.7 错误码显示（FaultCodes）

按位分解 faultCode → 中文名 + 清除方式（重启清除/指令清除/恢复到85°以下指令清除）拼接显示；
"重启清除"位掩码 = `0x0001|0x0002|0x0008|0x0010|0x0020|0x0100|0x0400|0x0800|0x1000|0x2000`，
命中时使能/移动按钮禁用并提示断电重启（沿用 Qt 版 `kRestartClearFaultMask` 结论）。

---

## 6. 配置持久化（config/motors.json）

```json
{
  "can": { "deviceType": 41, "deviceIndex": 0, "channel": 0,
           "arbitrationBaud": 1000000, "dataBaud": 5000000, "resistance": true },
  "motors": [
    { "logicalId": 1, "name": "电机1-偏航",   "deviceId": 11 },
    { "logicalId": 2, "name": "电机2-折叠上部", "deviceId": 12 },
    { "logicalId": 3, "name": "电机3-折叠中部", "deviceId": 13 },
    { "logicalId": 4, "name": "电机4-折叠下部", "deviceId": 14 }
  ],
  "profile": { "jogSpeedRpm": 250, "jogAccelRpmPerSec": 2000,
               "ppProfileSpeedRpm": 8, "ppAccelRpmPerSec": 2000 }
}
```
启动加载、[保存映射] 与参数修改时写回（System.Text.Json）。

---

## 7. 风险与对策清单

| # | 风险 | 对策 |
|---|---|---|
| 1 | 电机 500ms 看门狗自锁 | keepalive 100ms 重发；任何"停止"语义都替换为速度0/hold 帧而非停发；若真机仍自锁（重发帧未被计为喂狗），回退：连接后对每电机 SDO 写 0x2650=1 关看门狗（会同时失去机上限位保护，慎用） |
| 2 | 使能瞬间跳变 | 使能帧目标=当前反馈位置；无反馈缓存禁止使能 |
| 3 | 上伺服 100ms 内下目标被拒/异常 | 使能后 150ms 窗口禁用运动按钮 |
| 4 | SDO/PDO 模式编号混淆（PV: 3 vs 2） | 常量表 `SdoMode`/`PdoMode` 分开命名，禁止裸数字 |
| 5 | 字节序混用（SDO 小端 / PDO 大端） | CanFrame 提供 `WriteBe16/WriteLe32` 助手，构帧处注释标明 |
| 6 | 点动松开未停转 | MouseUp+MouseLeave+KeyUp+Form.Deactivate 四路收敛；keepalive 换速度0帧 |
| 7 | 急停后 keepalive 复活使能 | 急停先 DisarmAll 再发失能帧（顺序写死在代码与注释中） |
| 8 | 标零坐标系平移导致跳动 | 标零前先 Disarm keepalive（防旧目标帧重发），严格按说明书零位标定五步流程执行 |
| 9 | 改 ID 后未重启不生效 | 对话框明确提示；扫描功能辅助确认 |
| 10 | DLL 位数/路径 | csproj 固定 x64 + CopyToOutputDirectory；启动时 File.Exists 检查并提示 |
| 11 | 总线 ID 9/10 干扰 | 反馈按 ID 缓存、控制只发映射 ID；同步帧触发其反馈无副作用 |
| 12 | UI 消息洪水 | 接收线程只写缓存，UI 50ms 拉取；日志框限量 |
| 13 | IProperty 委托被 GC | 委托与结构体实例保存为 Transport 字段 |
| 14 | PP keepalive 重发疑似重触发 | 预留 SDO PP 备选序列（5.4 注），单测覆盖两种构帧 |
| 15 | 关闭设备时电机仍使能 | Close 前先执行急停（广播失能），避免电机以看门狗自锁方式抱闸 |
| 16 | 同 ID 并发 SDO 请求响应串扰 | 同设备 SDO 请求串行排队（4.2） |

---

## 8. 实施里程碑

1. **M1 链路**：csproj x64 + dll 拷贝 + ZlgCanNative + Transport 打开/关闭/接收线程 + 日志框原始帧显示。验收：打开设备后发同步帧即可见 0x300+ID 反馈帧；若在电机上电 2.5s 内打开设备，还能捕获一次性的 0x700+ID 上线帧。
2. **M2 反馈**：FeedbackParser + 同步轮询 + 映射框栏在线灯 + 单电机状态区刷新。验收：状态区数值与 ZLG 上位机工具一致。
3. **M3 配置**：MotorRegistry + json 持久化 + 总线扫描回填。
4. **M4 单轴控制**：使能/失能/清错/标零/移动到位置 + keepalive + 使能窗口保护。验收：真机单轴 30°/-30° 往复无跳变、无自锁。
5. **M5 点动**：PV 正反转 + 速度/加减速参数 + 四路收敛。验收：按住转、松开停、松手不自锁。
6. **M6 安全与运维**：急停、更改ID对话框、错误码中文显示、重启清除类故障禁能提示。
7. **M7 联调**：4 轴同时使能 + 各自移动/点动 30min 无自锁；急停全停；断电重启恢复流程。

## 9. 真机联调检查单

- [ ] （打开设备先于电机上电时）上电 ≥2.5s 后捕获 4 个 0x700+ID 一次性启动帧；
- [ ] 打开设备后 100ms 内收到 4 路 0x300 反馈（同步帧触发）；
- [ ] 使能后反馈 enabled=1、brakeReleased=1，轴保持不跳；
- [ ] 移动 ±30° 收敛且 targetReached=1；keepalive 日志 100ms 周期；
- [ ] 点动按住连续转、松开按减速度停；停止后 10s 不自锁；
- [ ] 人为停发 keepalive（调试开关）600ms → 电机自锁，验证看门狗模型；
- [ ] 急停：4 轴同时失能抱闸；
- [ ] 改 ID：SDO 确认 0x60，断电重启后新 ID 反馈出现；
- [ ] 标零（五步流程）：移动到目标零位→标定→反馈=0→失能+复位指令→重新使能；失能态快捷路径亦可；
- [ ] 报错注入（限位/堵转）：UI 显示中文故障与清除方式，keepalive 自动停止。

---

## 附录A：典型帧示例（hex）

| 场景 | ID | DLC | Data |
|---|---|---|---|
| 同步(反馈轮询) | 0x080 | 0 | — |
| 使能hold(dev11, 当前位置0x09E1, 加减速2000, 轮廓速度8) | 0x10B | 7 | C2 09 E1 07 D0 00 08 |
| 移动到90°(dev11) | 0x10B | 7 | C2 40 00 07 D0 00 08 |
| 点动正转250rpm(DEV11) | 0x10B | 7 | C4 00 FA 07 D0 00 00 |
| 点动停止(速度0) | 0x10B | 7 | C4 00 00 07 D0 00 00 |
| 失能 | 0x10B | 7 | 00 00 00 00 00 00 00 |
| 清除错误 | 0x10B | 7 | 20 00 00 00 00 00 00 |
| 急停广播(11/12/13/14) | 0x200 | 64 | 00×56 + 0B 0C 0D 0E + 00×4 |
| 改ID 11→15 | 0x60B | 8 | 23 30 25 00 0F 00 00 00 |
| 设零位 | 0x60B | 8 | 23 31 25 00 01 00 00 00 |
| 读位置 | 0x60B | 8 | 40 64 60 00 00 00 00 00 |
| 反馈(dev11：13° / -539RPM / -175mA / 无错 / 24.0°C / PP / 使能+抱闸释放) | 0x30B | 12 | 09 E1 FD E5 FF 51 00 00 00 F0 01 C0 |

（控制字 C2 = 使能|抱闸释放|PP(0x01<<1)；C4 = 使能|抱闸释放|PV(0x02<<1)。）
