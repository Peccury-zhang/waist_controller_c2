namespace waist_motor_controller.UI;

/// <summary>运动中的短暂状态不改变按钮Enabled；业务层仍检查重入、到位和使能保护期。</summary>
internal readonly record struct OperationUiState(
    bool Connected, bool FeedbackFresh, bool Faulted, bool Configuring,
    bool Armed, bool MotorEnabled, bool ReadingFirmware, bool InEnableWindow,
    bool Moving, bool AtTarget, bool CommandClearable = true)
{
    public bool CanCommand => Connected && FeedbackFresh && !Faulted && !Configuring && Armed && MotorEnabled;
    public string Hint => !Connected ? ""
        : !FeedbackFresh ? "等待选中电机的有效反馈，请检查连接。"
        : Faulted ? "电机存在故障，请查看故障详情及清除方式。"
        : Configuring ? "正在处理电机配置，请稍候。"
        : !MotorEnabled || !Armed ? "电机未通过本程序使能；使能后可定位或点动。"
        : InEnableWindow ? "使能保护期内，请稍候。"
        : Moving ? "点动 / 定位进行中，可随时停止；到位后可再次操作。"
        : "已就绪 · 点动每点击一次，增减指定角度。";
}
