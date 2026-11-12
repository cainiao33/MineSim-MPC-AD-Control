config = {
    "test_settings": {  # 测试设置
        "mode": "replay",  # "interaction"为双向交互仿真测试模块;'replay' 回放测试
        "visualize": False,  # 默认关闭可视化；命令行 --visualize 可以覆盖
        "save_fig_whitout_show": False,  # visualize=True 时，是否仅保存逐帧 PNG 而不显示窗口
        "skip_exist_scene": False,  # 是否跳过outputs中已有记录的场景(可以避免程序异常中断后重新测试之前已经完成的场景)
    },
}
