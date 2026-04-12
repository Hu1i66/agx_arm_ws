import time
from local_sdk_bootstrap import ensure_workspace_sdk_on_path

ensure_workspace_sdk_on_path()

from piper_sdk import C_PiperInterface

def activate_can():
    print("Connecting to CAN0...")
    piper = C_PiperInterface("can0")
    piper.ConnectPort()
    
    # 按照Windows配置流程模拟状态机切换
    print("\n--- 机械臂唤醒序列 ---")
    
    # 步骤 1
    print("[步骤 1/3] 正在向机械臂发送【待机模式(Standby)】指令 (0x00)...")
    piper.MotionCtrl_2(0x00, 0x01, 50)
    time.sleep(1.0)
    print("-> 待机模式指令已发送。")
    
    # 步骤 2
    print("\n[步骤 2/3] 正在向机械臂发送【CAN控制模式】指令 (0x01)...")
    piper.MotionCtrl_2(0x01, 0x01, 50)
    time.sleep(1.0)
    print("-> CAN控制模式指令已发送。")

    # 步骤 3
    print("\n[步骤 3/3] 正在【使能 (Enable)】电机接管控制权...")
    piper.EnableArm(7)
    time.sleep(1.0)
    print("-> 使能指令已发送。")
    
    print("\n✅ 激活协议发送完毕！您可以检查 Candump 确认心跳包，或者运行ROS2节点了。")

if __name__ == "__main__":
    try:
        activate_can()
    except KeyboardInterrupt:
        print("\n操作已被用户取消。")
