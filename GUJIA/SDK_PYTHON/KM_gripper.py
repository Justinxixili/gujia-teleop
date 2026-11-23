"""ROS2 订阅 /joint_states 的位置控制脚本.

修改：
1. 原先固定往返控制改为从 /joint_states 话题获取目标位置 (pos_des)。
2. 使用 Timer 以固定频率(默认50Hz)下发 control_pos_force。
3. 加入软限位 (基于初始化时扫描得到的 min/max)，防止越界。
4. 优雅关闭：节点销毁时关闭串口。
"""

import math
import time
# 安装后入口点方式运行需要包内相对导入；源码直接运行 fallback
try:
    from .KM_CAN import *  # type: ignore  # noqa
    from .fx_robot import Marvin_Robot  # type: ignore  # noqa
except ImportError:
    from KM_CAN import *  # type: ignore  # noqa
    from fx_robot import Marvin_Robot  # type: ignore  # noqa

# 全局（保持与原结构兼容）
Motor1, Motor2, grippers = None, None, KMGripperControl()
Motor1_min_pos, Motor1_max_pos = 0.0, 0.0
Motor2_min_pos, Motor2_max_pos = 0.0, 0.0
auto_calibrate = False


class GripperMotorNode():
    def __init__(self):
        self.q1 = 0.0
        self.q2 = 0.0  # Motor2 目标位置缓存
        self.vel_cmd = 0
        self.i_des = 0
        self.current_desired_pos_m1 = 0.0
        self.current_desired_pos_m2 = 0.0
        self.have_joint_state = False
        self.left_id = 1
        self.right_id = 2
        self.robot = Marvin_Robot()
        self.robot.connect('192.168.1.190')
        self._init_motors()
        print('Gripper motor node started. Waiting joint states...')

    def reset_motors_callback(self, request, response):
        grippers.disable(Motor1)
        grippers.disable(Motor2)
        time.sleep(0.1)
        grippers.enable(Motor1)
        grippers.enable(Motor2)
        response.success = True
        response.message = "Gripper motors have been reset."
        return response

    def control_timer_callback(self):
        self.q1 = 0.5+math.sin(time.time())*0.5  # 模拟数据，范围0~1
        self.q2 = 0.5+math.sin(time.time())*0.5
        grippers.controlMIT(Motor1, 3.0, 0.12, self.q1, self.vel_cmd, self.i_des)
        grippers.controlMIT(Motor2, 3.0, 0.12, self.q2, self.vel_cmd, self.i_des)
        grippers.recv()

        
        q_fb_R = Motor2.getPosition()
        print(f"GripperR pos_fb:{q_fb_R:.4f}")
        dq_fb_R = Motor2.getVelocity()
        tau_fb_R = Motor2.getTorque()
        err_fb_R = Motor2.getErr()
        temp_mos_fb_R = Motor2.getTempMos()
        temp_motor_fb_R = Motor2.getTempMotor()

        q_fb_L = Motor1.getPosition()
        print(f"GripperL pos_fb:{q_fb_L:.4f}")
        dq_fb_L = Motor1.getVelocity()
        tau_fb_L = Motor1.getTorque()
        err_fb_L = Motor1.getErr()
        temp_mos_fb_L = Motor1.getTempMos()
        temp_motor_fb_L = Motor1.getTempMotor()
        
        
        
        # self.get_logger().info(f'GripperR pos_fb:{q_fb:.4f}')

    # ---------------- 初始化与标定 ----------------
    def _init_motors(self):
        global Motor1, Motor2, grippers
        grippers = KMGripperControl(robot=self.robot)
        Motor1 = Motor(KM_Motor_Type.DM4310, self.left_id, self.left_id+0x10)
        Motor2 = Motor(KM_Motor_Type.DM4310, self.right_id, self.right_id+0x10)

        grippers.addMotor(Motor1)
        grippers.add_to_ch(Motor1, 'left')

        grippers.addMotor(Motor2)
        grippers.add_to_ch(Motor2, 'right')
        
        grippers.enable(Motor1)
        # self.motor1_send()
        grippers.enable(Motor2)
        # self.motor2_send()
        print("Gripper motors initialized.")
        # MotorControl1.set_zero_position(Motor1)
        # MotorControl1.set_zero_position(Motor2)
    def _clamp(self, v, vmin, vmax):
        v = v*(vmax - vmin) + vmin
        return v
    def destroy_node(self):  # 资源释放
        return super().destroy_node()

if __name__ == '__main__':  # 兼容直接运行
    gripper_node = GripperMotorNode()
    while True:
        gripper_node.control_timer_callback()
