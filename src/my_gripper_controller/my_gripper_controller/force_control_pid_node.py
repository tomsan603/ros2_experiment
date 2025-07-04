import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand

from simple_pid import PID

class GripperForceController(Node):
    def __init__(self):
        super().__init__('gripper_force_pid_controller')

        # パラメータの宣言 (ゲインと目標値を動的に変更可能にする)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('force_setpoint', 5.0) # 目標とする力 [N・m] (要調整)
        self.declare_parameter('gripper_joint_name', 'robotiq_85_left_knuckle_joint')


        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.force_setpoint = self.get_parameter('force_setpoint').get_parameter_value().double_value

        # PIDコントローラーの初期化
        self.pid = PID(self.kp, self.ki, self.kd, setpoint=self.force_setpoint)
        self.pid.output_limits = (-0.05, 0.05) # 1ステップあたりの位置変化量の上限 (要調整)

        
        # パラメータを取得
        self._update_params()


        # 状態変数の初期化
        self.current_effort = 0.0
        self.current_position = 0.0
        self.joint_name = self.get_parameter('gripper_joint_name').get_parameter_value().string_value
        self.joint_index = -1
        self.data_received = False

        # サブスクライバ (関節の状態を取得)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # アクションクライアント (グリッパーの位置指令を送信)
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd')

        # 制御ループのためのタイマー
        self.timer = self.create_timer(0.02, self.control_loop) # 50Hzで制御

        self.get_logger().info('Gripper Force PID Controller has been started.')

    def _update_params(self):
        """パラメータを読み込み、PIDコントローラを更新する"""
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.force_setpoint = self.get_parameter('force_setpoint').get_parameter_value().double_value

        self.pid.tunings = (self.kp, self.ki, self.kd)
        self.pid.setpoint = self.force_setpoint
        self.get_logger().info(f"PID updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, Setpoint={self.force_setpoint}")


    def joint_state_callback(self, msg: JointState):
        """/joint_statesトピックから現在の関節トルクと位置を取得"""
        if not self.data_received:
            try:
                self.joint_index = msg.name.index(self.joint_name)
                self.data_received = True
            except ValueError:
                # self.get_logger().warn(f"'{self.joint_name}' not found in joint states yet.")
                return

        self.current_effort = abs(msg.effort[self.joint_index]) # 常に正の値として扱う
        self.current_position = msg.position[self.joint_index]
        # self.get_logger().info(f"Effort: {self.current_effort:.2f}, Pos: {self.current_position:.2f}")

    def control_loop(self):
        """PID制御計算と指令送信を行うループ"""
        if not self.data_received:
            self.get_logger().warn("No joint state data received yet. Skipping control loop.")
            return

        # パラメータの変更をチェック
        self._update_params()

        # PID制御量の計算
        # 目標値より力が大きい -> 正の制御量 -> グリッパーを少し開く
        # 目標値より力が小さい -> 負の制御量 -> グリッパーを少し閉じる
        control_output = self.pid(self.current_effort)

        # 新しい目標位置の計算
        # 制御量が負（閉じる方向）なら、現在の位置から引く
        new_position_goal = self.current_position - control_output

        # 関節の可動範囲内に収める
        # URDFより: lower="0.0" upper="0.8"
        new_position_goal = max(0.0, min(new_position_goal, 0.8))

        # self.get_logger().info(f"Current Effort: {self.current_effort:.2f} -> PID output: {control_output:.4f} -> New Pos Goal: {new_position_goal:.4f}")

        # アクションゴールを作成して送信
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = new_position_goal
        goal_msg.command.max_effort = 100.0  # この値は大きくても問題ない

        self.gripper_action_client.wait_for_server(timeout_sec=1.0)
        self.gripper_action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperForceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()