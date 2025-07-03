import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys

class GripperMover(Node):
    def __init__(self):
        super().__init__('gripper_mover_node')
        self._action_client = ActionClient(self, GripperCommand, '/gripper_position_controller/gripper_cmd')

    def send_goal(self, position, max_effort):
        """アクションサーバーにゴールを送信する関数"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal to move gripper to position: {position}')
        # send_goal_asyncは非同期でゴールを送信し、すぐに完了を待たずに次に進む
        # ゴールの完了を待ちたい場合は、返ってきたgoal_handleを使う
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal sent.')


def main(args=None):
    rclpy.init(args=args)

    # コマンドライン引数から目標位置を取得
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> gripper_mover.py <position>")
        print("Example: ros2 run my_package gripper_mover.py 0.8")
        return

    try:
        # 第1引数をfloat型に変換
        target_position = float(sys.argv[1])
    except ValueError:
        print("Error: Position must be a number.")
        return

    # ノードを作成してゴールを送信
    action_client = GripperMover()
    action_client.send_goal(position=target_position, max_effort=-1.0) # max_effort=-1.0は制限なし

    # ゴールを送信したら少し待ってから終了
    # 本来はゴールの完了を待つべきだが、シンプルな例として即時終了しないようにする
    try:
        rclpy.spin_once(action_client, timeout_sec=5.0)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()