"""ROS 2 Action 範例：接收兩個參數陣列，回報執行進度與結果。"""
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from interfaces.action import ExecuteTaskPlan


class TaskPlanServer(Node):
    def __init__(self):
        # 建立 ROS 2 節點與 Action Server，名稱必須與 client 相同。
        super().__init__('task_plan_server')
        self._server = ActionServer(
            self, ExecuteTaskPlan, '/manip/pick_place',
            execute_callback=self.execute,
            goal_callback=self.accept_goal,
        )
        self.get_logger().info('等待請求：/manip/pick_place')

    def accept_goal(self, request):
        # 兩個陣列以相同索引對應，因此長度必須一致，且至少有一組資料。
        keys = request.subtask.parameter_keys
        values = request.subtask.parameter_values
        if not keys or len(keys) != len(values) or any(not key.strip() for key in keys):
            self.get_logger().warning('拒絕請求：參數名稱不可空白，且兩個陣列長度必須相同。')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        # 分別取出兩個陣列，不轉成字典，也不把名稱當成內建控制指令。
        parameter_keys = list(goal_handle.request.subtask.parameter_keys)
        parameter_values = list(goal_handle.request.subtask.parameter_values)
        self.get_logger().info(f'parameter_keys: {parameter_keys}')
        self.get_logger().info(f'parameter_values: {parameter_values}')

        # 同一個索引代表一組資料，例如 keys[0] 是 Grasp、values[0] 是 water_bottle。
        # 實際應用可在這裡依照需求使用兩個陣列，替換下方的模擬執行。
        result = ExecuteTaskPlan.Result()
        try:
            # 固定模擬執行約 5 秒；status 固定，progress 從 0.0 更新至 1.0。
            for step in range(11):
                if not rclpy.ok():
                    return result
                feedback = ExecuteTaskPlan.Feedback()
                feedback.status = 'working normally'
                feedback.progress = step / 10.0
                goal_handle.publish_feedback(feedback)
                if step < 10:
                    time.sleep(0.5)

            # 標記任務成功，並回傳自訂 Action 中定義的 Result 欄位。
            goal_handle.succeed()
            result.success = True
            result.error_code = 0
            result.message = (
                f'parameter_keys={parameter_keys}; '
                f'parameter_values={parameter_values}'
            )
        except Exception as exc:
            # 執行發生例外時，讓 client 收到失敗結果。
            goal_handle.abort()
            result.success = False
            result.error_code = 1
            result.message = str(exc)
        return result


def main(args=None):
    # 初始化 ROS 2，持續處理請求；此簡單範例依序執行任務。
    rclpy.init(args=args)
    node = TaskPlanServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 結束時釋放 Action Server、節點及 ROS 2 資源。
        node._server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
