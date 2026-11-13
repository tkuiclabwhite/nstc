"""使用真實 ROS 2 server 驗證陣列傳輸、Feedback、Result 與 GUI。"""
import os
from pathlib import Path
import signal
import subprocess
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from interfaces.action import ExecuteTaskPlan


class DemoTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.addClassCleanup(rclpy.shutdown)
        cls.node = Node('task_plan_integration_test')
        cls.addClassCleanup(cls.node.destroy_node)
        log = Path(__file__).parent.parent / 'log/test_server.log'
        log.parent.mkdir(parents=True, exist_ok=True)
        cls.server_log = log.open('w')
        cls.addClassCleanup(cls.server_log.close)
        cls.server = subprocess.Popen(
            ['ros2', 'run', 'task_plan_demo', 'server'],
            stdout=cls.server_log, stderr=subprocess.STDOUT, start_new_session=True)
        cls.addClassCleanup(cls.stop_server)
        cls.client = ActionClient(cls.node, ExecuteTaskPlan, '/manip/pick_place')
        if not cls.client.wait_for_server(timeout_sec=15):
            raise RuntimeError('Server 未啟動，請查看 log/test_server.log')

    @classmethod
    def stop_server(cls):
        if cls.server.poll() is None:
            os.killpg(cls.server.pid, signal.SIGINT)
            try:
                cls.server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(cls.server.pid, signal.SIGKILL)
                cls.server.wait()

    def wait(self, future):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)
        self.assertTrue(future.done(), 'ROS 操作逾時')
        return future.result()

    @staticmethod
    def request(keys, values):
        goal = ExecuteTaskPlan.Goal()
        goal.subtask.parameter_keys = keys
        goal.subtask.parameter_values = values
        return goal

    def test_01_arrays_feedback_result(self):
        # 重複名稱不會因字典轉換而遺失；duration 與 fail 也只是一般字串。
        keys = ['Grasp', 'Grasp', 'duration', 'fail']
        values = ['water_bottle', 'cup', '=', 'true']
        feedback = []
        handle = self.wait(self.client.send_goal_async(
            self.request(keys, values), feedback_callback=lambda msg: feedback.append(msg.feedback)))
        self.assertTrue(handle.accepted)
        response = self.wait(handle.get_result_async())
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.success)
        self.assertEqual(response.result.error_code, 0)
        self.assertIn(f'parameter_keys={keys}', response.result.message)
        self.assertIn(f'parameter_values={values}', response.result.message)
        self.assertGreater(len(feedback), 1)
        self.assertTrue(all(msg.status == 'working normally' for msg in feedback))
        self.assertAlmostEqual(feedback[-1].progress, 1.0)

    def test_02_invalid_arrays_rejected(self):
        for keys, values in [([], []), (['Grasp'], []), ([''], ['water_bottle'])]:
            handle = self.wait(self.client.send_goal_async(self.request(keys, values)))
            self.assertFalse(handle.accepted)

    @unittest.skipUnless(os.environ.get('DISPLAY'), '需要圖形顯示環境')
    def test_03_gui_separate_arrays(self):
        import tkinter as tk
        from task_plan_demo.client_gui import ClientWindow
        root = tk.Tk()
        root.withdraw()
        node = Node('gui_integration_test')
        window = ClientWindow(root, node)
        try:
            self.assertTrue(window.client.wait_for_server(timeout_sec=10))
            request = window.build_request()
            self.assertEqual(request.subtask.parameter_keys, ['task_id', 'bottle_class', 'arm', 'place_target', 'timeout_s'])
            self.assertEqual(request.subtask.parameter_values, ['T-20261113-0001', 'b1', 'auto', 'tray_1', '30'])
            window.keys.insert('end', '\nPlace')
            with self.assertRaises(ValueError):
                window.build_request()
            window.values.insert('end', '\ntable')
            request = window.build_request()
            self.assertEqual(request.subtask.parameter_keys, ['task_id', 'bottle_class', 'arm', 'place_target', 'timeout_s', 'Place'])
            self.assertEqual(request.subtask.parameter_values, ['T-20261113-0001', 'b1', 'auto', 'tray_1', '30', 'table'])
            window.send()
            deadline = time.monotonic() + 15
            while window.busy and time.monotonic() < deadline:
                root.update()
                time.sleep(0.01)
            self.assertFalse(window.busy)
            text = window.output.get('1.0', 'end')
            self.assertIn('Feedback: working normally; progress=', text)
            self.assertIn('success=True', text)
            self.assertIn("parameter_keys=['task_id', 'bottle_class', 'arm', 'place_target', 'timeout_s', 'Place']", text)
            self.assertIn("parameter_values=['T-20261113-0001', 'b1', 'auto', 'tray_1', '30', 'table']", text)
            self.assertEqual(window.status.get(), '完成')
            self.assertAlmostEqual(window.progress['value'], 100)
            if os.environ.get('DEMO_SCREENSHOT'):
                from PIL import ImageGrab
                root.deiconify()
                root.update()
                time.sleep(0.2)
                root.update()
                x, y = root.winfo_rootx(), root.winfo_rooty()
                ImageGrab.grab(bbox=(x, y, x + root.winfo_width(), y + root.winfo_height())).save(os.environ['DEMO_SCREENSHOT'])
        finally:
            window.close()
            node.destroy_node()


if __name__ == '__main__':
    unittest.main(verbosity=2)
