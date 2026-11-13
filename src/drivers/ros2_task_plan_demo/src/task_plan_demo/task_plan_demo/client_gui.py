"""簡單的 Action client：分別輸入兩個陣列，顯示 Feedback 與 Result。"""
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interfaces.action import ExecuteTaskPlan


class ClientWindow:
    def __init__(self, root, node):
        self.root, self.node = root, node
        self.client = ActionClient(node, ExecuteTaskPlan, '/manip/pick_place')
        self.busy = False
        self.closing = False
        root.title('ROS 2 Task Plan Client')
        root.geometry('740x600')
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill='both', expand=True)
        frame.columnconfigure((0, 1), weight=1)
        ttk.Label(frame, text='每行一筆，左右兩欄依行序對應').grid(row=0, columnspan=2, sticky='w', pady=(0, 8))
        # 子三取放任務的欄位（S3-05 IF-SP3-001）；arm 可填 auto / left / right
        self.keys = self.input_box(frame, 'parameter_keys', 0,
                                   'task_id\nbottle_class\narm\nplace_target\ntimeout_s')
        self.values = self.input_box(frame, 'parameter_values', 1,
                                     'T-20261113-0001\nb1\nauto\ntray_1\n30')
        self.send_button = ttk.Button(frame, text='送出請求', command=self.send, state='disabled')
        self.send_button.grid(row=3, columnspan=2, sticky='w', pady=12)
        self.status = tk.StringVar(value='等待 server…')
        ttk.Label(frame, textvariable=self.status).grid(row=4, columnspan=2, sticky='w')
        self.progress = ttk.Progressbar(frame, maximum=100)
        self.progress.grid(row=5, columnspan=2, sticky='ew', pady=8)
        ttk.Label(frame, text='Feedback / Result').grid(row=6, columnspan=2, sticky='w')
        self.output = ScrolledText(frame, height=14, state='disabled')
        self.output.grid(row=7, columnspan=2, sticky='nsew', pady=4)
        frame.rowconfigure(7, weight=1)
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    @staticmethod
    def input_box(frame, label, column, initial):
        ttk.Label(frame, text=label).grid(row=1, column=column, sticky='w')
        box = ScrolledText(frame, height=5, width=28)
        box.grid(row=2, column=column, sticky='ew', padx=(0, 8) if column == 0 else (8, 0))
        box.insert('1.0', initial)
        return box

    def log(self, text):
        self.output.configure(state='normal')
        self.output.insert('end', text + '\n')
        self.output.see('end')
        self.output.configure(state='disabled')

    def tick(self):
        if self.closing:
            return
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        ready = self.client.server_is_ready()
        self.send_button.configure(state='normal' if ready and not self.busy else 'disabled')
        if not self.busy and not ready:
            self.status.set('等待 server…')
        elif ready and self.status.get() == '等待 server…':
            self.status.set('Server 已連線，可以送出請求')
        self.root.after(30, self.tick)

    def build_request(self):
        keys = self.keys.get('1.0', 'end-1c').split('\n')
        values = self.values.get('1.0', 'end-1c').split('\n')
        if any(not key.strip() for key in keys):
            raise ValueError('parameter_keys 每行必須有名稱')
        if len(keys) != len(values):
            raise ValueError('parameter_keys 與 parameter_values 的行數必須相同')
        goal = ExecuteTaskPlan.Goal()
        goal.subtask.parameter_keys = keys
        goal.subtask.parameter_values = values
        return goal

    def send(self):
        if self.busy or not self.client.server_is_ready():
            return
        try:
            goal = self.build_request()
            self.busy = True
            self.send_button.configure(state='disabled')
            self.progress['value'] = 0
            self.status.set('等待 server 接受請求…')
            self.log(f'\nparameter_keys: {list(goal.subtask.parameter_keys)}')
            self.log(f'parameter_values: {list(goal.subtask.parameter_values)}')
            self.client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
        except ValueError as exc:
            messagebox.showerror('輸入錯誤', str(exc), parent=self.root)
        except Exception as exc:
            self.failed(exc)

    def feedback(self, message):
        self.status.set(message.feedback.status)
        self.progress['value'] = message.feedback.progress * 100
        self.log(f'Feedback: {message.feedback.status}; progress={message.feedback.progress:.2f}')

    def accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.busy = False
                self.status.set('請求被拒絕')
                self.log('請求被拒絕，請檢查兩個參數陣列。')
                return
            handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.failed(exc)

    def result(self, future):
        try:
            result = future.result().result
            self.status.set('完成' if result.success else '失敗')
            self.log(f'Result:\nsuccess={result.success}\nerror_code={result.error_code}\nmessage={result.message}')
            self.busy = False
        except Exception as exc:
            self.failed(exc)

    def failed(self, exc):
        self.busy = False
        self.status.set('通訊錯誤')
        self.log(f'通訊錯誤：{exc}')

    def close(self):
        self.closing = True
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Node('task_plan_gui_client')
    try:
        root = tk.Tk()
        ClientWindow(root, node)
        root.mainloop()
    except tk.TclError as exc:
        node.get_logger().error(f'請在桌面終端開啟 GUI（需要 DISPLAY）：{exc}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
