#!/usr/bin/env python3
#coding=utf-8
import rclpy
from rclpy.node import Node
from nstc_msgs.msg import Dio
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO

PIN = 32

class USBDIONode(Node):
    def __init__(self):
        super().__init__('usb_dionode')
        
        # 1. GPIO 初始化移到這裡，只需執行一次
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # 發布到原本的 dio topic
        self.publisher = self.create_publisher(Dio, '/package/dioarray', 10)
        self.web_sub = self.create_subscription(Bool, '/web/is_start', self.web_callback, 10)

        # 每 0.05 秒檢查一次 (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("USB DIO Node started. Monitoring physical switch...")

        self.strategy_check = False
        self.prev_gpio_state = None  # 上一次的 GPIO 狀態，用來偵測變化
        self.current_strategy = False  # 當前實際輸出值

    def web_callback(self, msg):
        self.current_strategy = msg.data
        self.get_logger().info(f"Web override: strategy={self.current_strategy}")

    def timer_callback(self):
        try:
            gpio_state = GPIO.input(PIN)

            if self.prev_gpio_state is None:
                # 第一次執行：以 GPIO 初始化
                self.current_strategy = (gpio_state == GPIO.HIGH)
                self.prev_gpio_state = gpio_state
            elif gpio_state != self.prev_gpio_state:
                # 指撥「改變狀態」時才覆蓋 current_strategy
                self.current_strategy = (gpio_state == GPIO.HIGH)
                self.prev_gpio_state = gpio_state
                self.get_logger().info(f"Switch changed → strategy={self.current_strategy}")

            dio_msg = Dio()
            dio_msg.strategy = self.current_strategy

            if dio_msg.strategy != self.strategy_check:
                self.get_logger().error(f"{dio_msg.strategy}")
                self.strategy_check = dio_msg.strategy

            self.publisher.publish(dio_msg)

        except Exception as e:
            self.get_logger().error(f"GPIO Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = USBDIONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. 關閉節點時才清理 GPIO 資源
        node.get_logger().info("Shutting down... cleaning up GPIO.")
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()