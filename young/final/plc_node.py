#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class PLCNode(Node):
    def __init__(self):
        super().__init__('plc_node')
        self.cli = self.create_client(Trigger, 'judge_item')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ 컨트롤러 연결 대기 중...')
        self.req = Trigger.Request()

    def send_signal(self):
        self.get_logger().info('📡 [송신] 물건 도착 신호 전송!')
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = PLCNode()
    print("🏭 [가상 PLC] 엔터(Enter)를 치면 물건 도착 신호를 보냅니다.")

    try:
        while rclpy.ok():
            input("\n👉 Enter 키를 누르세요 >> ")
            res = node.send_signal()
            
            if res.message == "PASS_RUN":
                print("   ⚙️ [동작] 컨베이어 계속 가동 (불량)")
            elif res.message == "STOP_PICK":
                print("   🛑 [동작] 컨베이어 정지 & 로봇 작업 (양품)")
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
