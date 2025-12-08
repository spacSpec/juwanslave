import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import socket
import time

ESP32_IP = "172.30.1.45"  # << 반드시 너의 ESP32 IP로 수정
ESP32_PORT = 5000


class StmNode(Node):
    def __init__(self):
        super().__init__('stm_node')

        self.srv = self.create_service(
            SetBool,
            'plc/door_state',
            self.door_command_callback
        )

        self.get_logger().info("stm_node 서비스 실행 중...")

    def door_command_callback(self, request, response):
        if request.data is True:
            self.get_logger().info("문 열기 요청 수신 → ESP32 명령 전송")

            result = self.send_open_command()

            if result:
                response.success = True
                response.message = "open"
            else:
                response.success = False
                response.message = "fail"
        else:
            response.success = False
            response.message = "invalid request"

        return response

    def send_open_command(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((ESP32_IP, ESP32_PORT))

            sock.send(b'O')
            self.get_logger().info("ESP32로 'O' 전송 완료")

            time.sleep(0.2)
            recv = sock.recv(10).decode().strip()

            sock.close()

            if recv != "":
                return True
            else:
                return False

        except Exception as e:
            self.get_logger().error(f"ESP32 통신 오류: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = StmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
