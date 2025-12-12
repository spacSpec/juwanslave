import sys
import tty
import termios
import time
from pymycobot.mycobot320 import MyCobot320

# ================= 설정값 =================
PORT = '/dev/ttyACM0'
BAUD = 115200

# 좌표 제어 설정
STEP_MOVE = 5   # X, Y, Z 이동 거리 (mm) - 너무 크면 위험할 수 있음
STEP_ROT = 2    # Rx, Ry, Rz 회전 각도 (degree)
SPEED = 50      # 이동 속도 (0~100)
MODE = 1        # 0: angular(경로 무시, 빠름), 1: linear(직선 경로, 부드러움)

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    # 초기 자세 잡기 (안전한 위치에서 시작)
    mc.send_coords([-23.6, 175.5, 376.1, -161.52, 7.22, -15.57], 20,0)
    time.sleep(2) # 이동 완료 대기
except Exception as e:
    print(f"로봇 연결 실패: {e}")
    sys.exit()

# ================= 키 입력 유틸리티 =================
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ================= 메인 로직 =================
def main():
    mc.set_gripper_mode(0)
    mc.init_electric_gripper()
    time.sleep(1)
    
    saved_coords = []
    
    print("\n초기 좌표 읽는 중...")
    # 시작할 때 현재 좌표(Pose)를 읽어옴 [x, y, z, rx, ry, rz]
    current_coords = mc.get_coords()
    if not current_coords:
        print("❌ 좌표 읽기 실패. 연결을 확인하세요.")
        return
    
    # 리스트로 변환
    current_coords = list(current_coords)

    print("\n" + "="*50)
    print(" 📍 좌표(Coordinate) 제어 모드")
    print("="*50)
    print(" [X 축] Q / A  (앞/뒤 이동)")
    print(" [Y 축] W / S  (좌/우 이동)")
    print(" [Z 축] E / D  (위/아래 이동)")
    print("-" * 50)
    print(" [Rx  ] R / F  (X축 회전)")
    print(" [Ry  ] T / G  (Y축 회전)")
    print(" [Rz  ] Y / H  (Z축 회전)")
    print("-" * 50)
    print(" [SPACE] 현재 좌표 저장 (Sync)")
    print(" [ESC]   종료")
    print("="*50)
    
    # 초기 상태 출력
    print(f"시작 좌표: {current_coords}")

    while True:
        key = getch()
        move_needed = False
        
        # QWERTY 키 매핑 (좌표계 기준)
        # X축 (Q: 증가 / A: 감소)
        if key == 'q':   
            current_coords[0] += STEP_MOVE
            move_needed = True
        elif key == 'a': 
            current_coords[0] -= STEP_MOVE
            move_needed = True
            
        # Y축 (W: 증가 / S: 감소)
        elif key == 'w': 
            current_coords[1] += STEP_MOVE
            move_needed = True
        elif key == 's': 
            current_coords[1] -= STEP_MOVE
            move_needed = True
            
        # Z축 (E: 증가 / D: 감소)
        elif key == 'e': 
            current_coords[2] += STEP_MOVE
            move_needed = True
        elif key == 'd': 
            current_coords[2] -= STEP_MOVE
            move_needed = True
            
        # Rx (R: 증가 / F: 감소)
        elif key == 'r': 
            current_coords[3] += STEP_ROT
            move_needed = True
        elif key == 'f': 
            current_coords[3] -= STEP_ROT
            move_needed = True
            
        # Ry (T: 증가 / G: 감소)
        elif key == 't': 
            current_coords[4] += STEP_ROT
            move_needed = True
        elif key == 'g': 
            current_coords[4] -= STEP_ROT
            move_needed = True
            
        # Rz (Y: 증가 / H: 감소)
        elif key == 'y': 
            current_coords[5] += STEP_ROT
            move_needed = True
        elif key == 'h': 
            current_coords[5] -= STEP_ROT
            move_needed = True

        # 저장 및 동기화 (Space)
        elif key == ' ':
            real_coords = mc.get_coords()
            if real_coords:
                print(f"\n✅ 좌표 저장됨: {real_coords}")
                saved_coords.append(real_coords)
                current_coords = list(real_coords) # 실제 값으로 싱크 맞춤
            else:
                print("\n❌ 통신 오류")
        
        elif key == '\x1b': # ESC
            break
        
        # 이동 명령 전송
        if move_needed:
            # 좌표 제어에서는 하드웨어 한계(+-170도 등)가 아니라 작업 영역(Workspace)을 벗어나면
            # 로봇이 움직이지 않거나 경고를 뱉습니다. 별도의 limit 체크는 생략합니다.
            
            # send_coords(좌표리스트, 속도, 모드)
            # mode 1: Linear move (직선 이동)
            mc.send_coords(current_coords, SPEED, MODE)
            
            # 출력 포맷팅 (소수점 1자리)
            print(f"\r[XYZ] {current_coords[0]:.1f}, {current_coords[1]:.1f}, {current_coords[2]:.1f} | [RxRyRz] {current_coords[3]:.1f}, {current_coords[4]:.1f}, {current_coords[5]:.1f}   ", end="")

    print("\n" + "="*30)
    print("saved_coords = [")
    for p in saved_coords:
        formatted = [float(f"{x:.2f}") for x in p]
        print(f"    {formatted},") 
    print("]")

if __name__ == "__main__":
    main()