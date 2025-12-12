import sys
import tty
import termios
import time
from pymycobot.mycobot320 import MyCobot320

# ================= 설정값 =================
PORT = '/dev/ttyACM0'
BAUD = 115200
STEP_ANGLE = 10  # 한 번 누를 때 움직일 각도 (10도는 너무 확 튈 수 있어 5도로 설정, 필요시 수정)
SPEED = 50      # 이동 속도 (0~100)

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    mc.send_angles([-91.4, 2.81, 6.85, 75.05, -89.03, 0.43],10)
except Exception as e:
    print(f"로봇 연결 실패: {e}")
    sys.exit()

time.sleep(1)

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
    
    saved_angles = []
    
    print("\n초기 자세 읽는 중...")
    # 시작할 때 현재 각도를 읽어옴
    current_angles = mc.get_angles()
    if not current_angles:
        print("❌ 각도 읽기 실패. 연결을 확인하세요.")
        return
    
    # 리스트로 변환 (수정 가능하도록)
    current_angles = list(current_angles)

    print("\n" + "="*50)
    print(" 🦾 관절(Joint) 각도 제어 모드")
    print("="*50)
    print(" [J1] Q / A  (허리 회전)")
    print(" [J2] W / S  (메인 관절)")
    print(" [J3] E / D  (상부 관절)")
    print(" [J4] R / F  (팔뚝 회전)")
    print(" [J5] T / G  (손목 꺾기)")
    print(" [J6] Y / H  (손목 회전)")
    print("-" * 50)
    print(" [SPACE] 현재 각도 저장 (Sync)")
    print(" [ESC]   종료")
    print("="*50)

    while True:
        key = getch()
        move_needed = False
        
        # J1 ~ J6 키 매핑 (QWERTY / ASDFGH)
        if key == 'q':   
            current_angles[0] += STEP_ANGLE
            move_needed = True
        elif key == 'a': 
            current_angles[0] -= STEP_ANGLE
            move_needed = True
            
        elif key == 'w': 
            current_angles[1] += STEP_ANGLE
            move_needed = True
        elif key == 's': 
            current_angles[1] -= STEP_ANGLE
            move_needed = True
            
        elif key == 'e': 
            current_angles[2] += STEP_ANGLE
            move_needed = True
        elif key == 'd': 
            current_angles[2] -= STEP_ANGLE
            move_needed = True
            
        elif key == 'r': 
            current_angles[3] += STEP_ANGLE
            move_needed = True
        elif key == 'f': 
            current_angles[3] -= STEP_ANGLE
            move_needed = True
            
        elif key == 't': 
            current_angles[4] += STEP_ANGLE
            move_needed = True
        elif key == 'g': 
            current_angles[4] -= STEP_ANGLE
            move_needed = True
            
        elif key == 'y': 
            current_angles[5] += STEP_ANGLE
            move_needed = True
        elif key == 'h': 
            current_angles[5] -= STEP_ANGLE
            move_needed = True

        # 저장 및 동기화 (Space)
        elif key == ' ':
            real_angles = mc.get_angles()
            if real_angles:
                print(f"\n✅ 각도 저장됨: {real_angles}")
                saved_angles.append(real_angles)
                current_angles = list(real_angles) # 실제 값으로 싱크 맞춤
            else:
                print("\n❌ 통신 오류")
        
        elif key == '\x1b': # ESC
            break
        
        # 이동 명령 전송
        if move_needed:
            # 안전을 위한 각도 제한 (대략적인 하드웨어 한계)
            for i in range(6):
                if current_angles[i] > 170: current_angles[i] = 170
                if current_angles[i] < -170: current_angles[i] = -170
            
            # send_angles(각도리스트, 속도)
            mc.send_angles(current_angles, SPEED)
            
            # 출력 포맷팅
            print(f"\r[J1~J6] {current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}, {current_angles[3]:.0f}, {current_angles[4]:.0f}, {current_angles[5]:.0f}   ", end="")

    print("\n" + "="*30)
    print("saved_angles = [")
    for p in saved_angles:
        # 소수점 2자리까지만 깔끔하게 출력
        formatted = [float(f"{x:.2f}") for x in p]
        print(f"    {formatted},") 
    print("]")

if __name__ == "__main__":
    main()