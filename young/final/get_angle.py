import sys
import tty
import termios
import time
from pymycobot.mycobot320 import MyCobot320

# ================= 설정값 =================
PORT = '/dev/ttyACM0'
BAUD = 115200

# 키보드 조종 설정
KEYBOARD_STEP_ANGLE = 5   # 키보드 한 번 누를 때 각도
KEYBOARD_SPEED = 80       # 키보드 이동 속도

# ================= 유틸리티 함수 =================
def getch():
    """리눅스 터미널용 키 입력 함수"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# ================= 기능 1: 키보드 조종 모드 =================
# ================= 기능 1: 키보드 조종 모드 (좌표 표시 추가됨) =================
def mode_keyboard_control(mc):
    print("\n" + "="*50)
    print(" 🦾 [모드 1] 실시간 키보드 제어 (Joint & Coord)")
    print("="*50)
    print(" [J1] Q / A  (허리 회전)")
    print(" [J2] W / S  (메인 관절)")
    print(" [J3] E / D  (상부 관절)")
    print(" [J4] R / F  (팔뚝 회전)")
    print(" [J5] T / G  (손목 꺾기)")
    print(" [J6] Y / H  (손목 회전)")
    print("-" * 50)
    print(" [SPACE] 현재 각도/좌표 저장 (Sync)")
    print(" [ESC]   메인 메뉴로 복귀")
    print("="*50)

    # 1. 진입 시 서보에 힘을 줍니다 (안전)
    mc.power_on()
    time.sleep(0.5)

    # 2. 현재 각도 및 좌표 읽기
    current_angles = mc.get_angles()
    # 초기 좌표값 읽기
    current_coords = mc.get_coords() 
    
    if not current_angles:
        print("❌ 각도 읽기 실패. 연결 상태를 확인하세요.")
        return

    current_angles = list(current_angles)
    saved_angles = []

    # 좌표 출력을 위한 초기 문자열 설정
    coord_str = "Reading..."
    if current_coords:
        coord_str = f"X:{current_coords[0]:.1f} Y:{current_coords[1]:.1f} Z:{current_coords[2]:.1f}"

    # 초기 상태 출력
    print(f"\r[J1~J6] {current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}, {current_angles[3]:.0f}, {current_angles[4]:.0f}, {current_angles[5]:.0f} | 📍 {coord_str}   ", end="")

    while True:
        key = getch()
        move_needed = False
        
        # 키 매핑 (QWERTY / ASDFGH)
        if key == 'q':   current_angles[0] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'a': current_angles[0] -= KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'w': current_angles[1] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 's': current_angles[1] -= KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'e': current_angles[2] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'd': current_angles[2] -= KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'r': current_angles[3] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'f': current_angles[3] -= KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 't': current_angles[4] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'g': current_angles[4] -= KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'y': current_angles[5] += KEYBOARD_STEP_ANGLE; move_needed = True
        elif key == 'h': current_angles[5] -= KEYBOARD_STEP_ANGLE; move_needed = True

        # 저장 및 동기화 (Space)
        elif key == ' ':
            real_angles = mc.get_angles()
            real_coords = mc.get_coords() # 좌표도 같이 읽기
            
            if real_angles and real_coords:
                print(f"\n✅ 저장됨 -> 각도: {real_angles}")
                print(f"            좌표: {real_coords}")
                saved_angles.append(real_angles)
                current_angles = list(real_angles)
                # 좌표 문자열 업데이트
                coord_str = f"X:{real_coords[0]:.1f} Y:{real_coords[1]:.1f} Z:{real_coords[2]:.1f}"
            else:
                print("\n❌ 통신 오류 (데이터 읽기 실패)")
            
            # 줄바꿈이 일어났으므로 다시 입력 대기 상태 표시
            print(f"\r[J1~J6] {current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}, {current_angles[3]:.0f}, {current_angles[4]:.0f}, {current_angles[5]:.0f} | 📍 {coord_str}   ", end="")

        # 나가기 (ESC)
        elif key == '\x1b': 
            print("\n🔙 메인 메뉴로 돌아갑니다.")
            break
        
        # 이동 명령 전송
        if move_needed:
            # 안전 제한
            for i in range(6):
                if current_angles[i] > 170: current_angles[i] = 170
                if current_angles[i] < -170: current_angles[i] = -170
            
            mc.send_angles(current_angles, KEYBOARD_SPEED)

            # [핵심] 이동 후 현재 좌표를 요청해서 읽어옴
            # 주의: 통신 속도 때문에 키 반응이 약간 느려질 수 있음
            temp_coords = mc.get_coords()
            if temp_coords:
                coord_str = f"X:{temp_coords[0]:.1f} Y:{temp_coords[1]:.1f} Z:{temp_coords[2]:.1f}"
            
            # 각도와 좌표를 한 줄에 출력
            print(f"\r[J1~J6] {current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}, {current_angles[3]:.0f}, {current_angles[4]:.0f}, {current_angles[5]:.0f} | 📍 {coord_str}   ", end="")

    # 종료 시 저장된 데이터 출력
    if saved_angles:
        print("\n" + "="*30)
        print("💾 [저장된 키보드 포인트]")
        print("saved_list = [")
        for p in saved_angles:
            formatted = [float(f"{x:.2f}") for x in p]
            print(f"    {formatted},") 
        print("]")

# ================= 기능 2: 튜닝/수동 모드 =================
def mode_tuning_tool(mc):
    print("\n" + "="*60)
    print("🛠️ [모드 2] 심영주 에디션: 정밀 튜닝 & 수동 측정")
    print("=" * 60)

    step_count = 1

    while True:
        print(f"\n--- [Step {step_count}] ---")
        mode = input("🎮 기능 선택 (a: 앵글입력 / c: 좌표입력 / m: 수동(손으로) / q: 뒤로가기): ").strip().lower()

        if mode == 'q':
            # 나갈 때 혹시 모르니 서보를 잠궈줍니다.
            mc.power_on()
            print("🔙 메인 메뉴로 돌아갑니다.")
            break
        
        # === [m] 수동 측정 모드 ===
        if mode == 'm':
            print("\n🔓 [수동 모드] 로봇 힘을 풉니다 (Release). 손으로 받쳐주세요!")
            mc.release_all_servos()
            time.sleep(0.5)
            
            print("👉 원하는 자세를 잡은 후 'Enter'를 누르세요.")
            print("👉 나가려면 'q' 입력 후 Enter")
            
            sub_step = 1
            while True:
                user_cmd = input(f"   📸 [측정 {sub_step}] Enter: 캡처 / q: 나가기 >> ").strip().lower()
                if user_cmd == 'q':
                    print("🔒 수동 모드 종료. 로봇에 힘을 줍니다.")
                    mc.power_on() # 다시 힘을 줘서 자세 유지
                    break
                
                curr_angles = mc.get_angles()
                curr_coords = mc.get_coords()
                
                if curr_angles and curr_coords:
                    print(f"   ✅ 캡처 완료!")
                    print(f"      📐 각도: {curr_angles}")
                    print(f"      📍 좌표: {curr_coords}")
                    sub_step += 1
                else:
                    print("   ⚠️ 데이터 읽기 실패.")
            continue

        # === [a/c] 값 입력 이동 ===
        if mode not in ['a', 'c']:
            print("⚠️ 올바른 키를 눌러주세요.")
            continue

        type_str = "각도(Angles)" if mode == 'a' else "좌표(Coords)"
        user_input = input(f"🔢 이동할 {type_str} 6개 입력 (공백 구분): ")

        if user_input.strip().lower() == 'q':
            break

        try:
            target_values = [float(x) for x in user_input.replace(',', ' ').split()]
            if len(target_values) != 6:
                print(f"⚠️ 값은 정확히 6개여야 합니다.")
                continue

            print(f"🔄 이동 중... ({type_str})")
            mc.power_on() # 이동 전 확실하게 힘 주기

            if mode == 'a':
                mc.send_angles(target_values, 10)
            else:
                mc.send_coords(target_values, 10, 1)

            time.sleep(2.5) 
            
            # 결과 확인
            curr_angles = mc.get_angles()
            curr_coords = mc.get_coords()
            if curr_angles:
                print(f"✅ 이동 완료! 현재 각도: {curr_angles}")
                step_count += 1
            if curr_coords:
                print(f"✅ 이동 완료! 현재 whkvy: {curr_coords}")
                step_count += 1

        except ValueError:
            print("❌ 숫자 형식이 아닙니다.")
        except Exception as e:
            print(f"❌ 오류: {e}")

# ================= 메인 실행 =================
def main():
    try:
        print(f"🔌 로봇 연결 시도 중... ({PORT})")
        mc = MyCobot320(PORT, BAUD)
        mc.power_on()
        time.sleep(0.5)
        print("✅ 연결 성공!")
        
        # 초기 자세 이동 (선택사항)
        # mc.send_angles([-90, 0, 0, 0, 0, 0], 30)

    except Exception as e:
        print(f"❌ 연결 실패: {e}")
        sys.exit()

    while True:
        print("\n" + "■"*40)
        print(" 🤖 통합 로봇 제어 시스템 v3.0")
        print("■"*40)
        print(" 1. 키보드 조종 (Joint Control)")
        print(" 2. 튜닝 툴 (수동/입력)")
        print(" Q. 프로그램 종료")
        print("-" * 40)
        
        choice = input("👉 번호를 선택하세요: ").strip().lower()

        if choice == '1':
            mode_keyboard_control(mc)
        elif choice == '2':
            mode_tuning_tool(mc)
        elif choice == 'q':
            print("👋 프로그램을 종료합니다.")
            sys.exit()
        else:
            print("⚠️ 1, 2, Q 중에서 선택해주세요.")

if __name__ == "__main__":
    main()