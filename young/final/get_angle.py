import time
from pymycobot.mycobot320 import MyCobot320

def tuning_robot_posture(port, baudrate=115200):
    """
    [세계 최고 AGV 천재 심영주 전용 툴 v2.1]
    - 모드 선택 기능 (Angle vs Coord vs Manual)
    - [NEW] 수동(m) 모드: 손으로 움직이고 Enter 치면 좌표 따기 기능 추가
    """
    try:
        # 1. 로봇 연결
        mc = MyCobot320(port, baudrate)
        print(f"✅ [천재의 로봇] 연결 완료: {port}")
        time.sleep(0.5)
        
        # 2. 초기화 (필요시 주석 해제)
        mc.power_on() # 토크 켜기
        mc.send_angles([0, 0, 0, 0, 0, 0], 10)
        # mc.send_coords([250, 6.1, 300,-180, -10, 0],10,1)

        print("=" * 60)
        print("🤖 [심영주 에디션: 앵글/좌표/수동 올인원 튜닝기] 실행")
        print("👉 a: 앵글 입력 이동")
        print("👉 c: 좌표 입력 이동")
        print("👉 m: [NEW] 손으로 직접 움직여서 좌표 따기")
        print("=" * 60)

        step_count = 1

        while True:
            # 3. 모드 선택
            print(f"\n--- [Step {step_count}] ---")
            mode = input("🎮 모드 선택 (a: 앵글 / c: 좌표 / m: 수동측정 / q: 종료): ").strip().lower()

            if mode == 'q':
                print("👋 프로그램을 종료합니다.")
                break
            
            # === [NEW] 수동 측정 모드 (Manual) ===
            if mode == 'm':
                print("\n🔓 [수동 모드] 로봇의 힘을 풉니다. (Release Servos)")
                print("⚠️ 주의: 로봇이 쳐지지 않도록 손으로 받쳐주세요!")
                mc.release_all_servos()
                time.sleep(0.5)
                
                print("👉 원하는 자세를 잡은 후 'Enter'를 누르세요.")
                print("👉 메뉴로 돌아가려면 'q'를 입력하세요.")
                
                sub_step = 1
                while True:
                    user_cmd = input(f"   📸 [측정 {sub_step}] Enter: 캡처 / q: 나가기 >> ").strip().lower()
                    if user_cmd == 'q':
                        print("🔒 수동 모드를 종료합니다.")
                        # mc.power_on() # 필요하면 다시 힘을 걸어줍니다 (옵션)
                        break
                    
                    # 현재 위치 읽기
                    curr_angles = mc.get_angles()
                    curr_coords = mc.get_coords()
                    
                    if curr_angles and curr_coords:
                        print(f"   ✅ 캡처 완료!")
                        print(f"      📐 각도: {curr_angles}")
                        print(f"      📍 좌표: {curr_coords}")
                        print("      💾 [복사용 코드]")
                        print(f"      mc.send_angles({curr_angles}, 40)")
                        print(f"      mc.send_coords({curr_coords}, 40, 1)")
                        sub_step += 1
                    else:
                        print("   ⚠️ 데이터를 읽지 못했습니다.")
                continue

            # === 기존 입력 모드 (Angle / Coord) ===
            if mode not in ['a', 'c']:
                print("⚠️ 올바른 모드를 선택해주세요.")
                continue

            # 데이터 입력 받기
            type_str = "각도(Angles)" if mode == 'a' else "좌표(Coords)"
            user_input = input(f"🔢 이동할 {type_str} 6개 입력: ")

            if user_input.strip().lower() == 'q':
                break

            try:
                # 입력받은 문자열을 숫자 리스트로 변환
                target_values = [float(x) for x in user_input.replace(',', ' ').split()]

                if len(target_values) != 6:
                    print(f"⚠️ {type_str} 값은 정확히 6개여야 합니다.")
                    continue

                # 로봇 이동 명령
                print(f"🔄 로봇 이동 중... ({type_str})")
                
                if mode == 'a':
                    mc.send_angles(target_values, 40)
                else:
                    mc.send_coords(target_values, 40, 1)

                # 이동 대기
                time.sleep(3.0) 
                
                # 결과 확인
                curr_angles = mc.get_angles()
                curr_coords = mc.get_coords()

                if curr_angles and curr_coords:
                    print(f"✅ 이동 완료 (Step {step_count})")
                    print("-" * 40)
                    print(f"   📐 현재 각도: {curr_angles}")
                    print(f"   📍 현재 좌표: {curr_coords}")
                    print("-" * 40)
                    
                    print("💾 [코드 복사용 출력]")
                    print(f"   mc.send_angles({curr_angles}, 40)")
                    print(f"   mc.send_coords({curr_coords}, 40, 1)")
                    
                    step_count += 1
                else:
                    print("⚠️ 데이터를 읽지 못했습니다.")

            except ValueError:
                print("❌ 숫자 형식이 올바르지 않습니다.")
            except Exception as e:
                print(f"❌ 이동 중 오류 발생: {e}")

    except Exception as e:
        print(f"\n❌ 프로그램 치명적 오류: {e}")

if __name__ == "__main__":
    # 포트 설정 (Linux: /dev/ttyACM0, Windows: COMx)
    # 심영주님 환경에 맞춰 포트 변경됨
    PORT = '/dev/ttyACM2'  
    tuning_robot_posture(PORT)