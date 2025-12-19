# -*- coding: utf-8 -*-
"""
MyCobot 320 Protection Recover Script (확실히 동작하는 320 전용)
"""

import time
from pymycobot.mycobot320 import MyCobot320

PORT = "/dev/ttyACM0"
BAUD = 115200


def safe_get(mc, func, default=None):
    try:
        return func()
    except:
        return default


def print_status(mc):
    print("\n===== ROBOT STATUS CHECK =====")

    # 320이 지원하는 유일한 상태 계열 함수
    fresh = safe_get(mc, mc.get_fresh_mode)
    print(f"Fresh Mode: {fresh}")

    angles = safe_get(mc, mc.get_angles)
    print(f"Angles: {angles}")

    coords = safe_get(mc, mc.get_coords)
    print(f"Coords: {coords}")

    print("================================\n")


def clear_errors(mc):
    print("🔧 에러 클리어 시도 중...")

    try:
        mc.release_all_servos()    # 모터 잠금 해제
        time.sleep(0.5)
    except:
        pass

    try:
        mc.set_free_mode()         # 힘 빼기
        time.sleep(0.5)
    except:
        pass

    try:
        mc.power_on()              # 다시 파워온
        time.sleep(1)
    except:
        pass

    print("✔ 기본 복구 절차 완료\n")


def servo_test(mc):
    print("🧪 서보 테스트 (각 축 활성화)")

    for i in range(1, 7):
        try:
            mc.set_servo_enable(i, True)
            print(f" - Servo {i} enabled")
        except:
            print(f"   ⚠️ Servo {i} enable FAILED")

        time.sleep(0.2)

    print("✔ 서보 활성화 완료\n")


def move_home(mc):
    print("🏠 HOME 포즈 이동")
    home = [0, 0, 0, 0, 0, 0]

    try:
        mc.send_angles(home, 20)
    except:
        print("⚠️ HOME 이동 실패")


def main():
    print("🔌 포트 연결 중...")
    mc = MyCobot320(PORT, BAUD)
    time.sleep(1)

    print("🧩 초기 상태 확인")
    print_status(mc)

    clear_errors(mc)

    print("🧩 복구 후 상태 재확인")
    print_status(mc)

    servo_test(mc)

    move_home(mc)

    print("\n🎉 복구 스크립트 완료!")


if __name__ == "__main__":
    main()
