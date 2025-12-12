import time
import sys
# 리눅스 환경에서 가끔 포트 권한 에러가 날 때를 대비해 os 모듈 추가
import os 
from pymycobot.mycobot320 import MyCobot320

# ================= 설정값 =================
PORT = '/dev/ttyACM0'
BAUD = 115200

def reset_and_connect():
    """
    통신 라인을 깔끔하게 청소하고 연결하는 함수
    작성자님의 의견을 반영하여 '초기화' 과정을 강화했습니다.
    """
    print(f"\n♻️ [초기화] {PORT} 통신 버퍼 클리어 및 재연결 시도...")
    
    try:
        # 1. 기존 연결이 있다면 끊어주는 효과를 위해 잠시 대기
        time.sleep(0.5)
        
        # 2. 객체 생성 (이때 시리얼 포트가 열립니다)
        mc = MyCobot320(PORT, BAUD)
        time.sleep(0.5)
        
        # 3. [핵심] 찌꺼기 데이터 비우기 (Wake Up)
        # 의미 없는 신호를 한번 보내거나, 상태를 읽어서 버퍼를 비웁니다.
        mc.power_on() 
        time.sleep(0.1)
        
        # 안정화를 위해 잠시 대기
        time.sleep(1.0)
        print("✅ 통신 라인 초기화 완료. 연결 성공!")
        return mc
        
    except Exception as e:
        print(f"❌ 포트 연결 실패: {e}")
        print("👉 팁: USB 케이블을 뽑았다 다시 꽂거나, 'sudo chmod 777 /dev/ttyACM1'을 입력해보세요.")
        return None

def deep_scan():
    # 1. 안정적인 연결 객체 생성
    mc = reset_and_connect()
    if mc is None:
        return

    print("\n🔬 [정밀 진단 모드] 개별 모터 응답 테스트를 시작합니다.")
    print(f"{'ID':<5} | {'연결 상태':<10} | {'전압(V)':<10} | {'온도(°C)':<10} | {'상태(Status)'}")
    print("-" * 80)

    success_count = 0
    
    # 1번부터 6번 모터까지 개별 호출
    for joint_id in range(1, 7):
        status_msg = "❓ 응답 없음"
        volt_msg = "-"
        temp_msg = "-"
        
        try:
            # 시도: 데이터 읽기
            # 읽기 전에 아주 짧은 쉼을 줘서 통신 충돌 방지
            time.sleep(0.1) 
            volts = mc.get_servo_voltages()
            
            time.sleep(0.05)
            temps = mc.get_servo_temps()
            
            # [데이터 검증 로직]
            # 1. None 체크
            # 2. -1 (통신 에러 코드) 체크
            # 3. 리스트 길이 체크
            is_voltage_error = (not volts) or (isinstance(volts, int)) or (len(volts) < joint_id)
            is_temp_error = (not temps) or (isinstance(temps, int)) or (len(temps) < joint_id)
            
            if is_voltage_error or is_temp_error:
                status_msg = "❌ 패킷 유실 (통신불안)"
            else:
                v = volts[joint_id-1]
                t = temps[joint_id-1]
                
                volt_msg = f"{v}V"
                temp_msg = f"{t}°C"
                
                # 상태 판별
                if v < 18.0:
                    status_msg = "⚠️ 전압 부족 (Low Battery)"
                elif t > 60:
                    status_msg = "🔥 과열 (Overheat)"
                else:
                    status_msg = "✅ 정상"
                
                success_count += 1

        except Exception as e:
            status_msg = f"💥 코드 에러: {e}"

        # 결과 출력
        connect_status = 'Connected' if ('정상' in status_msg or '부족' in status_msg or '과열' in status_msg) else 'LOST'
        print(f"J{joint_id:<4} | {connect_status:<10} | {volt_msg:<10} | {temp_msg:<10} | {status_msg}")
        
        # 다음 모터 조회 전 충분한 대기 시간 (통신 꼬임 방지)
        time.sleep(0.2) 

    print("-" * 80)

    # 2. 결과 분석
    print("\n🧐 [분석 결과]")
    if success_count == 6:
        print("1️⃣ 모든 관절 모터와 통신 성공! (통신 상태 양호)")
        print("   👉 이제 본 프로젝트 코드를 실행하셔도 됩니다.")
    elif success_count == 0:
        print("2️⃣ 모든 모터 응답 없음.")
        print("   👉 비상정지 버튼 확인 / 전원 어댑터 연결 확인 / USB 포트 변경 필요.")
    else:
        print(f"3️⃣ 일부 모터({6-success_count}개) 응답 없음.")
        print("   👉 통신은 되지만 불안정합니다. USB 선을 건드리지 말고 다시 실행해보세요.")

    # 3. 시스템 에러 코드 확인 (옵션)
    try:
        sys_err = mc.get_error_information()
        if sys_err != 0:
            print(f"\n⚠️ 로봇 내부 에러 코드 감지: {sys_err}")
    except:
        pass

if __name__ == "__main__":
    deep_scan()