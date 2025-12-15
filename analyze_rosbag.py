import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

plt.rcParams['font.family'] = 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False

# -----------------------------------------------------
# 안전한 CSV 로딩 함수
# -----------------------------------------------------
def safe_read_csv(filename):
    if os.path.exists(filename):
        print(f"📄 로딩 성공: {filename}")
        return pd.read_csv(filename)
    else:
        print(f"⚠️ 파일 없음, 스킵: {filename}")
        return None


# -----------------------------------------------------
# timestamp 컬럼 자동 찾기
# -----------------------------------------------------
def find_time_column(df):
    if df is None:
        return None

    for c in df.columns:
        if "time" in c.lower():    # %time, time, timestamp 등 자동 탐지
            return c

    print("⚠️ timestamp 컬럼 없음 → 시간 정규화 스킵")
    return None


# -----------------------------------------------------
# 시간 정규화 함수
# -----------------------------------------------------
def normalize_time(df):
    if df is None:
        return None

    t_col = find_time_column(df)
    if t_col is None:
        return None

    t = df[t_col] / 1e9
    return t - t.iloc[0]


# ----------------------------- 파일 읽기 ---------------------------
odom = safe_read_csv("odom.csv")
imu = safe_read_csv("imu.csv")
cmd = safe_read_csv("cmdvel.csv")
# agv = safe_read_csv("agv_pose_log.csv")
amcl = safe_read_csv("amcl.csv")


# ----------------------------- 시간 정규화 ---------------------------
odom_time = normalize_time(odom)
imu_time = normalize_time(imu)
cmd_time = normalize_time(cmd)
# agv_time = normalize_time(agv)
amcl_time = normalize_time(amcl)


# -----------------------------------------------------
# 특정 데이터 컬럼 자동 탐색
# -----------------------------------------------------
def find_column(df, contains, ends):
    if df is None:
        return None
    for col in df.columns:
        if contains in col and col.endswith(ends):
            return col
    return None


# ----------------------------- ODOM yaw -----------------------------
odom_yaw_col = find_column(odom, "orientation", ".z")
odom_yaw = odom[odom_yaw_col] if odom_yaw_col else None

# ----------------------------- IMU yaw-rate -----------------------------
imu_yaw_col = find_column(imu, "angular", ".z")
imu_yaw_rate = imu[imu_yaw_col] if imu_yaw_col else None

# ----------------------------- CMD vel -----------------------------
cmd_col = find_column(cmd, "angular", ".z")
cmd_vel_ang = cmd[cmd_col] if cmd_col else None

# ----------------------------- AGV pose yaw -----------------------------
# agv_yaw = None
# if agv is not None:
#     agv_yaw_col = find_column(agv, "orientation", ".z")
#     if agv_yaw_col:
#         agv_yaw = agv[agv_yaw_col]

# ----------------------------- AMCL yaw -----------------------------
amcl_yaw = None
if amcl is not None:
    amcl_yaw_col = find_column(amcl, "orientation", ".z")
    if amcl_yaw_col:
        amcl_yaw = amcl[amcl_yaw_col]


# -----------------------------------------------------
# 그래프 그리기
# -----------------------------------------------------
rows = 0
if odom_yaw is not None: rows += 1
if imu_yaw_rate is not None: rows += 1
# if agv_yaw is not None: rows += 1
if amcl_yaw is not None: rows += 1
if cmd_vel_ang is not None: rows += 1

plt.figure(figsize=(18, 4 * rows))
row = 1

def try_plot(title, t, y, color=None):
    global row
    if t is None or y is None:
        return
    plt.subplot(rows,1,row); row+=1
    plt.title(title)
    plt.plot(t, y, color=color)
    plt.grid()


try_plot("ODOM yaw", odom_time, odom_yaw)
try_plot("IMU yaw-rate", imu_time, imu_yaw_rate, "orange")
# try_plot("AGV pose yaw", agv_time, agv_yaw, "green")
try_plot("AMCL pose yaw", amcl_time, amcl_yaw, "purple")
try_plot("cmd_vel angular.z", cmd_time, cmd_vel_ang, "red")

plt.tight_layout()
plt.show()


# -----------------------------------------------------
# 급발진 탐지
# -----------------------------------------------------
if cmd_vel_ang is not None:
    threshold = 1.2
    spikes = cmd[cmd_vel_ang.abs() > threshold]
    print("\n===== 🚨 급발진 후보 지점 =====")
    print(spikes[[cmd.columns[0], cmd_col]])
else:
    print("cmd_vel 데이터 없음 → 급발진 분석 불가")
