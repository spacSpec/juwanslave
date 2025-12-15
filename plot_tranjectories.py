#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

def load_log(path):
    t = []
    wheel_x, wheel_y = [], []
    ekf_x, ekf_y = [], []
    dr_x, dr_y = [], []

    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["time"]))

            def parse(val):
                return float(val) if val not in ("", None) else np.nan

            wheel_x.append(parse(row["wheel_x"]))
            wheel_y.append(parse(row["wheel_y"]))
            ekf_x.append(parse(row["ekf_x"]))
            ekf_y.append(parse(row["ekf_y"]))
            dr_x.append(parse(row["dr_x"]))
            dr_y.append(parse(row["dr_y"]))

    return (np.array(t),
            np.array(wheel_x), np.array(wheel_y),
            np.array(ekf_x),   np.array(ekf_y),
            np.array(dr_x),    np.array(dr_y))

def main():
    if len(sys.argv) < 2:
        print("사용법: python3 plot_trajectories.py pose_log_1.csv")
        return

    path = "/home/vboxuser/myagv_ros/agv_pose_log.csv"
    (t,
     wheel_x, wheel_y,
     ekf_x, ekf_y,
     dr_x, dr_y) = load_log(path)

    fig, ax = plt.subplots(figsize=(6, 6))

    # 전체 궤적
    ax.plot(wheel_x, wheel_y, 'b-', label="Wheel odometry")
    ax.plot(ekf_x,   ekf_y,   'r-', label="Robot pose EKF")

    # dead reckoning 데이터가 있으면 표시
    if not np.all(np.isnan(dr_x)):
        ax.plot(dr_x, dr_y, 'g-', label="Dead reckoning")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper right")
    ax.set_title("AGV trajectories")

    # ==== 차이가 가장 큰 지점 자동 찾기 (wheel vs EKF) ====
    valid = ~np.isnan(wheel_x) & ~np.isnan(ekf_x)
    if np.any(valid):
        dx = wheel_x[valid] - ekf_x[valid]
        dy = wheel_y[valid] - ekf_y[valid]
        dist = np.hypot(dx, dy)
        idx_local = np.argmax(dist)
        idx = np.where(valid)[0][idx_local]

        cx = (wheel_x[idx] + ekf_x[idx]) / 2.0
        cy = (wheel_y[idx] + ekf_y[idx]) / 2.0
        radius = 0.5  # 확대 영역 반경 (필요하면 조절)

        # 메인 플롯에 원 그리기
        circle = plt.Circle((cx, cy), radius, fill=False, color='k', lw=1.5)
        ax.add_patch(circle)

        # 인셋 축 생성
        ax_ins = inset_axes(ax, width="40%", height="40%", loc="center right")
        ax_ins.plot(wheel_x, wheel_y, 'b-')
        ax_ins.plot(ekf_x,   ekf_y,   'r-')
        if not np.all(np.isnan(dr_x)):
            ax_ins.plot(dr_x, dr_y, 'g-')

        # 확대 구간 범위 설정
        ax_ins.set_xlim(cx - radius, cx + radius)
        ax_ins.set_ylim(cy - radius, cy + radius)
        ax_ins.set_aspect("equal", adjustable="box")
        ax_ins.tick_params(labelsize=7)

        # 해당 지점 표시
        ax_ins.plot(wheel_x[idx], wheel_y[idx], 'bo')
        ax_ins.plot(ekf_x[idx],   ekf_y[idx],   'ro')
        if not np.all(np.isnan(dr_x)):
            ax_ins.plot(dr_x[idx], dr_y[idx], 'go')

        # 원과 인셋 연결 선
        mark_inset(ax, ax_ins, loc1=2, loc2=4, fc="none", ec="0.5")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
