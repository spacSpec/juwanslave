#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import mysql.connector

DB_HOST = "172.30.1.96"
DB_USER = "rosuser"
DB_PASSWORD = "1234"
DB_NAME = "dbdb"

def insert_ros_quality(m0_state: int, good_count: int, bad_count: int):
    """
    ros_quality_log에 M0 상태 + 양품/불량 누적 카운트 1줄 INSERT
    m0_state : 0 or 1
    good_count : 현재까지 양품 개수
    bad_count  : 현재까지 불량 개수  (DB에서는 defect_count 컬럼에 저장)
    """
    try:
        conn = mysql.connector.connect(
            host=DB_HOST,
            user=DB_USER,
            password=DB_PASSWORD,
            database=DB_NAME,
        )
        cursor = conn.cursor()

        # 🔴 여기만 컬럼 이름을 반드시 defect_count 로!
        sql = """
            INSERT INTO ros_quality_log (m0_state, good_count, defect_count)
            VALUES (%s, %s, %s)
        """
        data = (int(m0_state), int(good_count), int(bad_count))
        cursor.execute(sql, data)
        conn.commit()

        print(f"[db_ros] INSERT ros_quality_log OK (m0={m0_state}, good={good_count}, bad={bad_count})")

    except Exception as e:
        print("[db_ros] DB INSERT 실패:", e)

    finally:
        try:
            cursor.close()
            conn.close()
        except:
            pass