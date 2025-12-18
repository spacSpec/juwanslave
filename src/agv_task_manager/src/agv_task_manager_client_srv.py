#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def handle_setbool(req):
    rospy.loginfo(f"[ROS1] /demo_flag 요청 받음: data={req.data}")

    # 여기에서 실제 로직 수행 (예: PLC 제어, AGV 제어 등)
    success = True
    if req.data:
        message = "[True] Okay"
    else:
        message = "[False] Faild"

    return SetBoolResponse(success=success, message=message)

if __name__ == "__main__":
    rospy.init_node("ros1_setbool_server")

    srv = rospy.Service("/agv/request_dispatch", SetBool, handle_setbool)
    rospy.loginfo("[ROS1] /demo_flag SetBool 서비스 서버 준비 완료")

    rospy.spin()
