from pymycobot.mycobot320 import MyCobot320
import time
mc = MyCobot320("/dev/ttyACM1", 115200)
#그리퍼 초기화
mc.set_gripper_mode(0)
mc.init_electric_gripper()
time.sleep(2)
mc.set_electric_gripper(0)
mc.set_gripper_value(100,20,1)
print("Gripper Mode", mc.get_gripper_mode())
time.sleep(0.5)
mc.set_gripper_value(40,20,1)
# time.sleep(0.5)
# mc.set_gripper_value(100,20,1)
# time.sleep(0.5)
# mc.set_gripper_value(1,20,1)
# time.sleep(0.5)
# mc.set_gripper_value(50,20,1)










