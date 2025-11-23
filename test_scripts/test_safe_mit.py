#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
from piper_sdk import *
from math import pi

def enable_piper(piper):
    max_retries=100
    for i in range(max_retries):
        if piper.EnablePiper():
            print(f"piper enabled after {i} retries.")
            return True
        print(f"piper trying to enable, attempt {i + 1}/{max_retries}")
        time.sleep(0.02)
    print(f"Failed to enable piper after {max_retries} retries.")
    return False

def config_joint_limit(piper):
    # Config angle limit and speed limit
    # angle limit unit:0.1degree
    # speed limit unit:0.001rad/s
    piper.MotorAngleLimitMaxSpdSet(6, 450, -450, int(15/180*pi*1000))
    print("Joint motor Angle limit and Speed limit configed.")

    # Config acceleration limit
    # acceleration limit unit:0.001rad/s^2
    piper.JointMaxAccConfig(6, 500)
    print("Joint Acceleration limit configed.")

    print(piper.GetAllMotorAngleLimitMaxSpd())

def main():
    piper = C_PiperInterface_V2("can0")
    print("FirmwareVersion:", piper.GetPiperFirmwareVersion())
    piper.ConnectPort()
    enabled_flag = enable_piper(piper)
    if not enabled_flag:
        return
    
    config_joint_limit(piper)

    while True:
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,-pi/3,0,3,0.8,0)
        print("sent moving to -1 command to joint 6")
        time.sleep(2)
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,0,0,3,0.8,0)
        print("sent moving to 0 command to joint 6")
        time.sleep(2)
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,pi/3,0,3,0.8,0)
        print("sent moving to 1 command to joint 6")
        time.sleep(2)
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        piper.JointMitCtrl(6,0,0,3,0.8,0)
        print("sent moving to 0 command to joint 6")
        time.sleep(2)


if __name__ == "__main__":
    main()