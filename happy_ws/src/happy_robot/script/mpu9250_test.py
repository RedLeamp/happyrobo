#! /usr/bin/env python3

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

# MPU9250 객체 생성
mpu = MPU9250(
    address_ak=AK8963_ADDRESS,  # 자기 센서 주소
    address_mpu_master=MPU9050_ADDRESS_68,  # MPU 주소
    address_mpu_slave=None,
    bus=7,  # I2C 버스 (Jetson에서는 보통 1번)
    gfs=GFS_1000,  # 자이로 풀 스케일
    afs=AFS_8G,   # 가속도 풀 스케일
    mfs=AK8963_BIT_16,  # 자기 센서 해상도
    mode=AK8963_MODE_C100HZ)

# 초기화
# mpu.calibrate()  # 센서 보정
mpu.configure()  # 센서 설정

# 데이터 읽기
while True:
    print("Accelerometer: ", mpu.readAccelerometerMaster())
    print("Gyroscope: ", mpu.readGyroscopeMaster())
    print("Magnetometer: ", mpu.readMagnetometerMaster())
    print("-----------------------------")