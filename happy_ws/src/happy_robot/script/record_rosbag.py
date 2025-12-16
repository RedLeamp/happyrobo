#!/usr/bin/env python3
import os
import datetime
import subprocess
import rospy
import rospkg

def main():
    rospy.init_node('rosbag_recorder', anonymous=True)

    # 현재 시간 기반 파일명 생성
    time_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # ROS 패키지 경로 내 record 디렉토리 지정
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('happy_robo')
    bag_dir = os.path.join(pkg_path, "record")
    os.makedirs(bag_dir, exist_ok=True)

    # 최종 저장 경로
    bag_path = os.path.join(bag_dir, f"record_{time_str}.bag")
    rospy.loginfo(f"Recording to {bag_path}")

    # rosbag record 실행
    command = ["rosbag", "record", "-O", bag_path, "-a", "/tf", "/tf_static"]
    subprocess.Popen(command)

    rospy.spin()  # 노드 유지

if __name__ == '__main__':
    main()
