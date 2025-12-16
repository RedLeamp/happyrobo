#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def main():
    rospy.init_node('clear_map_publisher')

    # 퍼블리셔 생성
    pub = rospy.Publisher('/clear_map', Bool, queue_size=10)

    rospy.loginfo("Publishing to /clear_map topic. Press 't' to send True, 'f' to send False, or 'q' to quit.")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # 사용자 입력 대기
            user_input = input("Enter command (t: True, f: False, q: Quit): ").strip().lower()

            if user_input == 't':
                msg = Bool(data=True)
                pub.publish(msg)
                rospy.loginfo("Published: True")
            elif user_input == 'f':
                msg = Bool(data=False)
                pub.publish(msg)
                rospy.loginfo("Published: False")
            elif user_input == 'q':
                rospy.loginfo("Exiting...")
                break
            else:
                rospy.logwarn("Invalid input. Please enter 't', 'f', or 'q'.")
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
