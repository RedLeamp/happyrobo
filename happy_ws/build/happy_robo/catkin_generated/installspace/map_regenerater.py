#! /usr/bin/env python3

from std_msgs.msg import Bool
import rospy

class MapController:

    def __init__(self):
        self.clear_map_pub = rospy.Publisher('/clear_map', Bool, queue_size=1)

    def regenerate_map(self):
        self.clear_map_pub.publish(Bool(data=False))
        rospy.loginfo("Regenerating map...")

if __name__ == '__main__':

    rospy.init_node("init_map_controller_node")
    rospy.loginfo("init_map_controller_node")

    try :
        oBBCalculator = MapController()
        while not rospy.is_shutdown():
            oBBCalculator.regenerate_map()
            rospy.sleep(1)
        # while not rospy.is_shutdown():
        #     rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Regenerate Map interrupted.")