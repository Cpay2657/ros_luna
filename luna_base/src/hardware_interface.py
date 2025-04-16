#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from luna_base.msg import encoders

ENCODER_TOPIC = '/baal/encoders'
CMD_VEL_TOPIC = '/baal/cmd_vel'

#Subscriber Callback: What we do when we get a message
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s, %s", data.linear.x, data.angular.z)

def luna_hardware_interface():

    ENCODER_LEFT = 0
    ENCODER_RIGHT = 0

    #Publisher
    rospy.init_node('luna_hardware_interface', anonymous=True)
    pub = rospy.Publisher(ENCODER_TOPIC, encoders, queue_size=10)
    #How often we publish
    rate = rospy.Rate(10) # 10hz
    
    #Subscriber
    rospy.Subscriber(CMD_VEL_TOPIC, Twist, callback)
	
	#Publisher Loop: What we do everytime the publisher publishes
    while not rospy.is_shutdown():
    	ENCODER_LEFT = ENCODER_LEFT + 1
    	ENCODER_RIGHT = ENCODER_RIGHT - 1
    	msg = encoders()
    	msg.encoder_left = ENCODER_LEFT
    	msg.encoder_right = ENCODER_RIGHT
    	#msg = "hello world %s" % rospy.get_time()
    	rospy.loginfo(msg)
    	pub.publish(msg)
    	rate.sleep()

if __name__ == '__main__':
    try:
        luna_hardware_interface()
    except rospy.ROSInterruptException:
        pass
