import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import time

rospy.init_node('talkera', anonymous=True)
pub = rospy.Publisher('/deep_radar/out/clustering', PoseArray, queue_size=1)

msg = PoseArray()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id="base_radar_link"

p = Pose()
p.position.x = 12
p.orientation.w =1 
msg.poses.append(p)

time.sleep(1)

pub.publish(msg)

time.sleep(3)

p = Pose()
p.position.x = 14
p.orientation.w =1 
msg.poses.append(p)

msg.poses[0].position.x = 9

pub.publish(msg)
