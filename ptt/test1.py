import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import time

rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('/deep_radar/out/clustering', PoseArray, queue_size=1)

msg = PoseArray()
msg.header.stamp = rospy.Time.now()

p = Pose()
p.position.x = 12
p.orientation.w =1 
msg.poses.append(p)
pub.publish(msg)

p = Pose()
p.position.x = 9
msg.poses.append(p)
msg.poses[0].position.x = 16
#pub.publish(msg)

time.sleep(4)
