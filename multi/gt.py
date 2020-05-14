import rospy
import time
from scipy import interpolate
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

coords = []
with open("./3d.txt", "r") as f:
    coords = f.read()
coords = coords.split("\n")
coords = filter(None, coords)

ts, ax, ay, az, bx, by, bz = [], [], [], [], [], [], []

for i in coords:
    i = i.split(" ")
    ts.append(float(i[0]))
    ax.append(float(i[1]))
    ay.append(float(i[2]))
    az.append(float(i[3]))
    bx.append(float(i[4]))
    by.append(float(i[5]))
    bz.append(float(i[6]))

def interp(t):

    tax = interpolate.splrep(ts, ax)
    nax = interpolate.splev(t, tax)
    tay = interpolate.splrep(ts, ay)
    nay = interpolate.splev(t, tay)
    taz = interpolate.splrep(ts, az)
    naz = interpolate.splev(t, taz)

    tbx = interpolate.splrep(ts, bx)
    nbx = interpolate.splev(t, tbx)
    tby = interpolate.splrep(ts, by)
    nby = interpolate.splev(t, tby)
    tbz = interpolate.splrep(ts, bz)
    nbz = interpolate.splev(t, tbz)

    return (nax, nay, naz, nbx, nby, nbz)

pub = rospy.Publisher('/person/ground_truth', PoseArray, queue_size=10)
rospy.init_node("ground_truth")
r = rospy.Rate(30)

while not rospy.is_shutdown():
    print(str(rospy.Time.now()))
    if str(rospy.Time.now()) == "0":
        print("sl")
        r.sleep()
    vals = interp(float(str(rospy.Time.now())))

    msg = PoseArray()
    msg.header.frame_id = "camera_depth_optical_frame"
    msg.header.stamp = rospy.Time.now()

    msg.poses = []

    p = Pose()
    p.position.x = vals[0]
    p.position.y = vals[1]
    p.position.z = vals[2]
    p.orientation.w = 1
    msg.poses.append(p)

    p = Pose()
    p.position.x = vals[3]
    p.position.y = vals[4]
    p.position.z = vals[5]
    p.orientation.w = 1
    msg.poses.append(p)

    pub.publish(msg)
    
    time.sleep(0.05)
