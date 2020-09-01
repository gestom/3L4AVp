#!/usr/bin/env python
import rospy
import time
from scipy import interpolate
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

def callback(_):
    global val, pub
    if val == None:
        print("waiting")
    else:
        pub.publish(val)


pub = rospy.Publisher('/person/ground_truth', PoseArray, queue_size=10)
rospy.init_node("ground_truth_exp_3")
rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback)
r = rospy.Rate(30)

person_ = rospy.get_param("/ground_truth_exp_3/person")
path_ = rospy.get_param("/ground_truth_exp_3/path")

print(person_ , path_)

coords = []
with open(path_, "r") as f:
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

    #bspline
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

    #alternative
    kind = "linear"
    kind = "cubic"
    tax = interpolate.interp1d(ts, ax, kind=kind)
    nax = tax(t)
    tay = interpolate.interp1d(ts, ay, kind=kind)
    nay = tay(t)
    taz = interpolate.interp1d(ts, az, kind=kind)
    naz = taz(t)

    tbx = interpolate.interp1d(ts, bx, kind=kind)
    nbx = tbx(t)
    tby = interpolate.interp1d(ts, by, kind=kind)
    nby = tby(t)
    tbz = interpolate.interp1d(ts, bz, kind=kind)
    nbz = tbz(t)

    return [nax, nay, naz, nbx, nby, nbz]

val = None

while not rospy.is_shutdown():
    if str(rospy.Time.now()) == "0":
        print("sl")
        r.sleep()

    if float(str(rospy.Time.now())) < ts[0]:
        print("not yet started")
        continue

    if float(str(rospy.Time.now())) > ts[-1]:
        print("finished")
        break


    vals = interp(float(str(rospy.Time.now())))

    print(vals)
    vals[2] = vals[2] * 0.001
    vals[5] = vals[5] * 0.001

    vals[0] = vals[2] * ((vals[0] - 312.674) * (1/627.5634765625))
    vals[3] = vals[5] * ((vals[3] - 312.674) * (1/627.5634765625))

    #vals[0] = vals[2] * ((vals[0] - 307.674) * (1/640.56347))
    #vals[3] = vals[5] * ((vals[3] - 307.674) * (1/640.56347))


    vals[1] = vals[2] * ((vals[1] - 241.3366) * (1/627.56347))
    vals[4] = vals[5] * ((vals[4] - 241.3366) * (1/627.56347))

    msg = PoseArray()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()

    msg.poses = []

    p = Pose()
    p.position.x = vals[2]-0.5
    p.position.y = -vals[0]+0.227
    p.position.z = 0#vals[1]
    p.orientation.w = 1
    msg.poses.append(p)

    p = Pose()
    p.position.x = vals[5]-0.5
    p.position.y = -vals[3]+0.227
    p.position.z = 0#vals[4]
    p.orientation.w = 1
    msg.poses.append(p)

    #pub.publish(msg)
    val = msg

    time.sleep(0.01)
