# -*- coding: utf-8 -*-

# for the real world
import pickle
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sensor_msgs.msg
import rospy
from collections import deque

bridge = CvBridge()
data = deque()
def write(data1, data2,data3):
    try:
        image = bridge.compressed_imgmsg_to_cv2(data1)
    except CvBridgeError as e:
        print e
#   timestr = "%.6f" %  data1.header.stamp.to_sec()
    depth = data2.range
#   timestr = "%.6f" %  data2.header.stamp.to_sec()
    gps=(data3.longitude,data3.latitude)
#    timestr = "%.6f" %  data2.header.stamp.to_sec()
    data.append((image, depth, gps))
    print depth
    print gps
    print image
def shut():
    pickle.dump(data, open("data.p", "wb"))
    rospy.loginfo("Pickle the data")
    rospy.sleep(0.5)
def main():
    rospy.init_node("syn")
    image = Subscriber("/usb_cam/image_raw/compressed", sensor_msgs.msg.CompressedImage)
    depth = Subscriber("/sonar", sensor_msgs.msg.Range)
    gps=Subscriber("/mavros/global_position/global",sensor_msgs.msg.NavSatFix)
    ats = ApproximateTimeSynchronizer([image, depth,gps], queue_size=5, slop=0.1)
    ats.registerCallback(write)
    rospy.spin()
    rospy.on_shutdown(shut)
if __name__ == '__main__':
    main()
