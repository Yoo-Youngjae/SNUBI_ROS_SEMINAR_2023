import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import ros_numpy
import cv2
import numpy as np
class VisionController(object):
    def __init__(self):
        self.rgb_image = None
        self.depth_image = None
        self.pc = None
        self.bridge = CvBridge()

        # for azure kinect
        rgb_topic = '/rgb/image_raw'
        depth_topic = '/depth_to_rgb/image_raw'
        pointcloud_topic = '/points2'

        # for realsense-ros
        # rgb_topic = '/camera/color/image_raw'
        # depth_topic = '/camera/depth/image_rect_raw'
        # pointcloud_topic = '/camera/depth_registered/points'
        self._rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self._depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)
        self._pc_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self._pc_callback)


    def _rgb_callback(self,data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "passthrough")[:,:,:3]
        cv2.imshow('rgb image', self.rgb_image)
        cv2.waitKey(1)

    def _depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")


    def _pc_callback(self, point_msg):
        self.pc = ros_numpy.numpify(point_msg)


if __name__ == '__main__':
    rospy.init_node('ros_seminar_baseline', anonymous=True)
    vision_controller = VisionController()
    rate = rospy.Rate(5)
    image_resolution = (720, 1280, 3)
    while not rospy.is_shutdown():
        if vision_controller.pc is None:
            continue
        pc = np.array(vision_controller.pc.tolist()).reshape(image_resolution[0], image_resolution[1], -1)
        # print(pc)
        rate.sleep()


