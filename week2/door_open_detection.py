import rospy
from sensor_msgs.msg import LaserScan, Image
import cv2
import numpy as np

class ROS_sub():
    def __init__(self, lidar_topic, rgb_topic, threshold_dist):
        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_cb)
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_cb)
        self.threshold_dist = threshold_dist
        self.dist = None
        self.img = None

    def _depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")


    def _lidar_cb(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 0.0  # remove nans
        num_angles = data_np.shape # resolution of the lidar
        # degree 1 == 4
        # degree 240 == 960
        print(num_angles)
        _width = 16 # ranges that you'll search
        # degree 4 == 16

        # center_idx = data_np.shape[0] // 2
        # data region of interest that we want to compute
        # roi_data = data_np[center_idx - _width: center_idx + _width]
        # self.dist = np.mean(roi_data)

        # self.dist = data_np[center_idx]
        # print("dist", self.dist)

    def _rgb_cb(self, data):
        # rospy.loginfo('image_sub node started')
        self.img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        if self.dist is not None:
            cv2.putText(self.img, "Dist to the door : {:.3f}".format(self.dist),\
                        (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,\
                        (255, 0, 0), 3)
            if self.dist <= 1.5: # if dist is less than 1.5 the door is closed
                cv2.putText(self.img, "Door Closed.".format(self.dist),\
                        (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,\
                        (0, 255, 0), 3)
            else: # else we considered the door is opened
                cv2.putText(self.img, "Door Open.".format(self.dist), \
                            (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                            (0, 0, 255), 3)
        cv2.imshow("img", self.img)
        cv2.waitKey(1)

if __name__ == '__main__':

    while not rospy.is_shutdown():
        rospy.init_node('door_detection')
        ls = ROS_sub('/hsrb/base_scan', '/hsrb/head_rgbd_sensor/rgb/image_raw', .5)
        rospy.spin()