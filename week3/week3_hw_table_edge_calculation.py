import rospy
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import cv2
import numpy as np
from cv_bridge import CvBridge
import ros_numpy

class HSR_sub():
    def __init__(self,
                 lidar_topic='/hsrb/base_scan',
                 rgb_topic='/hsrb/head_rgbd_sensor/rgb/image_raw',
                 depth_topic='/hsrb/head_rgbd_sensor/depth_registered/image_raw',
                 pc_topic='/hsrb/head_rgbd_sensor/depth_registered/rectified_points'):
        self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self._rgb_callback)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self._depth_callback)
        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)
        self._pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self._pc_callback)

        self.dist = None
        self.rgb_img = None
        self.depth_img = None
        self.pc = None
        self.bridge = CvBridge()



    def _rgb_callback(self, data):
        self.rgb_img = cv2.cvtColor(np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1), cv2.COLOR_RGB2BGR)
        cv2.putText(self.rgb_img, "Table edge length: ".format(self.dist), \
                    (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, \
                    (0, 255, 0), 3)
        cv2.imshow("rgb_img", self.rgb_img)
        cv2.waitKey(1)

    def _depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1") # [mm]

    def _pc_callback(self, data):
        self.pc = ros_numpy.numpify(data)


    def _lidar_callback(self, data):
        data_np = np.asarray(data.ranges)
        data_np[np.isnan(data_np)] = 0.0  # remove nans
        num_angles = data_np.shape # resolution of the lidar
        _width = 15 # ranges that you'll search
        center_idx = data_np.shape[0] // 2
        # print(num_angles)


if __name__ == '__main__':
    rospy.init_node('table_edge_length_detection')
    hsr_sub = HSR_sub()
    rate = rospy.Rate(5)
    image_resolution = (480, 640, 3)
    while not rospy.is_shutdown():
        if hsr_sub.pc is None:
            continue
        pc = np.array(hsr_sub.pc.tolist()).reshape(image_resolution[0], image_resolution[1], -1) #[m]
        rate.sleep()