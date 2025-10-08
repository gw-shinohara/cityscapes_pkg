import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthEnhancerNode(Node):
    """
    左右のステレオ画像とカメラ情報から視差を計算し、深度画像を生成するノード。
    """
    def __init__(self):
        super().__init__('depth_enhancer')
        self.get_logger().info("Initializing Stereo Depth Enhancer Node...")

        # パラメータの宣言 (ベースライン)
        self.declare_parameter('baseline', 0.22)
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.get_logger().info(f"Using baseline: {self.baseline} meters")

        self.bridge = CvBridge()
        self.focal_length = None # CameraInfoから取得するまでNone

        # QoSプロファイル
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriberの作成
        left_image_sub = message_filters.Subscriber(self, Image, '/cityscape/front/left/image_raw', qos_profile=qos_profile)
        right_image_sub = message_filters.Subscriber(self, Image, '/cityscape/front/right/image_raw', qos_profile=qos_profile)
        camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/cityscape/front/camera_info', qos_profile=qos_profile)

        # 3つのトピックを同期
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [left_image_sub, right_image_sub, camera_info_sub],
            queue_size=10,
            slop=0.2
        )
        self.time_synchronizer.registerCallback(self.synced_callback)

        # Publisher
        self.depth_publisher = self.create_publisher(Image, '/cityscape/front/dense_depth', qos_profile)

        # StereoSGBM の設定
        block_size = 5
        min_disp = 0
        num_disp = 128 - min_disp
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        self.get_logger().info("Node initialized successfully. Waiting for synced messages...")

    def synced_callback(self, left_image_msg, right_image_msg, camera_info_msg):
        """
        左右の画像とカメラ情報が同期して受信されたときのコールバック
        """
        # カメラ情報から焦点距離(fx)を取得
        self.focal_length = camera_info_msg.k[0]
        if self.focal_length <= 0:
            self.get_logger().warn("Invalid focal length received. Skipping frame.")
            return

        try:
            left_bgr = self.bridge.imgmsg_to_cv2(left_image_msg, "bgr8")
            right_bgr = self.bridge.imgmsg_to_cv2(right_image_msg, "bgr8")
            left_gray = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # 視差マップを計算
        disparity_map = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
        valid_mask = disparity_map > 0
        depth_map = np.zeros_like(disparity_map, dtype=np.float32)

        # 深度を計算
        depth_map[valid_mask] = (self.focal_length * self.baseline) / disparity_map[valid_mask]

        # 深度画像をPublish
        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, "32FC1")
            depth_msg.header = left_image_msg.header
            self.depth_publisher.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthEnhancerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

