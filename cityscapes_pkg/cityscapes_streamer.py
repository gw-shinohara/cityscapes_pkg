import rclpy
from rclpy.node import Node
from rclpy.time import Time
import os
import cv2
from cv_bridge import CvBridge
import glob
import json
import ast
import numpy as np

from sensor_msgs.msg import Image, NavSatFix, Temperature, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

class CityScapeRos2Streamer(Node):
    """
    CityScapeデータセットをROS 2トピックとしてストリーミングするノード
    """
    def __init__(self):
        super().__init__('cityscape_streamer')

        # パラメータ宣言
        self.declare_parameter('dataroot', '/path/to/your/cityscape/data')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('scene_names', "['aachen', 'bochum']")
        self.declare_parameter('play_all_scenes', False)

        # パラメータ取得
        self.dataroot = self.get_parameter('dataroot').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        try:
            self.scene_names = ast.literal_eval(self.get_parameter('scene_names').get_parameter_value().string_value)
        except (ValueError, SyntaxError):
            self.get_logger().warn("Could not parse scene_names, using empty list.")
            self.scene_names = []
        self.play_all_scenes = self.get_parameter('play_all_scenes').get_parameter_value().bool_value

        self.get_logger().info(f"CityScape dataroot: {self.dataroot}")

        # Publisherの作成
        self.cv_bridge = CvBridge()
        self.image_publishers = {
            'front_image': self.create_publisher(Image, '/cityscape/front/image_raw', 10),
            'right_image': self.create_publisher(Image, '/cityscape/right/image_raw', 10),
        }
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/cityscape/front/camera_info', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/cityscape/gps/fix', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/cityscape/odom', 10)
        self.temp_publisher = self.create_publisher(Temperature, '/cityscape/weather/temperature', 10)

        self._prepare_scene_list()

        if not self.left_image_files:
            self.get_logger().error("No image files found for the selected scenes. Please check the dataroot path and scene_names.")
            rclpy.shutdown()
            return

        self.current_frame_index = 0
        self.current_scene_name = ""
        self.timer = self.create_timer(1.0 / self.publish_rate, self.stream_data)

    def _prepare_scene_list(self):
        """再生するシーンの画像ファイルリストを作成する"""
        self.left_image_files = []
        scenes_root = os.path.join(self.dataroot, 'leftImg8bit')

        if not os.path.isdir(scenes_root):
            self.get_logger().error(f"Directory not found: {scenes_root}")
            return

        available_scenes = sorted([d for d in os.listdir(scenes_root) if os.path.isdir(os.path.join(scenes_root, d))])

        scenes_to_play = []
        if self.play_all_scenes:
            self.get_logger().info(f"Playing all {len(available_scenes)} available scenes.")
            scenes_to_play = available_scenes
        else:
            scenes_to_play = [s for s in self.scene_names if s in available_scenes]
            self.get_logger().info(f"Playing specified scenes: {scenes_to_play}")

        for scene in scenes_to_play:
            scene_path = os.path.join(scenes_root, scene)
            files = sorted(glob.glob(os.path.join(scene_path, '*_leftImg8bit.png')))
            self.left_image_files.extend(files)


    def stream_data(self):
        if self.current_frame_index >= len(self.left_image_files):
            self.get_logger().info("Finished streaming all scenes.")
            self.timer.cancel()
            return

        left_img_path = self.left_image_files[self.current_frame_index]
        base_name = os.path.basename(left_img_path).replace('_leftImg8bit.png', '')
        city_name = os.path.basename(os.path.dirname(left_img_path))

        if city_name != self.current_scene_name:
            self.current_scene_name = city_name
            self.get_logger().info(f"Streaming scene: {self.current_scene_name}")

        vehicle_json_path = os.path.join(self.dataroot, 'vehicle', city_name, f"{base_name}_vehicle.json")
        sequence_id = "_".join(base_name.split('_')[:2])
        weather_json_path = os.path.join(self.dataroot, 'weather', city_name, f"{sequence_id}_weather.json")

        vehicle_data = {}
        if os.path.exists(vehicle_json_path):
            with open(vehicle_json_path, 'r') as f:
                vehicle_data = json.load(f)

        weather_data = {}
        if os.path.exists(weather_json_path):
            with open(weather_json_path, 'r') as f:
                weather_data = json.load(f)

        timestamp_ns = vehicle_data.get('timestamp_ns')
        if timestamp_ns:
            seconds = timestamp_ns // 1_000_000_000
            nanoseconds = timestamp_ns % 1_000_000_000
            ros_timestamp = Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        else:
            self.get_logger().warn(f"Timestamp not found in {vehicle_json_path}. Falling back to current ROS time.")
            ros_timestamp = self.get_clock().now().to_msg()

        self.publish_images(left_img_path, ros_timestamp)
        self.publish_camera_info(left_img_path, ros_timestamp)
        self.publish_gps(vehicle_data, ros_timestamp)
        self.publish_odometry(vehicle_data, ros_timestamp)
        self.publish_temperature(weather_data, ros_timestamp)

        self.current_frame_index += 1

    def publish_images(self, left_img_path, timestamp):
        if os.path.exists(left_img_path):
            img = cv2.imread(left_img_path)
            self.publish_image(img, 'front_image', timestamp)

        right_img_path = left_img_path.replace('leftImg8bit.png', 'rightImg8bit.png').replace('leftImg8bit', 'rightImg8bit')
        if os.path.exists(right_img_path):
            img = cv2.imread(right_img_path)
            self.publish_image(img, 'right_image', timestamp)

    def publish_image(self, image, publisher_key, timestamp):
        img_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = publisher_key
        self.image_publishers[publisher_key].publish(img_msg)

    def publish_camera_info(self, image_path, timestamp):
        base_name = os.path.basename(image_path).replace('_leftImg8bit.png', '')
        city_name = os.path.basename(os.path.dirname(image_path))
        camera_json_path = os.path.join(self.dataroot, 'camera', city_name, f"{base_name}_camera.json")

        if not os.path.exists(camera_json_path):
            return

        with open(camera_json_path, 'r') as f:
            cam_data = json.load(f)

        info_msg = CameraInfo()
        info_msg.header.stamp = timestamp
        info_msg.header.frame_id = "front_image" # 左カメラのフレームIDに合わせる
        info_msg.height = cam_data.get('imgHeight', 1024)
        info_msg.width = cam_data.get('imgWidth', 2048)

        # 歪みパラメータ
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0] * 5 # Cityscapesは歪み補正済みのためゼロ

        # 内部パラメータ行列 K
        intrinsic = cam_data.get('intrinsic', {})
        fx = intrinsic.get('fx', 0.0)
        fy = intrinsic.get('fy', 0.0)
        u0 = intrinsic.get('u0', 0.0)
        v0 = intrinsic.get('v0', 0.0)
        info_msg.k = [fx, 0.0, u0,
                      0.0, fy, v0,
                      0.0, 0.0, 1.0]

        # 回転行列 R (歪み補正済みのため単位行列)
        info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]

        # 投影行列 P
        info_msg.p = [fx, 0.0, u0, 0.0,
                      0.0, fy, v0, 0.0,
                      0.0, 0.0, 1.0, 0.0]

        self.camera_info_publisher.publish(info_msg)

    def publish_gps(self, vehicle_data, timestamp):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = timestamp
        gps_msg.header.frame_id = "gps_link"
        if 'gpsLatitude' in vehicle_data and 'gpsLongitude' in vehicle_data:
            gps_msg.latitude = vehicle_data.get('gpsLatitude', 0.0)
            gps_msg.longitude = vehicle_data.get('gpsLongitude', 0.0)
            gps_msg.altitude = 0.0
            gps_msg.status.status = 0
        else:
            gps_msg.status.status = -1
        self.gps_publisher.publish(gps_msg)

    def publish_odometry(self, vehicle_data, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = vehicle_data.get('speed', 0.0)
        odom_msg.twist.twist.angular.z = vehicle_data.get('yawRate', 0.0)
        self.odom_publisher.publish(odom_msg)

    def publish_temperature(self, weather_data, timestamp):
        temp_msg = Temperature()
        temp_msg.header.stamp = timestamp
        temp_msg.header.frame_id = "weather_sensor"
        temp_msg.temperature = float(weather_data.get('temperature', 0.0))
        temp_msg.variance = 0.0
        self.temp_publisher.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CityScapeRos2Streamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

