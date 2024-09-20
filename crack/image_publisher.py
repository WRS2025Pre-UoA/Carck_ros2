import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # 画像ディレクトリのパラメータを宣言
        self.declare_parameter('image_dir', '/home/ros/ros2_ws/src/crack/Input_Images')
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        # 指定したディレクトリ内の画像ファイルを取得
        self.image_files = sorted([f for f in os.listdir(self.image_dir) if f.endswith('.jpg') or f.endswith('.png')])
        self.current_image_idx = 0
        # 画像をパブリッシュするためのパブリッシャを作成
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        # タイマーを設定（1秒ごとにコールバックを呼び出す）
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # 画像ファイルがない場合の処理
        if not self.image_files:
            self.get_logger().info('No images found in directory.')
            return

        # 現在の画像ファイルを読み込む
        image_file = os.path.join(self.image_dir, self.image_files[self.current_image_idx])
        img = cv2.imread(image_file)

        # 画像の読み込みに失敗した場合の処理
        if img is None:
            self.get_logger().error(f'Failed to load image: {image_file}')
            return

        # 必要に応じて画像をリサイズ
        img = cv2.resize(img, (720, 720), interpolation=cv2.INTER_AREA)

        # ROS Image メッセージに変換
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing image {self.current_image_idx}/{len(self.image_files)}')



        # 次の画像に移動（最初に戻る）
        self.current_image_idx = (self.current_image_idx + 1) % len(self.image_files)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()