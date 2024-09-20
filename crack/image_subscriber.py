import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from . import detect  # 相対インポート   

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # 画像を受信するためのサブスクリプションを作成
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.subscription  # 未使用の変数警告を防ぐ
        self.bridge = CvBridge()
        self.image_received = False
        self.image = None
        self.get_logger().info('Image subscriber initialized.')

    def listener_callback(self, msg):
        if self.image_received:
            # すでに画像が処理されている場合、以降の処理をスキップ
            return

        try:
            # ROSのImageメッセージをOpenCVの画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received = True
            self.image = cv_image

            # 画像を処理（点の抽出、ワープなど）
            processed_image = detect.extract_test_piece(cv_image)

            # 処理された画像を表示
            # cv2.imshow('Processed Image', processed_image)
            cv2.waitKey(1)

            # 処理後、ノードをシャットダウンするか、トピックからの購読を解除
            self.get_logger().info('Image processed successfully, shutting down node.')
            self.destroy_node()  # 1つの画像を処理後にノードをシャットダウン

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()