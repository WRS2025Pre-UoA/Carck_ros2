import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from . import detect  # 相対インポート

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 1  # 画像のカウントを初期化
        #self.output_dir = '/mnt/c/Users/motti/Desktop/Carck_ros2/Output_Images'  # 保存先ディレクトリ
        #os.makedirs(self.output_dir, exist_ok=True)  # 保存先ディレクトリが存在しない場合は作成

    def listener_callback(self, msg):
        # 受信したROS ImageメッセージをOpenCV形式に変換
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 画像処理（線の長さを計測）
        processed_image = detect.extract_test_piece(img)

        # 保存ファイル名を生成（例: processed_image1.png, processed_image2.png, ...）
        #output_file = os.path.join(self.output_dir, f'processed_image{self.image_count}.png')

        # 処理済み画像を保存
        #cv2.imwrite(output_file, processed_image)
        #self.get_logger().info(f'Saved processed image as {output_file}')
        
        # カウントをインクリメント
        #self.image_count += 1

        # 必要ならここでノードの終了処理を呼び出す
        # self.destroy_node()
        
        # 0キーが押された場合にノードを終了
        if cv2.waitKey(1) & 0xFF == ord('0'):
            self.get_logger().info("Process terminated by user pressing 0.")
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
