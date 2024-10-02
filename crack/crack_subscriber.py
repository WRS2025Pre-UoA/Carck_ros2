import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
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
        self.image_publisher = self.create_publisher(Image, 'crack_result_image', 10)
        # self.value_publisher = self.create_publisher(Float64, 'crack_result_value', 10)
        self.value_publisher = self.create_publisher(String, 'crack_result_txt', 10)
        self.bridge = CvBridge()
        self.image_count = 1  # 画像のカウントを初期化
        #self.output_dir = '/mnt/c/Users/motti/Desktop/Carck_ros2/Output_Images'  # 保存先ディレクトリ
        #os.makedirs(self.output_dir, exist_ok=True)  # 保存先ディレクトリが存在しない場合は作成

    def listener_callback(self, msg):
        try:
            # 受信したROS ImageメッセージをOpenCV形式に変換
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # 画像処理（線の長さを計測）
            processed_image,result1,result2 = detect.extract_test_piece(img)

            # cv2.putText(img, f"Length: {result1:.2f} mm", (x1, y1 - 10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
            # cv2.putText(img, f"Thickness: {result2:.1f} mm", (x1, y1 - 30),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

            # print(f"Result: {result1:.4f} mm")
            # print(f"Result: {result2:.4f} mm")
            
            # 保存ファイル名を生成（例: processed_image1.png, processed_image2.png, ...）
            #output_file = os.path.join(self.output_dir, f'processed_image{self.image_count}.png')

            # 処理済み画像を保存
            #cv2.imwrite(output_file, processed_image)
            #self.get_logger().info(f'Saved processed image as {output_file}')
            
            # カウントをインクリメント
            #self.image_count += 1

            # 必要ならここでノードの終了処理を呼び出す
            # self.destroy_node()
            
            # # 0キーが押された場合にノードを終了
            # if cv2.waitKey(1) & 0xFF == ord('0'):
            #     self.get_logger().info("Process terminated by user pressing 0.")
            #     self.destroy_node()
            #     rclpy.shutdown()

            h,w = processed_image.shape[:2]
            nw = 1280
            aspect = w/h
            nh = int(nw / aspect)
            process_image = cv2.resize(processed_image,(nw,nh))

            cv2.imshow("result",process_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            if result1 != None and result2 != None:
                txt = str(result1)+","+str(result2)
                # result_value = Float64()
                result_value = String()
                result_value.data = txt
                self.value_publisher.publish(result_value)
                ros_image = self.bridge.cv2_to_imgmsg(process_image, 'bgr8')#くり抜いた画像に結果も貼り付けた
                # ros_image = self.bridge.cv2_to_imgmsg(img, 'bgr8')#提出を元画像に結果を貼り付けた
                self.image_publisher.publish(ros_image)

        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
