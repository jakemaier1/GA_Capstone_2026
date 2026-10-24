#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建压缩参数
        self.compression_params = [
            int(cv2.IMWRITE_JPEG_QUALITY),  # 使用JPEG格式
            80  # 压缩质量 (0-100)
        ]
        
        # 创建订阅者（原始图像）
        self.subscription = self.create_subscription(Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.image_callback,
            10  # 队列大小
        )
        
        # 创建发布者（压缩图像）
        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/rgb/image_raw/compressed',
            10  # 队列大小
        )
        
        self.get_logger().info("图像压缩节点已启动，正在监听原始图像并发布压缩图像...")

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 压缩图像
            _, compressed_data = cv2.imencode(
                '.jpg', 
                cv_image, 
                self.compression_params
            )
            
            # 创建压缩图像消息
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_data.tobytes()
            
            # 发布压缩图像
            self.publisher.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()