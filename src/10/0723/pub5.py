import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

class PublisherNode(Node):
    def __init__(self, message):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        self.get_logger().info(f'    wait create_publisher')
        time.sleep(3)
        self.message = message
        # 直接在初始化時發佈消息
        self.publish_message()

    def publish_message(self):
        self.publisher_.publish(String(data=self.message))
        self.get_logger().info(f'Publishing: "{self.message}"')
        time.sleep(1)
        flg_running=False
        self.get_logger().info(f'    quit')
        # 發佈完畢後關閉節點
        #self.destroy_node()
        #rclpy.shutdown()

def main(args=None):
    flg_running=False
    rclpy.init(args=args)
    
    # 確保有足夠的參數
    if len(sys.argv) != 2:
        print("Usage: python3 script.py '<int> <float> <float> <float>'")
        return
    
    # 獲取字串參數
    message = sys.argv[1]

    node1 = PublisherNode(message)
    # 不需要 spin，因為我們只發佈一次
    #rclpy.spin_once(node1, timeout_sec=0)  # 確保事件循環運行一次以發佈消息
    
    try:
        while rclpy.ok() and flg_running:
            pass
    except KeyboardInterrupt:
        pass
    finally:   
        node1.destroy_node()         
        rclpy.shutdown()            
    
if __name__ == '__main__':
    main()
