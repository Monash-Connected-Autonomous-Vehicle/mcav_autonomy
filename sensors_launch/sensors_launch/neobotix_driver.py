import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Range
from can_msgs.msg import Frame


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('neobotix_driver')
        self.subscription = self.create_subscription(
            Frame,
            '/from_can_bus',
            self.listener_callback,
            10)
        self.subscription  
        self.sensor_data_402=[0,0,0,0]
        self.sensor_data_403=[0,0,0,0]
    def listener_callback(self, msg):
        self.get_data(msg)
        # if msg.id == 0x402:
        #     self.get_logger().info('Ultrasonic_402: "%s"' % str(msg.data))
        # else:
        #     self.get_logger().info('Ultrasonic_403: "%s"' % str(msg.data)) 

    def get_data(self, can_frame):
        #402 or 0x402
        if can_frame.id ==0x402:
            #sensor 1 2 3 4
            self.sensor_data_402 = [can_frame.data[2], can_frame.data[3], can_frame.data[4], can_frame.data[5]]
            self.get_logger().info(f'402 : {self.sensor_data_402}')
        elif can_frame.id ==0x403:
            #sensor 5 6 7 8
            self.sensor_data_403 = [can_frame.data[2], can_frame.data[3], can_frame.data[4], can_frame.data[5]]
            self.get_logger().info(f"403 : {self.sensor_data_402}")
        
    



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
   