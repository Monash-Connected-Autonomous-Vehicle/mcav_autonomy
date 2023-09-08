import rcply 
from rcply.node import Node
from can_msgs.msg import Frame

class Neobotrix(Node):
    def __init__(self):
        super().__init__('neobotrix_driver')
        self.subscription = self.create_subscription(Frame, '/from_can_bus', self.something, 10)
        self.subscription
        
def something(self,msg):
    if msg.id == 0x402 or msg.id == 0x403:
        self.get_logger().info(f'Acquired sensor message from CAN frame {msg.id}: {msg.data}')

def main(args=None):
    rcply.init(args=args)
    node = Neobotrix()
    rcply.spin(node)
    node.destroy_node()
    rcply.shutdown()

if __name__ == '__main__':
    main()