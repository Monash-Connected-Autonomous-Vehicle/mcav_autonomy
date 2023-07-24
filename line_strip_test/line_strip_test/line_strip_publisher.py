import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class LineStripPublisher(Node):
    def __init__(self):
        super().__init__('line_strip_publisher')
        self.publisher_ = self.create_publisher(Marker, 'line_strip_topic', 10)
        self.timer_ = self.create_timer(1.0, self.draw_steering_path)
        self.line_strip_ = self.create_line_strip()

    def create_line_strip(self):
        line_strip = Marker()
        line_strip.header.frame_id = 'map'
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = Marker.ADD
        line_strip.pose.orientation.w = 1.0
        line_strip.scale.x = 0.1
        line_strip.color.r = 1.0
        line_strip.color.a = 1.0
        return line_strip

    def draw_steering_path(self):
        gamma = 0.6
        self.get_logger().info(f'Gamma: {gamma}')
        step = 0.2
        upper = 1/gamma * 2
        y = -1/gamma * 2
        max_val = 20
        
        #set lower bound for visualisation
        if gamma  < 0:
            y = min(y, max_val) #y > 0
            upper = max(upper, -max_val) #upper < 0, 
            step *= -1
        else:
            y = max(y, -max_val) #y < 0
            upper = min(upper, max_val) #upper > 0

        origin = Point(x=0.0, y=0.0, z=0.0)
        points = [origin]
        points_neg = [origin]

        while True:
            if step > 0 and round(y,2) > upper + step:
                break
            if step < 0 and round(y,2) < upper + step:
                break
                
            val = (1/abs(gamma))**2 - (y - 1/gamma)** 2

            if val < 0:
                self.get_logger().info(f'Failed -- Val: {val}')
                y += step
                continue

            self.get_logger().info(f'Passed - Val: {val}')

            x = float(math.sqrt(val))
            points.append(Point(x=x, y=y, z=0.0))
            points_neg.append(Point(x=-x, y=y, z=0.0))

            y += step

        points_neg = reversed(points_neg)
        points += points_neg

        self.line_strip_.points = points
        self.publisher_.publish(self.line_strip_)

def main(args=None):
    rclpy.init(args=args)
    line_strip_publisher = LineStripPublisher()
    rclpy.spin(line_strip_publisher)
    line_strip_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
