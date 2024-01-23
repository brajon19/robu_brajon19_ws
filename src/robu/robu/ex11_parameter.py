import rclpy

#used to create nodes
from rclpy.node import Node

class MinimalParameter(Node):
    def __init__(self):
        super().__init__('MinimalParameter')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('forward_speed_wf_fast',0.1),
                ('forward_speed_wf_slow',0.05),
                ('turning_speed_wf_fast',1.0),  # Fast turn
                ('turning_speed_wf_slow',0.1), # Slow turn
                ('dist_thresh_wf',0.3),# in meters  
                ('dist_hysteresis_wf',0.02) # in meters

            ])





        self.declare_parameter('my_parameter')

        my_param=self.get_parameter('my_parameter')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):

        forward_speed_wf_slow = self.get_parameter('forward_speed_wf_slow').value
        forward_speed_wf_fast = self.get_parameter('forward_speed_wf_fast').value
        turning_speed_wf_fast = self.get_parameter('turning_speed_wf_fast').value
        turning_speed_wf_slow = self.get_parameter('turning_speed_wf_slow').value
        dist_thresh_wf = self.get_parameter('dist_thresh_wf').value
        dist_hysteresis_wf = self.get_parameter('dist_hysteresis_wf').value

        self.get_logger().info('forward_speed_wf_slow: %5.2f, forward_speed_wf_fast: %5.2f'
                                %(forward_speed_wf_slow,forward_speed_wf_fast))
        self.get_logger().info('turning_speed_wf_fast: %5.2f, turning_speed_wf_slow: %5.2f'
                                %(turning_speed_wf_fast,turning_speed_wf_slow))
        self.get_logger().info('dist_thresh_wf: %5.2f, dist_hysteresis_wf: %5.2f'
                                %(dist_thresh_wf,dist_hysteresis_wf))

       # print("my_parameter:",my_param)
def main():
    rclpy.init()

    node = MinimalParameter()

    rclpy.spin(node)

