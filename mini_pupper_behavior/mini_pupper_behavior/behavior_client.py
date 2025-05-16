# Inspired by the ackage miini_pupper_dance from Mangdang

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import BehaviorCommand
import time

class MiniPupperBehaviorClientAsync(Node):
    def __init__(self):
        super().__init__('mini_pupper_behavior_client_async')
        self.behavior_cli = self.create_client(BehaviorCommand, 'behavior_command')

        while not self.behavior_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        yes= ['look_up',
              'look_middle',
              'look_up',
              'look_middle']
        
        no= ['look_right',
             'look_left',
             'look_right',
             'look_middle']
        
        shake= ['shift_left',
                'shift_right',
                'shift_left',
                'shift_right',
                'look_middle']
        bow= ['look_down',
              'stay',
              'stay',
              'look_middle']
        
        self.move_commands = ['stay'] + yes + ['stay'] + no + ['stay'] + shake + ['stay'] + bow + ['stay']
        # self.move_commands = ['move_down']


        # there are 10 commands you can choose:
        # move_forward: the robot will move forward
        # move_backward: the robot will move backward
        # move_left: the robot will move to the left
        # move_right: the robot will move to the right
        # look_up: the robot will look up
        # look_down: the robot will look down
        # look_left: the robot will look left
        # look_right: the robot will look right
        # look_middle: the robot will return to the default standing posture
        # stay: the robot will keep the last command
        # shift_left: the robot will shift to the left
        # shift_right: the robot will shift to the right

    def send_move_request(self, move_command):
        req = BehaviorCommand.Request()
        req.data = move_command
        future = self.behavior_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
def main():
    rclpy.init()
    minimal_client = MiniPupperBehaviorClientAsync()

    for index, command in enumerate(minimal_client.move_commands):
        # Send movemoment comment for the robot to dance
        response = minimal_client.send_move_request(command)
        if response.executed:
            minimal_client.get_logger().info('Command from client:' + command)

    minimal_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()