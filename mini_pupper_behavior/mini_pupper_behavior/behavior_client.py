# Inspired by the ackage miini_pupper_dance from Mangdang

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mini_pupper_interfaces.action import BehaviorCommand
import time

class MiniPupperBehaviorClient(Node):
    def __init__(self):
        super().__init__('mini_pupper_behavior_client')
        self.behavior_action_cli = ActionClient(self, BehaviorCommand, 'behavior_command')
        self.sequence_completed = False  # Add completion flag

        # Define test sequences - Test state detection and transitions
        # from these choices: 
        # sit, stand, lay, pause, breathing, move_forward, move_backward, move_left, 
        # move_right, turn_left, turn_right, yes, no, shake, bow,
        # look_up, look_down, look_left, look_right, move_up, move_down, shift_left, shift_right
        self.test_commands = [
            'default',
            'look_left',
            'look_right',
        ]

    def send_move_request(self, move_command):
        goal_msg = BehaviorCommand.Goal()
        goal_msg.data = move_command

        self.behavior_action_cli.wait_for_server()
        self.sequence_completed = False  # Reset completion flag
        self._send_goal_future = self.behavior_action_cli.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.status}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.executed}')
        self.sequence_completed = True  # Add flag to indicate completion
        return result

def main():
    rclpy.init()
    action_client = MiniPupperBehaviorClient()
    
    # Wait for the action server to be ready
    time.sleep(2)

    for command in action_client.test_commands:
        action_client.get_logger().info(f'Sending command: {command}')
        action_client.send_move_request(command)
        
        # Wait for the action to complete
        rclpy.spin_until_future_complete(action_client, action_client._send_goal_future)
        
        # Wait for the action to complete by spinning until sequence_completed flag is set
        while not action_client.sequence_completed:
            rclpy.spin_once(action_client, timeout_sec=0.1)
        
        action_client.get_logger().info(f'Command "{command}" completed!')

    action_client.get_logger().info('All sequences completed!')
    action_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()