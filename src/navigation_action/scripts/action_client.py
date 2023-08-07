import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from navigation_action_msg.action import CarNavigate
import time
from rclpy.executors import MultiThreadedExecutor

class carFollowingClient(Node):

    def __init__(self):
        super().__init__('car_following_client')
        self._action_client = ActionClient(self, CarNavigate, 'car_navigation_action')
        self.goals = []  # Initialize an empty goal list
        self.is_running_goal = False;

    def add_goal(self, x, y):
        self.goals.append((x, y))
        self.get_logger().info(f"add_goal, left:{len(self.goals)} point: {x}, {y}")

    def send_goals(self):
        if len(self.goals) == 0:
            return False
                
        if self.is_running_goal == False:
            x, y = self.goals.pop(0)  # Get the next goal from the list
            self.send_goal(x,y)
            self.is_running_goal = True
            self.get_logger().info(f"Send goal, left:{len(self.goals)} point: {x}, {y}")
        return True               

    def send_goal(self, x , y):
        goal_msg = CarNavigate.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        self.is_running_goal = False        
        self.get_logger().info('===>>>Result: {0}'.format(result.finish))


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Receive feedback({feedback.x},{feedback.y})")


def main(args=None):
    rclpy.init(args=args)

    action_client = carFollowingClient()
    # Add multiple goals
    action_client.add_goal(1.0, 1.0)
    action_client.add_goal(1.0, 2.0)
    action_client.add_goal(2.0, 1.0)
    # action_client.send_goal(float(sys.argv[1]), float(sys.argv[2]))
    has_goal= True
    while rclpy.ok :# and has_goal:
        has_goal = action_client.send_goals()
        rclpy.spin_once(action_client, timeout_sec=0.03)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()