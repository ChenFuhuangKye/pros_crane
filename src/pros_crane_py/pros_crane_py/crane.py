import time
from rclpy.node import Node
import rclpy
from rclpy.duration import Duration
import orjson
from std_msgs.msg import String
import curses
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from pros_crane_py.ENV import *

class CraneKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 10):
        super().__init__('crane_keyboard')        

        # Subscriber
        self.subscription = self.create_subscription(
            String,
            CRANE_STATE, # topic name
            self._sub_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(
            String,
            CRANE_CONTROL,  # topic name
            10
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, ARM_CONTROL, 10)
        self.joint_pos = [1.57, 1.57, 1.57, 1.57]
        self.stdscr = stdscr

    def _sub_callback(self, msg):        
        self._car_state_msg = str(self.get_clock().now()) + " " + msg.data

    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()
                movement = [0, 0, 0]
                # Check if a key was actually pressed
                if c != curses.ERR:
                    if c == ord('w') or c == ord('W'):
                        movement = [0, 1, 0]                    
                    elif c == ord('s') or c == ord('S'):
                        movement = [0, -1, 0]
                    elif c == ord('a') or c == ord('A'):
                        movement = [1, 0, 0]
                    elif c == ord('d') or c == ord('D'):
                        movement = [-1, 0, 0]
                    elif c == ord('z') or c == ord('Z'):
                        movement = [0, 0, 1]
                    elif c == ord('c') or c == ord('C'):
                        movement = [0, 0, -1]
                    elif c == ord('j'):
                        self.handle_key_j()
                    elif c == ord('k'):
                        self.handle_key_k()
                    elif c == ord('l'):
                        self.handle_key_l()
                    elif c == ord('u'):
                        self.handle_key_u()
                    elif c == ord('o'):
                        self.handle_key_o()
                    elif c == ord('q'):
                        self.pub_crane_control(movement)
                        break
                    
                    self.pub_crane_control(movement)
                    self.pub_arm()
                else:
                    # self.print_basic_info(ord(' '))
                    time.sleep(0.1)
                
        finally:
            curses.endwin()
    
    def pub_arm(self):
        msg = JointTrajectoryPoint()
        msg.positions = self.joint_pos
        msg.velocities = [0.0, 0.0, 0.0, 0.0]

        self.joint_trajectory_publisher_.publish(msg)

    def pub_crane_control(self, movement):
        # Generate a random control signal
        control_signal = {
            "type": CRANE_CONTROL,
            "data": dict(CraneMovementControl(
                moveX = movement[0],
                moveY = movement[1],
                moveZ = movement[2]
            ))
        }
        # Convert the control signal to a JSON string
        control_msg = String()
        control_msg.data = orjson.dumps(control_signal).decode()

        # Publish the control signal
        self.publisher.publish(control_msg)

        # self.get_logger().info(f'publish {control_msg}')
    
    def handle_key_i(self):
        self.stdscr.addstr(f"arm rift up")
        self.joint_pos[2] += 0.05
        pass

    def handle_key_j(self):
        self.stdscr.addstr(f"arm turn left")
        self.joint_pos[0] -= 0.05
        pass

    def handle_key_k(self):
        self.stdscr.addstr(f"arm rift down")
        self.joint_pos[2] -= 0.05
        pass

    def handle_key_l(self):
        self.stdscr.addstr(f"arm turn right")
        self.joint_pos[0] += 0.05
        pass

    def handle_key_u(self):
        self.stdscr.addstr(f"arm j4 rotate left")
        self.joint_pos[3] -= 0.05
        pass

    def handle_key_o(self):
        self.stdscr.addstr(f"arm j4 rotate right")
        self.joint_pos[3] += 0.05
        pass

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = CraneKeyboardController(stdscr)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped


if __name__ == '__main__':
    main()