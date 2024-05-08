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



class ArmKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 10):
        super().__init__('arm_keyboard')
        self.vel = vel

        # Subscriber
        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint, 'joint_trajectory_point', 10)
        self.joint_pos = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.0]
        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        
                

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        # TODO show data in screen
        self._car_state_msg = str(self.get_clock().now()) + " " + msg.data

   

    def run(self, vel=None):
        if vel is None:
            vel = self.vel
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    
                    if c == ord('i'):
                        self.handle_key_i()
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
                    elif c == ord('y'):
                        self.handle_key_y()
                    elif c == ord('h'):
                        self.handle_key_h()
                    elif c == ord('t'):
                        self.handle_key_t()
                    elif c == ord('g'):
                        self.handle_key_g()
                    elif c == ord('r'):
                        self.handle_key_r()
                    elif c == ord('f'):
                        self.handle_key_f()
                    elif c == ord('e'):
                        self.handle_key_e()
                    elif c == ord('d'):
                        self.handle_key_d()
                    elif c == ord('b'):
                        self.handle_key_b()

                    elif c == ord('q'):  # Exit on 'q'
                        
                        break                    
                    self.pub_arm()
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.01)

            # origin_string = self.serial.readline()
            # self.stdscr.move(3, 0)
            # self.stdscr.addstr(f"{self.key_in_count:5d} receive: {origin_string} ")

        finally:
            curses.endwin()

    def print_basic_info(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        # show receive data                
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"Arm pos : {self.joint_pos}")
        self.stdscr.move(2, 0)

        # self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")
    

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

    def handle_key_y(self):
        self.stdscr.addstr(f"arm catch!")
        self.joint_pos[6] = 1.57
        pass

    def handle_key_h(self):
        self.stdscr.addstr(f"arm release!")
        self.joint_pos[6] = 1.0
        pass
    
    def handle_key_t(self):
        self.stdscr.addstr(f"arm 4!")
        self.joint_pos[4] += 0.05        
        pass

    def handle_key_g(self):
        self.stdscr.addstr(f"arm 4!")
        self.joint_pos[4] -= 0.05
        pass

    def handle_key_r(self):
        self.stdscr.addstr(f"arm 5!")
        self.joint_pos[5] += 0.05
        pass
    
    def handle_key_f(self):
        self.stdscr.addstr(f"arm 5!")
        self.joint_pos[5] -= 0.05
        pass

    def handle_key_e(self):
        self.stdscr.addstr(f"arm 1!")
        self.joint_pos[1] += 0.05
        pass

    def handle_key_d(self):
        self.stdscr.addstr(f"arm 1!")
        self.joint_pos[1] -= 0.05
        pass

    def handle_key_b(self):
        self.stdscr.addstr(f"reset arm!")
        self.joint_pos = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.0]
        pass

    def pub_arm(self):
        msg = JointTrajectoryPoint()
        msg.positions = self.joint_pos  # Replace with actual desired positions
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
        # You can set other fields of the JointTrajectoryPoint message similarly.
        self.joint_trajectory_publisher_.publish(msg)


# ... Rest of your code, e.g. initializing rclpy and running the node

def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()    
    node = ArmKeyboardController(stdscr)

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
