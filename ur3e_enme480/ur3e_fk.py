import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ur3e_mrc_msgs.msg import PositionUR3e, CommandUR3e
import sys
import math
import numpy as np
from ur3e_enme480.submodules.kinematic_functions import KinematicFunctions

class ForwardKinematicsUR3e(Node):

    def __init__(self):
        super().__init__('ur3e_fk_publisher')
        self.publisher_ = self.create_publisher(CommandUR3e, '/ur3e/command', 10)

    def send_command(self, joint_positions):
        
        for angle in joint_positions:
            if abs(angle) > 3.16:
                joint_positions = [0.0]*6

        if joint_positions[1] > 0.05:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')
            joint_positions = [0.0]*6


        ur3e_matrix = KinematicFunctions().correct_calculate_dh_transform(joint_positions)
        if ur3e_matrix[2,3] < 0.01:
            self.get_logger().info(f'z is going below the workbench. Resetting to Zero configuration')
            joint_positions = [0.0]*6  

        ######################################### YOUR CODE STARTS ########################################

        # Create a CommandUR3e message
        # SET v AND a TO BE === 0.1
        msg = CommandUR3e()
        msg.destination = joint_positions
        msg.v = 0.1
        msg.a = 0.1
        msg.io_0 = False

        
        # Publish the command
        for _ in range(50):
            if self.publisher_.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.publisher_.publish(msg)
        self.publisher_.wait_for_all_acked(Duration(seconds=2))

        ######################################### END OF YOUR CODE ########################################
        ### DO NOT CHANGE ###

        # Calculate DH transformation
        dh_matrix = self.calculate_dh_transform(joint_positions)

        print("--------------------- \n Your Laser Prediction \n---------------------")
        KinematicFunctions().predict_laser_position(dh_matrix)

    def calculate_dh_transform(self, joint_positions):
        
        
        ################## YOUR CODE STARTS HERE ###################################
        # Calculate the FINAL TRANSFORMATION MATRIX here using DH parameters

        # Insert intermediate transformation matrices here ....
        base_angle = 2.35619449  # radians
        base_transform = np.array([
            [1.0, 0.0, 0.0, 0.21213 * math.cos(base_angle)],
            [0.0, 1.0, 0.0, 0.21213 * math.sin(base_angle)],
            [0.0, 0.0, 1.0, 0.010],
            [0.0, 0.0, 0.0, 1.0],
        ])

        # Standard DH: (a in metres, alpha in radians, d in metres, theta in radians).
        # Rows 1-6 use the six joint angles; row 7 is a fixed tool transform.
        # Standard DH puts joint i about z_(i-1): the first twist must
        # separate the shoulder-pan and shoulder-lift axes.
        # Use the corrected user frame assignment; all angular constants are radians.
        # Base height 0.010 + joint-1 height 0.152 = world height 0.162 m.
        dh_params = [
            (0.0, -math.pi / 2, 0.152, joint_positions[0]),
            (0.244, 0.0, 0.120, joint_positions[1]),
            (0.213, 0.0, -0.093, joint_positions[2]),
            (0.0, math.pi / 2, -0.104, joint_positions[3] + math.pi / 2),
            (0.0, math.pi / 2, -0.083, joint_positions[4] + math.pi),
            (0.0, 0.0, 0.0, joint_positions[5]),
            (0.0, 0.0, -0.092, 0.0),
        ]

        # Final transformation matrix
        transform = base_transform.copy()
        for a, alpha, d, theta in dh_params:
            ct, st = math.cos(theta), math.sin(theta)
            ca, sa = math.cos(alpha), math.sin(alpha)
            transform = transform @ np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ])

        ################################ YOUR CODE ENDS HERE #########################
        self.get_logger().info(f'Your DH Transformation Matrix:\n{transform}')

        return transform



def main(args=None):
    rclpy.init(args=args)

    # Check command-line arguments
    if len(sys.argv) != 7:
        print("Usage: ros2 run <package_name> <script_name> <joint1> <joint2> <joint3> <joint4> <joint5> <joint6>")
        return

    try:
        # Read joint positions from command-line arguments
        joint_positions = [float(arg) for arg in sys.argv[1:7]]
    except ValueError:
        print("All joint positions must be numbers.")
        return

    node = ForwardKinematicsUR3e()

    try:
    	node.send_command(joint_positions)  # Send the command with updated joint positions
        #rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
    	node.destroy_node()
    	rclpy.shutdown()

    

if __name__ == '__main__':
    main()
