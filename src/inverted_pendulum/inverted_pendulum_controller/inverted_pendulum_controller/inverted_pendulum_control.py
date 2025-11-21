import rclpy
from rclpy.node import Node
# from rclpy.exceptions import RCLError
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
# from controllers import LQRController
# from controllers.lqr_controller import LQRController
from .controllers.lqr_controller import LQRController

class InvertedPendulumControllerNode(Node):
    def __init__(self):
        super().__init__('inverted_pendulum_controller')
        
        # Publisher: send force commands to the cart joint
        # self.force_pub = self.create_publisher(Float64, '/cart_joint/cmd_force', 10)  # speak to the ros2 topic "/cart_joint/cmd_force". This is bridged to an equivalent gazebo topic in the inverted_pendulum_bridge.yaml
        self.force_pub = self.create_publisher(Float64MultiArray, '/cart_effort_controller/commands', 10)  # speak directly to ros2 controller "cart_effort_controller" defined in /src/config/cart_controllers.yaml
        
        # Subscriber: receive joint states (assumes joint names "cart_joint" and "pendulum_joint")
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.state = None  # to hold our 4D state vector [cart_pos, cart_vel, pend_angle, pend_ang_vel]
        self.setpoint = np.zeros(4)  # desired state (e.g., upright pendulum, zero cart motion)
        
        # Initialize the LQR controller with a given saturation limit (e.g., max force)
        self.controller = LQRController(saturation=10.0)
        
        # Define system dynamics constants and matrices (example values)
        M = 2.0  # cart mass
        m = 1.0  # pendulum mass
        g = 9.81
        L = 1.0  # pendulum length
        
        alpha = m * g / M
        beta = g * (M + m) / (L * M)
        A = np.array([[0, 1, 0, 0],
                      [0, 0, alpha, 0],
                      [0, 0, 0, 1],
                      [0, 0, beta, 0]])
        B = np.array([[0],
                      [1 / M],
                      [0],
                      [-1 / (L * M)]])
        Q = 0.1 * np.eye(4)
        R = np.array([[0.01]])
        
        self.controller.solveOCP(A, B, Q, R, debug=True)
        
        # Set up a timer for the control loop (e.g., 100 Hz)
        timer_period = 0.01  # seconds
        self.create_timer(timer_period, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        # Extract state values assuming the JointState message contains both "cart_joint" and "pendulum_joint"
        try:
            cart_idx = msg.name.index('cart_joint')
            pend_idx = msg.name.index('pendulum_joint')
            cart_pos = msg.position[cart_idx]
            cart_vel = msg.velocity[cart_idx]
            pend_angle = msg.position[pend_idx]
            pend_ang_vel = msg.velocity[pend_idx]
            self.state = np.array([cart_pos, cart_vel, pend_angle, pend_ang_vel])
            self.get_logger().debug(f"State updated: {self.state}")
        except ValueError:
            self.get_logger().warn("JointState message missing expected joint names.")

    def control_loop(self):
        if self.state is None:
            return  # wait until state is available
        
        # Compute control force using the LQR controller
        force_command = self.controller.control(self.state, self.setpoint, debug=True)
        
        # Publish the force command
        # cmd_msg = Float64()
        # cmd_msg.data = float(force_command)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(force_command)]
        self.force_pub.publish(cmd_msg)
        self.get_logger().debug(f"Published force command: {force_command}")

def main(args=None):
    rclpy.init(args=args)
    node = InvertedPendulumControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # node.destroy_node()
    # rclpy.shutdown()
    finally:  # handle errors where the rclpy.shutdown() is called twice for some reason
        node.destroy_node()
        # rclpy.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
        # except RCLError:
            pass  # Ignore if shutdown has already been called
        
if __name__ == '__main__':
    main()
