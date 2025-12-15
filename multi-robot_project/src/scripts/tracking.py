import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
import math


class IiwaRobustTracking(Node):

    def __init__(self):
        super().__init__('iiwa_robust_tracking')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pos_pub = self.create_publisher(
            Float64MultiArray,
            '/iiwa_arm_controller/commands',
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.alpha = 0.15 

        # Memoria (Posizioni di partenza)
        self.last_j1 = 0.0
        self.last_j2 = 0.5   
        self.last_j4 = -1.2  
        self.last_j6 = 0.0 # Partenza orizzontale

        self.get_logger().info("TRACKING ATTIVATO.")

    def control_loop(self):
        target_j1 = self.last_j1
        target_j2 = self.last_j2
        target_j4 = self.last_j4
        target_j6 = self.last_j6

        try:
            t = self.tf_buffer.lookup_transform(
                'iiwa_base',
                'base_footprint',
                Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            dist_xy = math.sqrt(x**2 + y**2)

            # --- 1. J1 (YAW) - Punta orizzontalmente ---
            raw_j1 = math.atan2(y, x)

            # --- 2. FATTORE DISTANZA (0m - 7m) ---
            d_min = 0.0
            d_max = 7.0
            
            if dist_xy <= d_min:
                factor = 0.0
            elif dist_xy >= d_max:
                factor = 1.0
            else:
                factor = (dist_xy - d_min) / (d_max - d_min)

            # --- 3. J2 e J4 (Estensione braccio) ---
            raw_j2 = 0.5 + (0.4 * factor)
            raw_j4 = -1.2 + (0.7 * factor)

            angle_to_target_z = math.atan2(z, dist_xy)
            arm_compensation = -(raw_j2 + raw_j4)
            
            raw_j6 = arm_compensation + angle_to_target_z

            # --- SMOOTHING ---
            if abs(raw_j1 - self.last_j1) > 4.0:
                self.last_j1 = raw_j1

            target_j1 = self.alpha * raw_j1 + (1 - self.alpha) * self.last_j1
            target_j2 = self.alpha * raw_j2 + (1 - self.alpha) * self.last_j2
            target_j4 = self.alpha * raw_j4 + (1 - self.alpha) * self.last_j4
            target_j6 = self.alpha * raw_j6 + (1 - self.alpha) * self.last_j6

            # Aggiorno memoria
            self.last_j1 = target_j1
            self.last_j2 = target_j2
            self.last_j4 = target_j4
            self.last_j6 = target_j6

        except TransformException:
            pass

        msg = Float64MultiArray()
        msg.data = [
            target_j1, 
            target_j2, 
            0.0,       # J3
            target_j4, 
            0.0,       # J5
            target_j6, # J6 Dinamico 
            0.0        # J7 Fisso
        ]
        self.pos_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = IiwaRobustTracking()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
