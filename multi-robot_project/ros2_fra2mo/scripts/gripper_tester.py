#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # Publisher per il movimento del robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher per il controllo della presa
        self.gripper_publisher = self.create_publisher(Bool, '/fra2mo/gripper/grasp', 10)
        
        self.get_logger().info('Tester del Gripper (Presa) avviato.')
        
    def move_robot(self, linear_x, angular_z, duration_sec):
        """Pubblica Twist per muovere il robot per una durata specificata."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Movimento: L:{linear_x:.2f}, A:{angular_z:.2f} per {duration_sec}s')
        
        # Attendi la durata
        time.sleep(duration_sec)
        
        # Ferma il robot
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('Robot fermato.')


    def control_gripper(self, grasp):
        """Invia il comando di presa (True per Prendi, False per Lascia)."""
        msg = Bool()
        msg.data = grasp
        
        # Pubblichiamo due volte per maggiore sicurezza
        self.gripper_publisher.publish(msg)
        time.sleep(0.5)
        self.gripper_publisher.publish(msg) 
        
        action = "Presa ATTIVATA (True)" if grasp else "Presa DISATTIVATA (False)"
        self.get_logger().warn(f'Comando Gripper Inviato: {action}')


    def run_test(self):
        """Sequenza di test: Avvicinati, Prendi, Allontanati, Lascia."""
        
        # 1. Avvicinati alla scatola (simulazione)
        # Muovi a 0.1 m/s per 2.0 secondi (Distanza percorsa = 0.2m)
        self.move_robot(0.1, 0.0, 2.0) # Correzione movimento
        time.sleep(1.0) # Pausa
        
        # 2. Attiva la presa
        self.control_gripper(True)
        time.sleep(2.0) # Pausa per l'attacco
        
        # 3. Allontanati con l'oggetto attaccato
        self.move_robot(-0.1, 0.0, 4.0)
        time.sleep(1.0) # Pausa
        
        # 4. Disattiva la presa (rilascia l'oggetto)
        self.control_gripper(False)
        time.sleep(2.0)
        
        self.get_logger().info('Sequenza di test completata.')
        
        
def main(args=None):
    rclpy.init(args=args)
    gripper_tester = GripperTester()
    
    try:
        gripper_tester.run_test()
    except Exception as e:
        gripper_tester.get_logger().error(f"Errore durante l'esecuzione del test: {e}")
    finally:
        gripper_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
