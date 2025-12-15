import time
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    # 1. Inizializza ROS 2
    rclpy.init()
    
    # 2. Crea il Navigatore
    navigator = BasicNavigator()

    # 3. Aspetta che Nav2 sia completamente attivo
    print("Aspetto che Nav2 sia pronto...")
    navigator.waitUntilNav2Active()

    # 4. Definisci l'obiettivo (Goal Pose)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # COORDINATE DEL TARGET
    goal_pose.pose.position.x = 4.2
    goal_pose.pose.position.y = -3.5
    
    # Orientamento (w=1.0 significa "guarda in avanti/est")
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    print(f"Invio il robot al Target_point x=1.5 , y=-2")
    
    # 5. Invia il comando: VAI!
    navigator.goToPose(goal_pose)

    # 6. Monitora il viaggio
    while not navigator.isTaskComplete():
        # Feedback ogni tanto
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distanza mancante: {feedback.distance_remaining:.2f} metri')
        
        # Non intasare la CPU
        time.sleep(1.0)

    # 7. Risultato finale
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Obiettivo raggiunto con successo! \U0001f389')
    elif result == TaskResult.CANCELED:
        print('Navigazione cancellata. \U0001f6d1')
    elif result == TaskResult.FAILED:
        print('Navigazione fallita! \u2620\ufe0f')
    else:
        print('Risultato sconosciuto.')

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
