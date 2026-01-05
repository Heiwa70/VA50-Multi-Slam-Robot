#!/usr/bin/env python3
#Script pour test la fusion de map à la main sans les pb de nav22
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class ArrowTeleop(Node):
    def __init__(self, robot_name):
        super().__init__(f'arrow_teleop_{robot_name}')
        self.publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        
        self.linear_speed = 0.12
        self.angular_speed = 0.3
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        print(f"\n{'='*50}")
        print(f"🎮 Contrôle {robot_name} avec les FLÈCHES DIRECTIONNELLES")
        print(f"{'='*50}")
        print("   ↑  : Avancer")
        print("   ↓  : Reculer")
        print("   ←  : Tourner à gauche")
        print("   →  : Tourner à droite")
        print("ESPACE : Arrêt")
        print("   q  : Quitter")
        print(f"{'='*50}\n")
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                if key == '\x1b':
                    key = sys.stdin.read(2)
                    
                    if key == '[A':
                        twist.linear.x = self.linear_speed
                        print("↑ Avancer")
                    elif key == '[B':
                        twist.linear.x = -self.linear_speed
                        print("↓ Reculer")
                    elif key == '[D':
                        twist.angular.z = self.angular_speed
                        print("��� Tourner gauche")
                    elif key == '[C':
                        twist.angular.z = -self.angular_speed
                        print("→ Tourner droite")
                
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print("⏹ STOP")
                
                elif key == 'q':
                    print("\n👋 Arrêt du contrôle")
                    break
                
                self.publisher.publish(twist)
                
        except Exception as e:
            print(f"\n❌ Erreur: {e}")
        
        finally:
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 teleop_arrows.py <robot_name>")
        print("Exemple: python3 teleop_arrows.py TB3_1")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    
    rclpy.init()
    teleop = ArrowTeleop(robot_name)
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
