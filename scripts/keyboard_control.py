import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.twist = Twist()
        self.speed = 0.5  # Linear speed (m/s)
        self.turn = 1.5   # Angular speed (rad/s)

        # Initialize pygame
        pygame.init()

    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    self.on_key_down(event.key)
                elif event.type == pygame.KEYUP:
                    self.on_key_up(event.key)
                elif event.type == pygame.QUIT:
                    running = False

            # Publish the current twist
            self.publisher_.publish(self.twist)

            # Control the frequency of the loop
            clock.tick(30)  # 30 FPS

        pygame.quit()
        self.destroy_node()
        rclpy.shutdown()

    def on_key_down(self, key):
        if key == pygame.K_w:  # Forward
            self.twist.linear.x = self.speed
            self.twist.angular.z = 0.0
        elif key == pygame.K_s:  # Backward
            self.twist.linear.x = -self.speed
            self.twist.angular.z = 0.0
        elif key == pygame.K_a:  # Turn left
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.turn
        elif key == pygame.K_d:  # Turn right
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.turn

    def on_key_up(self, key):
        if key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d]:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    keyboard_control.run()

if __name__ == '__main__':
    main()
