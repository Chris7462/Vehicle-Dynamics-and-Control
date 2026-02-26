#!/usr/bin/env python3
"""
Keyboard control for CARLA 0.9.16 native ROS2.
Publishes CarlaEgoVehicleControl to /carla/hero/vehicle_control_cmd

Controls:
  Arrow Up    - Throttle
  Arrow Down  - Brake
  Arrow Left  - Steer left
  Arrow Right - Steer right
  Space       - Handbrake
  R           - Toggle reverse
  P           - Toggle autopilot
  Q / ESC     - Quit

Requirements:
  pip install pygame
  source ~/ros2_ws/install/setup.bash
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from carla_msgs.msg import CarlaEgoVehicleControl

import carla
import pygame
import sys


# --- Tuning ---
THROTTLE_STEP   = 0.1   # how fast throttle builds up per frame
THROTTLE_DECAY  = 0.05  # how fast throttle drops when key released
STEER_STEP      = 0.05  # how fast steering builds up per frame
STEER_DECAY     = 0.1   # how fast steering returns to center
BRAKE_VALUE     = 0.8   # fixed brake amount when braking


class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('carla_keyboard_control')

        # QoS must match CARLA's subscriber: BEST_EFFORT + VOLATILE
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/hero/vehicle_control_cmd',
            qos
        )

        # Connect to CARLA for autopilot toggle
        self.vehicle = None
        self._connect_to_carla()

        # Control state
        self.throttle  = 0.0
        self.steer     = 0.0
        self.brake     = 0.0
        self.reverse   = False
        self.handbrake = False
        self.autopilot = False

        # Pygame setup
        pygame.init()
        self.screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption('CARLA Keyboard Control')
        self.font = pygame.font.SysFont('monospace', 16)
        self.clock = pygame.time.Clock()

        # Timer: publish at 20 Hz
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Keyboard control node started.')

    def _connect_to_carla(self):
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(5.0)
            world = client.get_world()
            actors = world.get_actors().filter('vehicle.*')
            for actor in actors:
                if actor.attributes.get('ros_name') == 'hero':
                    self.vehicle = actor
                    self.get_logger().info(f'Found hero vehicle id={actor.id}')
                    return
            self.get_logger().warn('Hero vehicle not found — autopilot toggle disabled')
        except Exception as e:
            self.get_logger().warn(f'Could not connect to CARLA: {e} — autopilot toggle disabled')

    def _toggle_autopilot(self):
        if self.vehicle is None:
            self.get_logger().warn('No vehicle found for autopilot toggle')
            return
        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot)
        self.get_logger().info(f'Autopilot: {"ON" if self.autopilot else "OFF"}')

    def control_loop(self):
        # --- Process pygame events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self._shutdown()
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self._shutdown()
                elif event.key == pygame.K_r:
                    self.reverse = not self.reverse
                    self.get_logger().info(f'Reverse: {"ON" if self.reverse else "OFF"}')
                elif event.key == pygame.K_p:
                    self._toggle_autopilot()

        # --- Read held keys ---
        keys = pygame.key.get_pressed()

        # Throttle
        if keys[pygame.K_UP]:
            self.throttle = min(1.0, self.throttle + THROTTLE_STEP)
            self.brake = 0.0
        else:
            self.throttle = max(0.0, self.throttle - THROTTLE_DECAY)

        # Brake
        if keys[pygame.K_DOWN]:
            self.brake = BRAKE_VALUE
            self.throttle = 0.0
        else:
            self.brake = 0.0

        # Steering
        if keys[pygame.K_LEFT]:
            self.steer = max(-1.0, self.steer - STEER_STEP)
        elif keys[pygame.K_RIGHT]:
            self.steer = min(1.0, self.steer + STEER_STEP)
        else:
            # Return to center
            if self.steer > 0:
                self.steer = max(0.0, self.steer - STEER_DECAY)
            elif self.steer < 0:
                self.steer = min(0.0, self.steer + STEER_DECAY)

        # Handbrake
        self.handbrake = bool(keys[pygame.K_SPACE])

        # --- Publish ---
        if not self.autopilot:
            msg = CarlaEgoVehicleControl()
            msg.throttle          = float(self.throttle)
            msg.steer             = float(self.steer)
            msg.brake             = float(self.brake)
            msg.hand_brake        = self.handbrake
            msg.reverse           = self.reverse
            msg.manual_gear_shift = False
            msg.gear              = 1
            self.pub.publish(msg)

        # --- Draw HUD ---
        self.screen.fill((30, 30, 30))
        lines = [
            'CARLA Keyboard Control',
            '',
            f'Throttle : {self.throttle:.2f}',
            f'Brake    : {self.brake:.2f}',
            f'Steer    : {self.steer:.2f}',
            f'Reverse  : {"ON" if self.reverse else "off"}',
            f'Handbrake: {"ON" if self.handbrake else "off"}',
            f'Autopilot: {"ON" if self.autopilot else "off"}',
            '',
            'P=autopilot  R=reverse  Q=quit',
        ]
        for i, line in enumerate(lines):
            color = (0, 255, 100) if i == 0 else (200, 200, 200)
            surf = self.font.render(line, True, color)
            self.screen.blit(surf, (20, 15 + i * 18))
        pygame.display.flip()
        self.clock.tick(60)

    def _shutdown(self):
        self.get_logger().info('Shutting down.')
        # Send a stop command before quitting
        msg = CarlaEgoVehicleControl()
        msg.throttle = 0.0
        msg.brake    = 1.0
        self.pub.publish(msg)
        pygame.quit()
        rclpy.shutdown()
        sys.exit(0)


def main():
    rclpy.init()
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown()


if __name__ == '__main__':
    main()
