#!/usr/bin/env python3
import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    bp_lib = world.get_blueprint_library()
    actors = []

    try:
        # --- Vehicle ---
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('ros_name', 'hero')

        vehicle = world.spawn_actor(vehicle_bp, world.get_map().get_spawn_points()[0])
        actors.append(vehicle)
        world.tick()

        # --- Camera ---
        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '640')
        camera_bp.set_attribute('image_size_y', '480')
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('role_name', 'rgb_front')
        camera_bp.set_attribute('ros_name', 'rgb_front')

        camera = world.spawn_actor(camera_bp,
                                   carla.Transform(carla.Location(x=1.5, z=2.4)),
                                   attach_to=vehicle)
        actors.append(camera)
        camera.enable_for_ros()

        # --- LiDAR ---
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('range', '50')
        lidar_bp.set_attribute('role_name', 'lidar')
        lidar_bp.set_attribute('ros_name', 'lidar')

        lidar = world.spawn_actor(lidar_bp,
                                  carla.Transform(carla.Location(x=0, z=2.5)),
                                  attach_to=vehicle)
        actors.append(lidar)
        lidar.enable_for_ros()

        # --- Spectator follow cam ---
        spectator = world.get_spectator()

        print("Vehicle spawned. Ready for keyboard control.")
        print("Topics: /carla/hero/rgb_front/image, /carla/hero/lidar/point_cloud")

        while True:
            world.tick()
            transform = vehicle.get_transform()
            fwd = transform.get_forward_vector()
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(
                    x=fwd.x * -6,
                    y=fwd.y * -6,
                    z=3
                ),
                carla.Rotation(pitch=-10, yaw=transform.rotation.yaw)
            ))

    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        world.apply_settings(original_settings)
        for actor in reversed(actors):
            actor.destroy()

if __name__ == '__main__':
    main()
