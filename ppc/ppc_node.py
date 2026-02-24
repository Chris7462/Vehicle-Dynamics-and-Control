#!/usr/bin/env python3
import csv, math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

# Program and Controller Inputs
# ADD THIS
CSV_FILENAME = '/workspace/src/waypoint/waypoints.csv' #from previous lab
LOG_FILENAME = '/workspace/src/ppc/ppc_vehicle_log.csv' #this code will generate this
SPEED = 5.0     # constant forward speed [m/s]
LOOKAHEAD = 6.0 # lookahead distance [m]
WHEELBASE = 2.7 # Nissan Leaf wheelbase [m]
DT = 0.03       # control loop preiod [s] (~33 Hz)

MAX_STEER_DEG = 70.0     # deg (steering saturation limit)


# This function loads the waypoints collected in the previous lab
def load_waypoints(csv_path):
    wpts = []
    with open(csv_path) as f:
        rdr = csv.reader(f)
        next(rdr, None)  # skip header row (column labels)
        for row in rdr:
            if not row:
                continue
            # Assumes x is column 1 and y is column 2 in the CSV
            x = float(row[1])
            y = float(row[2])
            wpts.append((x, y))
    return wpts


# This is the running Pure Pursuit Controller inside ROS
class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        # Load waypoint list once at startup of this ROS node
        self.waypoints = load_waypoints(CSV_FILENAME)
        self.log = []    # Store vehicle trajectory for later plotting

        # Initialize vehicle state (will be updated by odometry callback)
        self.x = None
        self.y = None
        self.heading = 0.0

        # Extra part to ensure the controller works well
        # keeps track of progress along the waypoint list
        self.last_idx = 0  # do not go backward

        # Subscribe to vehicle position data, publish vehicle commands
        self.create_subscription(Odometry,"/carla/hero/odometry",self.odom_cb,10)
        self.pub = self.create_publisher(AckermannDrive,"/carla/hero/ackermann_cmd",10)

        # Create a physcially timed control loop
        self.create_timer(DT, self.loop)


    # Odometry callback: runs every time ROS publishes a new odometry message
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position #get position
        self.x = float(p.x)
        self.y = float(p.y)

        o = msg.pose.pose.orientation  #heading (have to convert from quaternion)
        self.heading = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                            1.0 - 2.0 * (o.y * o.y + o.z * o.z))


    # Main control loop (runs about 33 times per second)
    def loop(self):
        if self.x is None: # Wait until we have received odometry
            return

        # (1) Choose a lookahead target from the path
        target_id_index = self.select_target_index(LOOKAHEAD)
        target_x, target_y = self.waypoints[target_id_index]

        # (2) Compute Pure Pursuit steering command
        steer = self.compute_steer(target_x, target_y)

        # (3) Send command to the vehicle
        self.publish_cmd(steer, SPEED)

        # Log time, position, and speed
        now = self.get_clock().now().nanoseconds * 1e-9
        self.log.append([now, self.x, self.y])  #you can add other variables


    # Determine which waypoint the car should aim toward
    def select_target_index(self, Ld):
        # Vehicle heading unit vector
        hx = math.cos(self.heading)
        hy = math.sin(self.heading)
        target_i = len(self.waypoints) - 1

        # Search forward from last_idx only
        for i in range(self.last_idx, len(self.waypoints)):
            wx, wy = self.waypoints[i]

            # Must be in front of the vehicle
            ahead = ((wx - self.x)*hx + (wy - self.y)*hy) > 0.0

            # Must be at least lookahead distance away
# ADD THIS
            far_enough = ...

            if ahead and far_enough:
                target_i = i
                break

        # Commit progress along the path
        self.last_idx = target_i
        return target_i


    # Pure Pursuit steering law (use calculated/actual Ld, called Ld_eff)
    def compute_steer(self, tx, ty):
# ADD THIS
        ...
        ...

        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi # wrap alpha to [-pi, pi]

        max_steer = math.radians(MAX_STEER_DEG) # enforce limits on steering
        if steer > max_steer:
            steer = max_steer
        if steer < -max_steer:
            steer = -max_steer
        return steer

    #Drive vehicle with steering command
    def publish_cmd(self, steer, speed):
        msg = AckermannDrive()
        msg.steering_angle = float(steer)
        msg.speed = float(speed)
        self.pub.publish(msg)

    # Write the CSV when the node shuts down
    def write_log(self):
        if not self.log:
            return

        with open(LOG_FILENAME, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "x", "y", "speed"])
            w.writerows(self.log)


# ROS entry point: starts ROS, creates controller node, runs it indefinitely, stops node with Ctrl+C
def main(argv=None):
    rclpy.init(args=argv)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_log()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
