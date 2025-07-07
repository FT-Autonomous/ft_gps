import rclpy
from rclpy.node import Node
import serial
import math
from collections import deque
from std_msgs.msg import String 
import datetime
import matplotlib.pyplot as plot

# Centerline logic
class Centerline:
    def __init__(self):
        self.segments = []  # List of tuples: ("circle", cx, cy, r) or ("line", (x1, y1), (x2, y2))

    def add_circle(self, cx, cy, r):
        self.segments.append(("circle", cx, cy, r))

    def add_line(self, x1, y1, x2, y2):
        self.segments.append(("line", (x1, y1), (x2, y2)))

    def check_proximity(self, x, y, threshold=1.0):
        for segment in self.segments:
            if segment[0] == "circle":
                _, cx, cy, r = segment
                dist_to_center = math.hypot(x - cx, y - cy)
                dist_from_path = abs(dist_to_center - r)
                if dist_from_path <= threshold:
                    return True  # Within allowed distance
            elif segment[0] == "line":
                _, (x1, y1), (x2, y2) = segment
                dist_from_path = self._distance_to_line(x, y, x1, y1, x2, y2)
                if dist_from_path <= threshold:
                    return True  # Within allowed distance
        return False  # Too far from all segments

    def _distance_to_line(self, px, py, x1, y1, x2, y2):
        """Shortest distance from point (px, py) to line segment (x1, y1)-(x2, y2)"""
        dx = x2 - x1
        dy = y2 - y1
        if dx == dy == 0:
            return math.hypot(px - x1, py - y1)  # line is a point
        t = ((px - x1) * dx + (py - y1) * dy) / (dx ** 2 + dy ** 2)
        t = max(0, min(1, t))  # clamp to segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return math.hypot(px - closest_x, py - closest_y)


def dm_to_dd(dm, direction):
    if not dm:
        return None
    degrees = int(dm[:len(dm) - 6])
    minutes = float(dm[len(dm) - 6:])
    dd = degrees + (minutes / 60)
    if direction in ['S', 'W']:
        dd *= -1
    return dd


def gps_to_local(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth radius in meters
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)
    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad
    x = dlon * R * math.cos((lat_rad + origin_lat_rad) / 2)
    y = dlat * R
    return x, y


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        # loggers
        self.info_pub = self.create_publisher(String, 'gps_info', 100)
        self.warn_pub = self.create_publisher(String, 'gps_warn', 100)
        self.error_pub = self.create_publisher(String, 'gps_error', 100)

        self.publish_info('GPS Node started')

        self.PORT = "/dev/ttyACM0"
        self.BAUD = 9600
        self.origin_lat = None
        self.origin_lon = None
        self.x_buffer = deque(maxlen=10)
        self.y_buffer = deque(maxlen=10)
        self.track = Centerline()
        self.setup_cones()
        self.setup_loop_zones()
        self.start_time = datetime.datetime.now().isoformat()

        try:
            #self.ser = serial.Serial(self.PORT, self.BAUD, timeout=1)
            self.timer = self.create_timer(0.1, self.read_gps_data) # loop 
        except Exception as e:
            self.publish_error(f"Serial init failed: {e}")
            raise
    
    def setup_cones(self):
        self.enter_left_cones = []
        self.enter_right_cones = []
        self.middle_cones = []
        self.exit_left_cones = []
        self.exit_right_cones = []
        self.lhs_cones = [] 
        self.rhs_cones = [] 
    
    def setup_loop_zones(self):
        self.loop_zones = {
            "start": {"x": 0, "y": 0, "radius": 0.2, "entered": False, "exited": False},
            "middle_point": {"x": 0, "y": 11, "radius": 0.2, "entry_count": 0, "in_zone": False},
            "right_loop": {"x": 15, "y": 11.5, "radius": 0.2, "entry_count": 0, "in_zone": False},
            "left_loop": {"x": -15, "y": 11.5, "radius": 0.2, "entry_count": 0, "in_zone": False},
            "exit": {"x": 0, "y": 22, "radius": 0.2, "entered": False, "exited": False}
        }

    def publish_info(self, msg: str):
        self.get_logger().info(msg)
        self.info_pub.publish(String(data=msg))

    def publish_warn(self, msg: str):
        self.get_logger().warn(msg)
        self.warn_pub.publish(String(data=msg))

    def publish_error(self, msg: str):
        self.get_logger().error(msg)
        self.error_pub.publish(String(data=msg))


    def check_cone_proximity(self, x, y, cones, label, near_thresh=0.5, off_thresh=0.2):
        for cone_x, cone_y in cones:
            distance = math.hypot(x - cone_x, y - cone_y)
            if distance < off_thresh:
                self.publish_warn(f"VERY CLOSE to {label} cone at ({cone_x:.2f}, {cone_y:.2f}) — possible collision!")
            elif distance < near_thresh:
                self.publish_warn(f"Near {label} cone at ({cone_x:.2f}, {cone_y:.2f})")


    def check_loop_events(self, x, y, zones):
        def in_radius(px, py, zx, zy, r):
            return math.hypot(px - zx, py - zy) <= r

        # If near the start, then say started skidpad
        if in_radius(x, y, zones["start"]["x"], zones["start"]["y"], zones["start"]["radius"]):
            if not zones["start"]["entered"]:
                zones["start"]["entered"] = True
        elif zones["start"]["entered"] and not zones["start"]["exited"]:
            zones["start"]["exited"] = True
            self.publish_info("Starting SKIDPAD")

        # Loop counter
        mp = zones["middle_point"]
        if in_radius(x, y, mp["x"], mp["y"], mp["radius"]):
            if not mp["in_zone"]:
                mp["entry_count"] += 1
                mp["in_zone"] = True
                if mp["entry_count"] == 1:
                    self.publish_info(" Starting right loop 1")
                elif mp["entry_count"] == 2:
                    self.publish_info(" Finished right loop 1, starting right loop 2")
                elif mp["entry_count"] == 3:
                    self.publish_info(" Starting left loop 1")
                elif mp["entry_count"] == 4:
                    self.publish_info(" Finished left loop 1, starting left loop 2")
                elif mp["entry_count"] == 5:
                    self.publish_info(" Loops completed, exiting")
        else:
            mp["in_zone"] = False

        # Right loop logic
        rl = zones["right_loop"]
        if in_radius(x, y, rl["x"], rl["y"], rl["radius"]):
            if not rl["in_zone"]:
                rl["entry_count"] += 1
                rl["in_zone"] = True
                self.publish_info(f" Right loop {rl['entry_count']} in progress")
        else:
            rl["in_zone"] = False

        # Left loop logic
        ll = zones["left_loop"]
        if in_radius(x, y, ll["x"], ll["y"], ll["radius"]):
            if not ll["in_zone"]:
                ll["entry_count"] += 1
                ll["in_zone"] = True
                self.publish_info(f" Left loop {ll['entry_count']} in progress")
        else:
            ll["in_zone"] = False

        # Exit logic
        if in_radius(x, y, zones["exit"]["x"], zones["exit"]["y"], zones["exit"]["radius"]):
            if not zones["exit"]["entered"]:
                zones["exit"]["entered"] = True
        elif zones["exit"]["entered"] and not zones["exit"]["exited"]:
            zones["exit"]["exited"] = True
            self.publish_info("SKIDPAD complete")


    def check_cone_hits(self, x, y, hit_radius=0.3):
        all_cone_lists = {
            "ENTER_LEFT": self.enter_left_cones,
            "ENTER_RIGHT": self.enter_right_cones,
            "LHS": self.lhs_cones,
            "RHS": self.rhs_cones,
            "MIDDLE": self.middle_cones,
            "EXIT_LEFT": self.exit_left_cones,
            "EXIT_RIGHT": self.exit_right_cones
        }

        hit_detected = False

        for label, cone_list in all_cone_lists.items():
            for i, (cx, cy) in enumerate(cone_list):
                distance = math.hypot(x - cx, y - cy)
                if distance <= hit_radius:
                    self.publish_warn(f"Cone hit: {label}{i + 1} at distance {distance:.2f} m")
                    hit_detected = True

        return hit_detected


    def print_all_cones(self):
        for i, (x, y) in enumerate(self.enter_left_cones, start=1):
            self.publish_info(f"ENTER_LEFT{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.enter_right_cones, start=1):
            self.publish_info(f"ENTER_RIGHT{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.lhs_cones, start=1):
            self.publish_info(f"LHS{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.rhs_cones, start=1):
            self.publish_info(f"RHS{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.middle_cones, start=1):
            self.publish_info(f"MIDDLE{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.exit_left_cones, start=1):
            self.publish_info(f"EXIT_LEFT{i}: x = {x:.2f} m, y = {y:.2f} m")
        for i, (x, y) in enumerate(self.exit_right_cones, start=1):
            self.publish_info(f"EXIT_RIGHT{i}: x = {x:.2f} m, y = {y:.2f} m")

    def draw_track_and_points(self, gps_filename="gps_data_log.txt"):
        fig, ax = plot.subplots(figsize=(10, 10))
        
        # Plot all cones
        def plot_cones(cones, color, label):
            xs, ys = zip(*cones) if cones else ([], [])
            ax.scatter(xs, ys, c=color, label=label, s=50)

        plot_cones(self.enter_left_cones, "blue", "Enter Left")
        plot_cones(self.enter_right_cones, "red", "Enter Right")
        plot_cones(self.lhs_cones, "green", "LHS")
        plot_cones(self.rhs_cones, "orange", "RHS")
        plot_cones(self.middle_cones, "purple", "Middle")
        plot_cones(self.exit_left_cones, "cyan", "Exit Left")
        plot_cones(self.exit_right_cones, "magenta", "Exit Right")

        # Plot GPS path from file
        gps_x = []
        gps_y = []
        try:
            with open(gps_filename, "r") as file:
                for line in file:
                    line = line.strip()
                    if line.startswith("$GNGGA"):
                        parts = line.split(",")
                        if len(parts) > 5 and parts[6] != '0':
                            lat_dm = parts[2]
                            lat_dir = parts[3]
                            lon_dm = parts[4]
                            lon_dir = parts[5]
                            
                            lat = dm_to_dd(lat_dm, lat_dir)
                            lon = dm_to_dd(lon_dm, lon_dir)

                            if self.origin_lat is None or self.origin_lon is None:
                                continue  # Skip until origin is known

                            x, y = gps_to_local(lat, lon, self.origin_lat, self.origin_lon)
                            gps_x.append(x)
                            gps_y.append(y)

            if gps_x and gps_y:
                ax.plot(gps_x, gps_y, color="black", linewidth=2, label="GPS Path")
        except Exception as e:
            self.publish_warn(f"Error while drawing: {e}")
        
        # Optional: draw centerline circles/lines
        if hasattr(self, 'track') and hasattr(self.track, 'segments'):
            for seg in self.track.segments:
                if seg["type"] == "circle":
                    circle = plot.Circle((seg["cx"], seg["cy"]), seg["r"], color='gray', fill=False, linestyle='--')
                    ax.add_patch(circle)
                elif seg["type"] == "line":
                    ax.plot([seg["x1"], seg["x2"]], [seg["y1"], seg["y2"]], color='gray', linestyle='--')

        ax.set_aspect('equal')
        ax.set_title("Track and GPS Data")
        ax.legend()
        ax.grid(True)
        plot.xlabel("Local X (meters)")
        plot.ylabel("Local Y (meters)")
        plot.show()
        
    def read_gps_data(self):
        self.draw_track_and_points()
        try:
            with open("gps_log.txt", "r") as file:
                for line in file:
                    line = line.strip()
            #line = self.ser.readline().decode(errors='ignore').strip()
            #with open(f"gps_log_{self.start_time}", "a") as f:
            #    f.write(line)
            #    f.write("\n")
                    if line.startswith("$GNGGA"):
                        parts = line.split(",")
                        if len(parts) > 5 and parts[6] != '0':
                            lat_dm = parts[2]
                            lat_dir = parts[3]
                            lon_dm = parts[4]
                            lon_dir = parts[5]

                            lat = dm_to_dd(lat_dm, lat_dir)
                            lon = dm_to_dd(lon_dm, lon_dir)

                            if self.origin_lat is None:
                                self.origin_lat = lat
                                self.origin_lon = lon

                                # Create track
                                self.track.add_circle(cx=7.5, cy=11.5, r=16.75)
                                self.track.add_circle(cx=-7.5, cy=11.5, r=16.75)
                                self.track.add_line(0, 0, 0, 24)
                                self.publish_info("Centerline set.")

                                # ENTER LEFT
                                self.enter_left_cones.extend([
                                    (-1.5, .2), (-1.5, 2.7), (-1.8, 4.9)
                                ])

                                # ENTER RIGHT
                                self.enter_right_cones.extend([
                                    (1.5, .2), (1.5, 2.6), (1.7, 5)
                                ])

                                # LHS CONES
                                self.lhs_cones.extend([
                                    (-1.9, 9.3), (-1.5, 11.5), (-1.8, 13.5), (-1.9, 14.0), (1.7, 18),
                                    (4, 19.3), (7.5, 20), (11, 19.4), (13.8, 17.5), (15.8, 14.8),
                                    (16.5, 11.6), (15.8, 8.4), (14, 5.6), (11, 3.7), (7.6, 3),
                                    (4.2, 3.6), (8.4, 2.9), (-3.1, 15.4), (-5.2, 16.6), (-7.5, 17),
                                    (-9.7, 16.6), (-12.9, 13.5), (-13.5, 11.5), (-13, 9.4), (-11.7, 7.5),
                                    (-9.7, 6.2), (-7.5, 5.8), (-5.2, 6.3), (-3.4, 7.5)
                                ])

                                # MIDDLE CONE
                                self.middle_cones.append((0, 16))

                                # RHS cones
                                self.rhs_cones.extend([
                                    (3.2, 7.5), (1.9, 9.3), (1.5, 11.5), (2, 13.5), (3.3, 15.5),
                                    (5.2, 16.8), (7.5, 17), (9.8, 16.6), (11.7, 15.5), (12.9, 16.6),
                                    (13, 13.6), (15, 15.6), (13.5, 11.5), (13, 9.3), (11.7, 7.4),
                                    (9.8, 6.3), (7.5, 5.8), (5.2, 6.3), (-1.8, 18), (-4, 19.3),
                                    (-7.4, 20), (-11, 19.3), (-13.8, 17.6), (-15.7, 14.8), (-16.4, 11.6),
                                    (-15.8, 8.4), (-13.9, 5.6), (-11, 3.7), (-7.6, 3), (-4.1, 3.6)
                                ])

                                # EXIT LEFT
                                self.exit_left_cones.extend([
                                    (-1.8, 18), (-1.5, 20.1), (-1.5, 22.8)
                                ])

                                # EXIT RIGHT
                                self.exit_right_cones.extend([
                                    (1.7, 18), (1.5, 20.1), (1.5, 22.8)
                                ])
                                self.publish_info("Local cones set")

                            else:
                                x, y = gps_to_local(lat, lon, self.origin_lat, self.origin_lon)
                                self.x_buffer.append(x)
                                self.y_buffer.append(y)
                                self.publish_info("Loading absolute & local coordinates...")

                                if len(self.x_buffer) == 10:
                                    avg_x = sum(self.x_buffer) / len(self.x_buffer)
                                    avg_y = sum(self.y_buffer) / len(self.y_buffer)
                                    self.publish_info(f"Absolute coordinates: lat. = {lat:.8f}, long. = {lon:.8f}")
                                    self.publish_info(f"Local coordinates: x = {avg_x:.2f}, y = {avg_y:.2f}\n")

                                    # Cone hit check
                                    self.check_cone_hits(avg_x, avg_y)

                                    # 🔍 Cone proximity checks
                                    self.check_cone_proximity(avg_x, avg_y, self.enter_left_cones, "ENTER_LEFT")
                                    self.check_cone_proximity(avg_x, avg_y, self.enter_right_cones, "ENTER_RIGHT")
                                    self.check_cone_proximity(avg_x, avg_y, self.lhs_cones, "LHS")
                                    self.check_cone_proximity(avg_x, avg_y, self.rhs_cones, "RHS")
                                    self.check_cone_proximity(avg_x, avg_y, self.middle_cones, "MIDDLE")
                                    self.check_cone_proximity(avg_x, avg_y, self.exit_left_cones, "EXIT_LEFT")
                                    self.check_cone_proximity(avg_x, avg_y, self.exit_right_cones, "EXIT_RIGHT")

                                    if not self.track.check_proximity(avg_x, avg_y):
                                        self.publish_warn("⚠️  Warning: more than 1 meter off centerline!")

                                    #  Check for loop events
                                    self.check_loop_events(avg_x, avg_y, self.loop_zones)
                    #else:
                    #    self.publish_info("Finding satellites... no GPS fix yet.")
        except KeyboardInterrupt:
            self.publish_info("\nStopped by user.")
            # change all `print(...)` to `self.publish_info(...)` or `.warn(...)` or `.error(...)`
        except Exception as e:
            self.publish_error(f"GPS read error: {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
