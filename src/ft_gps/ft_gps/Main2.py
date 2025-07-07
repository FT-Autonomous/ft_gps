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
