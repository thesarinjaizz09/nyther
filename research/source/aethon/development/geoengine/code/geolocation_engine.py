import numpy as np
from pyproj import CRS, Transformer
from math import sin, cos, tan, radians, degrees

class GeolocationEngine:
    """
    A core module for the Aethon UAV to calculate the geodetic position of a 
    detected object using Direct Georeferencing principles.
    """

    def __init__(self, camera_intrinsics, camera_to_body_rpy):
        """
        Initializes the engine with fixed camera and mounting parameters.

        :param camera_intrinsics: Tuple (fx, fy, cx, cy)
        :param camera_to_body_rpy: Tuple (roll, pitch, yaw) offset from body to camera frame (in radians).
        """
        self.fx, self.fy, self.cx, self.cy = camera_intrinsics
        self.R_cam_to_body = self._rpy_to_rotation_matrix(*camera_to_body_rpy)
        
        # Transformer for NED (Local Tangent Plane) to WGS84 (Geodetic)
        self.crs_wgs84 = CRS.from_epsg(4326) # WGS84
        self.crs_ned = CRS.from_proj4("+proj=geocent +ellps=WGS84 +units=m") # Placeholder, will be initialized per calculation

    def _rpy_to_rotation_matrix(self, roll, pitch, yaw):
        """
        Converts Roll, Pitch, Yaw (ZYX convention) to a 3x3 rotation matrix.
        """
        R_x = np.array([[1, 0, 0],
                        [0, cos(roll), -sin(roll)],
                        [0, sin(roll), cos(roll)]])
        
        R_y = np.array([[cos(pitch), 0, sin(pitch)],
                        [0, 1, 0],
                        [-sin(pitch), 0, cos(pitch)]])
        
        R_z = np.array([[cos(yaw), -sin(yaw), 0],
                        [sin(yaw), cos(yaw), 0],
                        [0, 0, 1]])
        
        # R = R_z @ R_y @ R_x (Standard aerospace convention for NED to Body)
        return R_z @ R_y @ R_x

    def _pixel_to_camera_frame(self, u, v):
        """
        Converts pixel coordinates to a normalized vector in the camera frame.
        """
        x_c = (u - self.cx) / self.fx
        y_c = (v - self.cy) / self.fy
        # The vector is [x_c, y_c, 1] where 1 is the focal length
        return np.array([x_c, y_c, 1.0])

    def _camera_to_ned_ray(self, uav_rpy, gimbal_rpy, pixel_coords):
        """
        Transforms the camera ray vector to the NED frame.
        """
        roll_uav, pitch_uav, yaw_uav = uav_rpy
        roll_gimbal, pitch_gimbal, yaw_gimbal = gimbal_rpy
        u, v = pixel_coords

        # 1. Pixel to Camera Frame
        v_cam = self._pixel_to_camera_frame(u, v)

        # 2. Camera to Gimbal Frame (Assuming gimbal RPY is relative to body)
        # In a real system, the gimbal RPY is relative to the body or a fixed frame.
        # For simplicity, we assume the gimbal RPY is the rotation from the body frame
        # to the camera frame, overriding the fixed R_cam_to_body.
        # For a stabilized gimbal, the gimbal RPY is often the rotation from the NED frame
        # to the camera frame, which simplifies the math significantly.
        # We will use the simplified, stabilized gimbal model:
        
        # R_gimbal_to_NED is the rotation from the stabilized gimbal frame to NED.
        # Since the gimbal is stabilized, its RPY is often the actual camera attitude in NED.
        # However, for a typical setup, the gimbal angles are relative to the body.
        
        # Let's stick to the full chain: Camera -> Body -> NED
        
        # R_body_to_NED (UAV attitude)
        R_body_to_NED = self._rpy_to_rotation_matrix(roll_uav, pitch_uav, yaw_uav)

        # R_cam_to_body (Gimbal attitude relative to body)
        # This is the combined rotation of the fixed mount and the gimbal motors
        R_gimbal_relative = self._rpy_to_rotation_matrix(roll_gimbal, pitch_gimbal, yaw_gimbal)
        R_cam_to_body_total = self.R_cam_to_body @ R_gimbal_relative # Fixed mount @ Gimbal movement

        # 3. Transform to NED
        v_ned = R_body_to_NED @ R_cam_to_body_total @ v_cam
        
        # Normalize the vector
        v_ned = v_ned / np.linalg.norm(v_ned)
        
        return v_ned

    def _ned_to_wgs84(self, uav_lat_lon_alt, ned_coords, target_alt_m):
        """
        Converts NED coordinates (relative to UAV) to WGS84 (Lat, Lon, Alt).
        """
        uav_lat, uav_lon, uav_alt = uav_lat_lon_alt
        x_ned, y_ned, z_ned = ned_coords

        # Initialize transformer for the specific local tangent plane (NED)
        # Initialize transformer for the specific local tangent plane (NED)
        # We use the Geodetic to ENU transformer, which is more standard for local frames.
        # pyproj's ENU is East-North-Up. NED is North-East-Down.
        transformer = Transformer.from_crs(
            CRS.from_proj4(f"+proj=geocent +ellps=WGS84 +units=m"),
            self.crs_wgs84,
            always_xy=True
        )
        
        # We need to convert NED (x_ned, y_ned, z_ned) to ECEF (Earth-Centered, Earth-Fixed)
        # The pyproj library has a dedicated function for this.
        
        # First, convert NED coordinates to ECEF coordinates
        # pyproj's `transform` function can handle the conversion from a local frame (like NED)
        # to a global frame (like WGS84) if the local frame is defined correctly.
        
        # Let's use the standard pyproj approach for local tangent plane (ENU)
        # ENU (East, North, Up) from NED (North, East, Down)
        x_enu = ned_coords[1] # East
        y_enu = ned_coords[0] # North
        z_enu = -ned_coords[2] # Up is negative Down
        
        # The standard way to transform from a local tangent plane (like NED/ENU)
        # to WGS84 is to use the `from_crs` method with the `+proj=tmerc` definition
        # centered at the UAV's position.
        
        # The transformer needs the origin of the local frame (UAV position)
        # We use the Geodetic to ENU transformer, which is more standard for local frames.
        # pyproj's ENU is East-North-Up. NED is North-East-Down.
        
        # We only need the 2D transformation for Lat/Lon, as Alt is determined by ray-casting.
        # The tmerc projection is a 2D projection, and the coordinates passed are Easting (x) and Northing (y).
        transformer = Transformer.from_crs(
            CRS.from_proj4(f"+proj=tmerc +lat_0={uav_lat} +lon_0={uav_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m"),
            self.crs_wgs84,
            always_xy=True
        )
        
        # Transform ENU (East, North) relative to the UAV to WGS84 (Lon, Lat)
        # pyproj's transform takes (x, y) where x=East, y=North
        # Note: pyproj's transform returns (longitude, latitude) when always_xy=True
        # The tmerc projection is a 2D projection, and the coordinates passed are Easting (x) and Northing (y).
        lon_target, lat_target = transformer.transform(
            x_enu, # East
            y_enu, # North
            direction="forward"
        )
        
        # The altitude returned by the transformer is relative to the ellipsoid.
        # Since the ray-tracing already accounted for the target's altitude (target_alt_m),
        # we should use that as the final altitude.
        # The transformer's Z-coordinate is the distance from the UAV's altitude.
        
        # For simplicity and to match the ray-tracing assumption, we will return the target_alt_m
        # as the final altitude, as the ray-tracing converged on that altitude.
        return lat_target, lon_target, target_alt_m
        
        return lat_target, lon_target, alt_target

    def _query_dem(self, lat, lon):
        """
        MOCK FUNCTION: In a real system, this would query a Digital Elevation Model (DEM)
        to get the terrain altitude at a given Lat/Lon.
        For this simulation, we assume a flat ground at 0 meters AGL.
        """
        # Replace this with a real DEM query (e.g., from a local file or a server)
        return 0.0 # Assuming 0 meters altitude for the ground

    def _ray_terrain_intersection(self, uav_pos_ned, v_ned, target_alt_m):
        """
        Calculates the intersection of the ray with the target altitude plane.
        
        :param uav_pos_ned: UAV position in NED (should be [0, 0, 0] as it's the origin)
        :param v_ned: Normalized ray vector in NED
        :param target_alt_m: The altitude of the target (e.g., 0 for ground).
        :return: Target position in NED coordinates [x, y, z]
        """
        # The ray equation is P(t) = P_UAV + t * v_NED
        # Since P_UAV is the origin of the NED frame, P_UAV = [0, 0, 0]
        # P(t) = t * v_NED
        
        # We are looking for the distance 't' where the Z-component of P(t) equals the target altitude.
        # The Z-axis in NED is Down (positive is down).
        # The target altitude is relative to the WGS84 ellipsoid.
        
        # For simplicity, we will use the flat-earth assumption where the target is at a fixed Z_NED.
        # Z_NED = t * v_NED[2]
        # The target is on the ground, so its Z_NED is the distance down from the UAV.
        
        # For a flat-earth model, the target's Z_NED is simply the UAV's altitude (H_AGL)
        # if the target is on the ground (Z_target = 0).
        
        # Let's use the more accurate approach: find the distance 't' where the ray intersects
        # the plane defined by the target's altitude (Z_target_NED).
        
        # Since the NED frame is centered at the UAV, the target's altitude in the NED frame
        # is not a simple value. We need to find the distance 't' where the ray hits the ground.
        
        # The simplest and most common approximation is to assume the target is at a fixed altitude
        # relative to the UAV's altitude.
        
        # Let's use the simple flat-earth model where the target is at Z_NED = H_AGL
        # H_AGL is the UAV's altitude above the target's altitude.
        
        # The ray vector v_NED is normalized, so v_NED[2] is the cosine of the angle between the ray and the Down axis.
        # The distance t is the hypotenuse of the right triangle.
        # t * v_NED[2] = H_AGL (Down distance)
        
        # Since we don't have H_AGL directly, we will use the UAV's altitude (Alt_UAV) and the target's altitude (Alt_Target).
        # H_AGL = Alt_UAV - Alt_Target
        
        # We need to find the distance 't' along the ray where the target's altitude is reached.
        # In the NED frame, the Z-axis is Down.
        # The target's Z-coordinate in NED is t * v_NED[2].
        
        # For the flat-earth model (target_alt_m is the ground altitude):
        # The target's Z-coordinate in NED is the distance from the UAV to the target in the Down direction.
        # This distance is (UAV_Alt - Target_Alt)
        
        # Let's assume the UAV's altitude is relative to the target's altitude for this simplified model.
        # The distance 't' is found by:
        
        if v_ned[2] <= 0:
            # Ray is pointing up or horizontal, cannot intersect ground (Down is positive Z)
            raise ValueError("Ray is pointing up or horizontal. Cannot intersect ground.")

        # t = (UAV_Alt - Target_Alt) / v_NED[2]
        # We will use a placeholder for the altitude difference, assuming the UAV is 100m AGL for now.
        # In the final function, we will use the actual UAV Alt and the DEM query.
        
        # For the final function, we will use the full UAV Alt and the DEM query.
        # Since we are in the NED frame, the UAV is at [0, 0, 0].
        # The target's Z-coordinate in NED is the distance down from the UAV.
        
        # Let's use the full model with a mock DEM query.
        
        # We need to find 't' such that the target's altitude (UAV_Alt - t * v_NED[2]) equals the DEM altitude.
        
        # Mock DEM-based Iterative Ray-Tracing (Binary Search for 't')
        
        # Initial guess for t (distance to target)
        t_min = 0.0
        t_max = 5000.0 # Max range of the camera
        
        # Mock UAV position (needed for the DEM query)
        uav_lat, uav_lon, uav_alt = self.uav_lat_lon_alt_global
        
        # The target's altitude in the NED frame is the distance down from the UAV's altitude.
        # The target's Z-coordinate in NED is t * v_NED[2].
        # We are looking for the distance 't' where the target's altitude is reached.
        
        # We will use the flat-earth model where the target is at a fixed altitude (target_alt_m).
        # This simplifies the ray-tracing to a single calculation.
        
        # The distance in the Down direction (Z_NED) from the UAV to the target is:
        Z_NED_target = uav_alt - target_alt_m
        
        # The ray equation for the Z-component is: Z_NED_target = t * v_NED[2]
        
        if v_ned[2] <= 0:
            # Ray is pointing up or horizontal, cannot intersect ground (Down is positive Z)
            raise ValueError("Ray is pointing up or horizontal. Cannot intersect ground.")

        # Solve for t (distance along the ray)
        t_final = Z_NED_target / v_ned[2]
        
        # Final target position in NED
        target_pos_ned = t_final * v_ned
        
        return target_pos_ned
                
        # Final distance 't' is t_mid
        t_final = (t_min + t_max) / 2.0
        
        # Final target position in NED
        target_pos_ned = t_final * v_ned
        
        return target_pos_ned

    def calculate_target_geoposition(self, pixel_coords, uav_lat_lon_alt, uav_rpy, gimbal_rpy, target_alt_m=0.0):
        """
        Main function to calculate the target's geodetic position.

        :param pixel_coords: Tuple (u, v) of the detected object.
        :param uav_lat_lon_alt: Tuple (lat, lon, alt) of the UAV (in degrees, meters).
        :param uav_rpy: Tuple (roll, pitch, yaw) of the UAV (in radians).
        :param gimbal_rpy: Tuple (roll, pitch, yaw) of the gimbal (in radians).
        :param target_alt_m: Assumed target altitude (in meters WGS84).
        :return: Tuple (lat, lon, alt) of the target (in degrees, meters).
        """
        self.uav_lat_lon_alt_global = uav_lat_lon_alt
        
        # 1. Transform ray to NED frame
        v_ned = self._camera_to_ned_ray(uav_rpy, gimbal_rpy, pixel_coords)

        # 2. Ray-Terrain Intersection (Mock DEM-based)
        try:
            target_pos_ned = self._ray_terrain_intersection(
                uav_pos_ned=np.array([0, 0, 0]), # UAV is the origin of NED
                v_ned=v_ned,
                target_alt_m=target_alt_m
            )
        except ValueError as e:
            print(f"Geolocation Error: {e}")
            return None

        # 3. Convert NED to WGS84
        target_lat, target_lon, target_alt = self._ned_to_wgs84(uav_lat_lon_alt, target_pos_ned, target_alt_m)

        return target_lat, target_lon, target_alt

# --- Example Usage and Simulation Setup ---
def simulate_geolocation():
    # 1. Define Camera Parameters (Example: 1080p camera, 50mm equivalent focal length)
    # Assume a sensor size of 6.4mm x 4.8mm (typical 1/2.3" sensor)
    # Resolution: 1920x1080
    # Focal length: 50mm (equivalent) -> actual focal length ~10mm
    
    # Let's use normalized focal lengths for simplicity in the intrinsic matrix
    # fx = fy = 1000 pixels (arbitrary for a 1920x1080 image)
    # cx = 1920 / 2 = 960
    # cy = 1080 / 2 = 540
    camera_intrinsics = (1000.0, 1000.0, 960.0, 540.0)
    
    # 2. Define Fixed Camera-to-Body Offset (in radians)
    # Assume the camera is mounted perfectly straight on the body
    camera_to_body_rpy = (radians(0), radians(0), radians(0))
    
    # Initialize the Engine
    engine = GeolocationEngine(camera_intrinsics, camera_to_body_rpy)

    # 3. Define UAV State (Example: Hovering over a location)
    # UAV Position: San Francisco, 100 meters above ground
    uav_lat_lon_alt = (37.7749, -122.4194, 100.0) # Lat, Lon, Alt (WGS84)
    
    # UAV Attitude (Level flight, facing North)
    uav_rpy = (radians(0), radians(0), radians(0)) # Roll, Pitch, Yaw (0, 0, 0)
    
    # Gimbal Attitude (Looking straight down)
    # Roll=0, Pitch=-90 degrees (down), Yaw=0 (relative to body)
    gimbal_rpy = (radians(0), radians(-90), radians(0)) 
    
    # 4. Define Detected Object (Pixel Coordinates)
    # Target is at the center of the image
    pixel_coords_center = (960, 540) 
    
    # Target is at the bottom right of the image
    pixel_coords_bottom_right = (1920, 1080)
    
    # 5. Target Altitude (Assumed flat ground)
    target_alt_m = 0.0 

    # --- Test Case 1: Center of Image (Straight Down) ---
    print("--- Test Case 1: Center of Image (Straight Down) ---")
    target_pos_center = engine.calculate_target_geoposition(
        pixel_coords_center, uav_lat_lon_alt, uav_rpy, gimbal_rpy, target_alt_m
    )
    if target_pos_center:
        print(f"UAV Position: Lat={uav_lat_lon_alt[0]:.6f}, Lon={uav_lat_lon_alt[1]:.6f}, Alt={uav_lat_lon_alt[2]:.2f}m")
        print(f"Target Position (Center): Lat={target_pos_center[0]:.6f}, Lon={target_pos_center[1]:.6f}, Alt={target_pos_center[2]:.2f}m")
        # Expected: Target should be directly below the UAV, at 0m altitude.

    # --- Test Case 2: Bottom Right of Image (Off-Center) ---
    print("\n--- Test Case 2: Bottom Right of Image (Off-Center) ---")
    target_pos_br = engine.calculate_target_geoposition(
        pixel_coords_bottom_right, uav_lat_lon_alt, uav_rpy, gimbal_rpy, target_alt_m
    )
    if target_pos_br:
        print(f"Target Position (Bottom Right): Lat={target_pos_br[0]:.6f}, Lon={target_pos_br[1]:.6f}, Alt={target_pos_br[2]:.2f}m")
        # Expected: Target should be South-East of the UAV, at 0m altitude.

    # --- Test Case 3: Tilted UAV and Gimbal ---
    print("\n--- Test Case 3: Tilted UAV and Gimbal ---")
    uav_rpy_tilted = (radians(5), radians(5), radians(45)) # 5 deg roll/pitch, 45 deg yaw
    gimbal_rpy_tilted = (radians(-5), radians(-85), radians(-45)) # Gimbal counter-tilts to keep camera mostly down
    
    target_pos_tilted = engine.calculate_target_geoposition(
        pixel_coords_center, uav_lat_lon_alt, uav_rpy_tilted, gimbal_rpy_tilted, target_alt_m
    )
    if target_pos_tilted:
        print(f"UAV RPY: Roll={degrees(uav_rpy_tilted[0]):.2f}, Pitch={degrees(uav_rpy_tilted[1]):.2f}, Yaw={degrees(uav_rpy_tilted[2]):.2f}")
        print(f"Gimbal RPY: Roll={degrees(gimbal_rpy_tilted[0]):.2f}, Pitch={degrees(gimbal_rpy_tilted[1]):.2f}, Yaw={degrees(gimbal_rpy_tilted[2]):.2f}")
        print(f"Target Position (Tilted): Lat={target_pos_tilted[0]:.6f}, Lon={target_pos_tilted[1]:.6f}, Alt={target_pos_tilted[2]:.2f}m")
        # Expected: Target should be close to the UAV's position, at 0m altitude, as the gimbal compensates.

if __name__ == "__main__":
    simulate_geolocation()
