"""
point_cloud_pub.py
──────────────────
Builds an accumulated 3D point cloud from TOF range readings and the
scanner head's spherical-coordinate odometry.
 
HOW A POINT IS PLACED:
  The scanner head sits on the semicircle arc at position (θ, φ). The TOF
  sensor on the head points RADIALLY OUTWARD from the Z axis — i.e., in the
  same direction as the vector from the center to the head, extended further
  by the measured range.
 
  Radial unit vector pointing from origin toward scanner head:
    û = (cos φ · cos θ,  cos φ · sin θ,  sin φ)
 
  Measured surface point:
    P = (R + d) · û
      = ( (R+d) · cos φ · cos θ,
          (R+d) · cos φ · sin θ,
          (R+d) · sin φ          )
 
  where R = ARC_RADIUS_M (imported from odom_tf_pubs.py) and d = TOF range.
 
  If your sensor points in a different direction, update _compute_point().
 
Topics subscribed:
  /tof/range  (sensor_msgs/Range)    — distance from VL53L0X (metres)
  /odom       (nav_msgs/Odometry)    — scanner head position
                                       twist.twist.linear.x = θ (degrees)
                                       twist.twist.linear.y = φ (degrees)
 
Topics published:
  /point_cloud  (sensor_msgs/PointCloud2)  — accumulated 3D scan, 'odom' frame
"""