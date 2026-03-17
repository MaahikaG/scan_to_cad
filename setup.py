from setuptools import setup

package_name = 'ScanToCAD'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'motor_controller      = ScanToCAD.motor_controller:main',
            'odom_tf_pubs          = ScanToCAD.odom_tf_pubs:main',
            'sensor_pub            = ScanToCAD.sensor_pub:main',
            'point_cloud_pub       = ScanToCAD.point_cloud_pub:main',
            'pan_tilt_controller   = ScanToCAD.pan_tilt_controller:main',
            'lcd_display           = ScanToCAD.lcd_display:main',
        ],
    },
)