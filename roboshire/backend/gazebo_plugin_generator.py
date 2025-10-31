"""
Gazebo Plugin Generator

Generates Gazebo plugin XML for common sensors and actuators.
No manual XML editing needed!
"""

import logging
from typing import Dict, List, Optional
from jinja2 import Template


class GazeboPluginWizard:
    """
    Generate Gazebo plugins for URDF

    Supports:
    - Camera sensors
    - Lidar sensors
    - IMU sensors
    - Differential drive controllers
    - Depth cameras
    - GPS sensors
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.plugins = []

    def add_camera_plugin(self, link_name: str, camera_name: str, topic: str,
                          update_rate: int = 30, width: int = 640, height: int = 480) -> str:
        """
        Add camera sensor plugin

        Args:
            link_name: Link to attach camera to
            camera_name: Name of camera sensor
            topic: ROS topic for camera data
            update_rate: Update frequency in Hz
            width: Image width
            height: Image height

        Returns:
            XML string for plugin
        """
        template = Template('''
    <gazebo reference="{{ link_name }}">
      <sensor name="{{ camera_name }}" type="camera">
        <update_rate>{{ update_rate }}</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>{{ width }}</width>
            <height>{{ height }}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace></namespace>
            <remapping>~/image_raw:={{ topic }}</remapping>
            <remapping>~/camera_info:={{ topic }}/camera_info</remapping>
          </ros>
          <camera_name>{{ camera_name }}</camera_name>
          <frame_name>{{ link_name }}</frame_name>
          <hack_baseline>0.07</hack_baseline>
        </plugin>
      </sensor>
    </gazebo>
''')

        plugin_xml = template.render(
            link_name=link_name,
            camera_name=camera_name,
            topic=topic,
            update_rate=update_rate,
            width=width,
            height=height
        )

        self.plugins.append({
            'type': 'camera',
            'name': camera_name,
            'xml': plugin_xml
        })

        self.logger.info(f"Added camera plugin: {camera_name}")
        return plugin_xml

    def add_lidar_plugin(self, link_name: str, sensor_name: str, topic: str,
                        update_rate: int = 10, min_range: float = 0.12,
                        max_range: float = 10.0, samples: int = 360) -> str:
        """
        Add lidar sensor plugin

        Args:
            link_name: Link to attach lidar to
            sensor_name: Name of lidar sensor
            topic: ROS topic for lidar data
            update_rate: Update frequency in Hz
            min_range: Minimum range in meters
            max_range: Maximum range in meters
            samples: Number of range samples

        Returns:
            XML string for plugin
        """
        template = Template('''
    <gazebo reference="{{ link_name }}">
      <sensor name="{{ sensor_name }}" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <visualize>true</visualize>
        <update_rate>{{ update_rate }}</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>{{ samples }}</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>{{ min_range }}</min>
            <max>{{ max_range }}</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace></namespace>
            <remapping>~/out:={{ topic }}</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>{{ link_name }}</frame_name>
        </plugin>
      </sensor>
    </gazebo>
''')

        plugin_xml = template.render(
            link_name=link_name,
            sensor_name=sensor_name,
            topic=topic,
            update_rate=update_rate,
            min_range=min_range,
            max_range=max_range,
            samples=samples
        )

        self.plugins.append({
            'type': 'lidar',
            'name': sensor_name,
            'xml': plugin_xml
        })

        self.logger.info(f"Added lidar plugin: {sensor_name}")
        return plugin_xml

    def add_imu_plugin(self, link_name: str, sensor_name: str, topic: str,
                      update_rate: int = 100, enable_noise: bool = True) -> str:
        """
        Add IMU sensor plugin

        Args:
            link_name: Link to attach IMU to
            sensor_name: Name of IMU sensor
            topic: ROS topic for IMU data
            update_rate: Update frequency in Hz
            enable_noise: Add realistic noise to measurements

        Returns:
            XML string for plugin
        """
        noise_template = '''
          <noise>
            <type>gaussian</type>
            <rate>
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </rate>
            <accel>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </accel>
          </noise>
''' if enable_noise else ''

        template = Template('''
    <gazebo reference="{{ link_name }}">
      <sensor name="{{ sensor_name }}" type="imu">
        <always_on>true</always_on>
        <update_rate>{{ update_rate }}</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.002' if enable_noise else '0.0' }}</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.002' if enable_noise else '0.0' }}</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.002' if enable_noise else '0.0' }}</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.01' if enable_noise else '0.0' }}</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.01' if enable_noise else '0.0' }}</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>{{ '0.01' if enable_noise else '0.0' }}</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <namespace></namespace>
            <remapping>~/out:={{ topic }}</remapping>
          </ros>
          <frame_name>{{ link_name }}</frame_name>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
      </sensor>
    </gazebo>
''')

        plugin_xml = template.render(
            link_name=link_name,
            sensor_name=sensor_name,
            topic=topic,
            update_rate=update_rate,
            enable_noise=enable_noise
        )

        self.plugins.append({
            'type': 'imu',
            'name': sensor_name,
            'xml': plugin_xml
        })

        self.logger.info(f"Added IMU plugin: {sensor_name}")
        return plugin_xml

    def add_differential_drive_plugin(self, left_joint: str, right_joint: str,
                                     cmd_vel_topic: str = "/cmd_vel",
                                     odom_topic: str = "/odom",
                                     wheel_separation: float = 0.5,
                                     wheel_diameter: float = 0.1,
                                     max_wheel_torque: float = 20.0,
                                     max_wheel_acceleration: float = 1.0) -> str:
        """
        Add differential drive controller plugin

        Args:
            left_joint: Name of left wheel joint
            right_joint: Name of right wheel joint
            cmd_vel_topic: Topic to receive velocity commands
            odom_topic: Topic to publish odometry
            wheel_separation: Distance between wheels (m)
            wheel_diameter: Diameter of wheels (m)
            max_wheel_torque: Maximum torque (Nm)
            max_wheel_acceleration: Maximum acceleration (m/s²)

        Returns:
            XML string for plugin
        """
        template = Template('''
    <gazebo>
      <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <ros>
          <namespace></namespace>
          <remapping>cmd_vel:={{ cmd_vel_topic }}</remapping>
          <remapping>odom:={{ odom_topic }}</remapping>
        </ros>

        <!-- Wheels -->
        <left_joint>{{ left_joint }}</left_joint>
        <right_joint>{{ right_joint }}</right_joint>

        <!-- Kinematics -->
        <wheel_separation>{{ wheel_separation }}</wheel_separation>
        <wheel_diameter>{{ wheel_diameter }}</wheel_diameter>

        <!-- Limits -->
        <max_wheel_torque>{{ max_wheel_torque }}</max_wheel_torque>
        <max_wheel_acceleration>{{ max_wheel_acceleration }}</max_wheel_acceleration>

        <!-- Output -->
        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>false</publish_wheel_tf>

        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>base_link</robot_base_frame>
      </plugin>
    </gazebo>
''')

        plugin_xml = template.render(
            left_joint=left_joint,
            right_joint=right_joint,
            cmd_vel_topic=cmd_vel_topic,
            odom_topic=odom_topic,
            wheel_separation=wheel_separation,
            wheel_diameter=wheel_diameter,
            max_wheel_torque=max_wheel_torque,
            max_wheel_acceleration=max_wheel_acceleration
        )

        self.plugins.append({
            'type': 'diff_drive',
            'name': 'differential_drive_controller',
            'xml': plugin_xml
        })

        self.logger.info("Added differential drive plugin")
        return plugin_xml

    def add_depth_camera_plugin(self, link_name: str, camera_name: str,
                                topic: str, update_rate: int = 30,
                                width: int = 640, height: int = 480,
                                min_depth: float = 0.1, max_depth: float = 10.0) -> str:
        """
        Add depth camera plugin

        Args:
            link_name: Link to attach depth camera to
            camera_name: Name of depth camera
            topic: ROS topic for depth data
            update_rate: Update frequency in Hz
            width: Image width
            height: Image height
            min_depth: Minimum depth range (m)
            max_depth: Maximum depth range (m)

        Returns:
            XML string for plugin
        """
        template = Template('''
    <gazebo reference="{{ link_name }}">
      <sensor name="{{ camera_name }}" type="depth">
        <update_rate>{{ update_rate }}</update_rate>
        <camera>
          <horizontal_fov>1.047198</horizontal_fov>
          <image>
            <width>{{ width }}</width>
            <height>{{ height }}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>{{ min_depth }}</near>
            <far>{{ max_depth }}</far>
          </clip>
        </camera>
        <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace></namespace>
            <remapping>~/image_raw:={{ topic }}/image_raw</remapping>
            <remapping>~/depth/image_raw:={{ topic }}/depth/image_raw</remapping>
            <remapping>~/camera_info:={{ topic }}/camera_info</remapping>
          </ros>
          <camera_name>{{ camera_name }}</camera_name>
          <frame_name>{{ link_name }}</frame_name>
          <min_depth>{{ min_depth }}</min_depth>
          <max_depth>{{ max_depth }}</max_depth>
        </plugin>
      </sensor>
    </gazebo>
''')

        plugin_xml = template.render(
            link_name=link_name,
            camera_name=camera_name,
            topic=topic,
            update_rate=update_rate,
            width=width,
            height=height,
            min_depth=min_depth,
            max_depth=max_depth
        )

        self.plugins.append({
            'type': 'depth_camera',
            'name': camera_name,
            'xml': plugin_xml
        })

        self.logger.info(f"Added depth camera plugin: {camera_name}")
        return plugin_xml

    def add_gps_plugin(self, link_name: str, sensor_name: str, topic: str,
                      update_rate: int = 1) -> str:
        """
        Add GPS sensor plugin

        Args:
            link_name: Link to attach GPS to
            sensor_name: Name of GPS sensor
            topic: ROS topic for GPS data
            update_rate: Update frequency in Hz

        Returns:
            XML string for plugin
        """
        template = Template('''
    <gazebo reference="{{ link_name }}">
      <sensor name="{{ sensor_name }}" type="gps">
        <always_on>true</always_on>
        <update_rate>{{ update_rate }}</update_rate>
        <gps>
          <position_sensing>
            <horizontal>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2.0</stddev>
              </noise>
            </horizontal>
            <vertical>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>4.0</stddev>
              </noise>
            </vertical>
          </position_sensing>
        </gps>
        <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
          <ros>
            <namespace></namespace>
            <remapping>~/out:={{ topic }}</remapping>
          </ros>
          <frame_name>{{ link_name }}</frame_name>
        </plugin>
      </sensor>
    </gazebo>
''')

        plugin_xml = template.render(
            link_name=link_name,
            sensor_name=sensor_name,
            topic=topic,
            update_rate=update_rate
        )

        self.plugins.append({
            'type': 'gps',
            'name': sensor_name,
            'xml': plugin_xml
        })

        self.logger.info(f"Added GPS plugin: {sensor_name}")
        return plugin_xml

    def get_all_plugins(self) -> List[Dict]:
        """Get all configured plugins"""
        return self.plugins

    def clear_plugins(self):
        """Clear all plugins"""
        self.plugins.clear()

    def export_to_urdf(self, urdf_content: str) -> str:
        """
        Add all plugins to existing URDF content

        Args:
            urdf_content: Existing URDF XML string

        Returns:
            URDF with plugins added
        """
        if not self.plugins:
            return urdf_content

        # Find </robot> tag and insert plugins before it
        plugin_xml = '\n  <!-- Gazebo Plugins -->\n'
        for plugin in self.plugins:
            plugin_xml += plugin['xml'] + '\n'

        # Insert before closing </robot> tag
        if '</robot>' in urdf_content:
            urdf_content = urdf_content.replace('</robot>', f'{plugin_xml}\n</robot>')
        else:
            urdf_content += plugin_xml

        self.logger.info(f"Added {len(self.plugins)} plugins to URDF")
        return urdf_content

    def detect_gazebo_version(self) -> Optional[str]:
        """
        Detect installed Gazebo version

        Returns:
            Version string or None if not found
        """
        import subprocess

        try:
            # Try Gazebo Classic
            result = subprocess.run(['gazebo', '--version'],
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                version = result.stdout.strip()
                return version

        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        try:
            # Try new Gazebo (Ignition/Fortress/etc)
            result = subprocess.run(['gz', 'sim', '--version'],
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                version = result.stdout.strip()
                return f"Gazebo (new): {version}"

        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass

        self.logger.warning("Could not detect Gazebo version")
        return None
