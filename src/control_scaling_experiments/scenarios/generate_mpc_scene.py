#!/usr/bin/env python3
"""
Scene generator for MPC scaling experiments.
Creates scenes with n robots pushing a long cuboid along one face.
"""

import argparse
import math
from pathlib import Path


def generate_robot_body(robot_id, pos_x, pos_y, pos_z=0.0, yaw=0.0):
    """Generate XML for a single TurtleBot3 instance."""
    return f'''
    <!-- Robot {robot_id} -->
    <body name="robot_{robot_id}_base" pos="{pos_x} {pos_y} {pos_z}" euler="0 0 {yaw}">
      <freejoint name="robot_{robot_id}_joint"/>
      <geom pos="-0.064 0 0.01" quat="1 0 0 0" type="mesh" rgba="0.4 0.4 0.4 1" mesh="waffle_pi_base"/>
      <geom size="0.015 0.0045 0.01" pos="-0.177 -0.0639992 0.005" quat="0.707388 -0.706825 0 0" type="box"/>
      <geom size="0.015 0.0045 0.01" pos="-0.177 0.0640008 0.005" quat="0.707388 -0.706825 0 0" type="box"/>
      <geom pos="-0.049 0 0.1255" quat="1 0 0 0" type="mesh" rgba="0.3 0.3 0.3 1" mesh="lds"/>
      <geom size="0.0075 0.015 0.0135" pos="0.078 0 0.107" type="box"/>

      <body name="robot_{robot_id}_wheel_left" pos="0 0.144 0.033" quat="0.707388 -0.706825 0 0">
        <inertial pos="0 0 0" quat="-0.000890159 0.706886 0.000889646 0.707326" mass="0.0284989" diaginertia="2.07126e-05 1.11924e-05 1.11756e-05"/>
        <joint name="robot_{robot_id}_wheel_left_joint" pos="0 0 0" axis="0 0 1" limited="false" armature="0.01"/>
        <geom quat="0.707388 0.706825 0 0" type="mesh" rgba="0.3 0.3 0.3 1" mesh="left_tire"/>
      </body>

      <body name="robot_{robot_id}_wheel_right" pos="0 -0.144 0.033" quat="0.707388 -0.706825 0 0">
        <inertial pos="0 0 0" quat="-0.000890159 0.706886 0.000889646 0.707326" mass="0.0284989" diaginertia="2.07126e-05 1.11924e-05 1.11756e-05"/>
        <joint name="robot_{robot_id}_wheel_right_joint" pos="0 0 0" axis="0 0 1" limited="false" armature="0.01"/>
        <geom quat="0.707388 0.706825 0 0" type="mesh" rgba="0.3 0.3 0.3 1" mesh="right_tire"/>
      </body>
    </body>'''


def generate_robot_actuators(robot_id):
    """Generate actuator XML for a single robot."""
    return f'''
    <velocity name="robot_{robot_id}_left_actuator" ctrllimited="true" ctrlrange="-30.0 30.0"
              gear="1" kv="1.0" joint="robot_{robot_id}_wheel_left_joint" />
    <velocity name="robot_{robot_id}_right_actuator" ctrllimited="true" ctrlrange="-30.0 30.0"
              gear="1" kv="1.0" joint="robot_{robot_id}_wheel_right_joint" />'''


def generate_mpc_scene(num_robots, push_distance=10.0, output_path=None):
    """
    Generate a scene for MPC scaling experiments.

    Configuration:
    - n robots arranged symmetrically along one long face of a cuboid
    - Cuboid length scales with n (each robot gets ~0.35m of contact)
    - Cuboid positioned so it needs to be pushed push_distance meters
    - Robots start 0.5m behind the cuboid, facing forward (+x direction)

    Args:
        num_robots: Number of robots
        push_distance: Distance the payload must travel to reach goal (meters)
        output_path: Path to save the XML file

    Returns:
        xml_content: The generated XML as a string
    """

    # Cuboid sizing
    robot_spacing = 0.35  # Each robot gets ~35cm of contact space
    cuboid_length = max(0.5, num_robots * robot_spacing)  # Minimum 0.5m
    cuboid_width = 0.3  # Fixed width
    cuboid_height = 0.2  # Fixed height

    # Mass scaling: keep density constant, so mass scales with volume
    density = 50  # kg/m^3 (foam-like density, light enough for small robots to push)
    cuboid_volume = cuboid_length * cuboid_width * (2 * cuboid_height)
    cuboid_mass = density * cuboid_volume

    # Cuboid starts at x=0, needs to reach x=push_distance
    cuboid_start_x = 0.0
    cuboid_start_y = 0.0
    goal_x = push_distance

    # Robot positioning: behind cuboid (-x face), evenly spaced along y-axis
    robot_start_x = cuboid_start_x - cuboid_width/2 - 0.5  # 0.5m behind cuboid
    robot_positions = []

    for i in range(num_robots):
        # Distribute robots evenly along the length
        if num_robots == 1:
            y_pos = cuboid_start_y
        else:
            # Spread from -length/2 to +length/2, centered
            y_offset = (i / (num_robots - 1) - 0.5) * (cuboid_length * 0.9)  # 90% to avoid edges
            y_pos = cuboid_start_y + y_offset

        robot_positions.append((robot_start_x, y_pos, 0.033, 0.0))  # yaw=0 (facing +x)

    # Generate robot bodies
    robot_bodies = "\n".join([
        generate_robot_body(i, x, y, z, yaw)
        for i, (x, y, z, yaw) in enumerate(robot_positions)
    ])

    # Generate robot actuators
    robot_actuators = "\n".join([
        generate_robot_actuators(i)
        for i in range(num_robots)
    ])

    # Compute moment of inertia for cuboid (box shape)
    # I = (1/12) * m * (h^2 + w^2) for rotation about length axis
    Ixx = (1/12) * cuboid_mass * (cuboid_width**2 + (2*cuboid_height)**2)
    Iyy = (1/12) * cuboid_mass * (cuboid_length**2 + (2*cuboid_height)**2)
    Izz = (1/12) * cuboid_mass * (cuboid_length**2 + cuboid_width**2)

    # Complete scene XML
    xml_content = f'''<mujoco model="mpc_scaling_experiment">
  <compiler angle="radian" meshdir="../../../mujoco_swarm_demo/models/assets" autolimits="true"/>

  <option integrator="Euler" timestep="0.005">
    <flag contact="enable" warmstart="enable"/>
  </option>

  <size njmax="500" nconmax="200"/>

  <statistic center="{push_distance/2} 0 0.4" extent="{max(push_distance, 5)}"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <!-- Textures -->
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
             rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>

    <!-- Robot meshes -->
    <mesh name="waffle_pi_base" file="waffle_pi_base.stl" scale="0.001 0.001 0.001"/>
    <mesh name="left_tire" file="left_tire.stl" scale="0.001 0.001 0.001"/>
    <mesh name="right_tire" file="right_tire.stl" scale="0.001 0.001 0.001"/>
    <mesh name="lds" file="lds.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <default>
    <joint limited="false" armature="0.01" damping="0.1"/>
    <geom condim="4" friction="1 0.005 0.0001" solimp="0.99 0.99 0.001" solref="0.01 1"/>
    <equality solref="0.0002 1" solimp="0.99 0.99 0.0001"/>
  </default>

  <worldbody>
    <light pos="{push_distance/2} 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>

    <!-- Goal marker (visual only) -->
    <geom name="goal_marker" type="box" size="{cuboid_width/2} {cuboid_length/2} 0.01"
          pos="{goal_x} {cuboid_start_y} 0.01" rgba="0.3 0.8 0.3 0.3" contype="0" conaffinity="0"/>

    <!-- Robots -->
{robot_bodies}

    <!-- Payload cuboid -->
    <body name="payload" pos="{cuboid_start_x} {cuboid_start_y} {cuboid_height}">
      <freejoint name="payload_joint"/>
      <inertial pos="0 0 0" mass="{cuboid_mass:.3f}" diaginertia="{Ixx:.6f} {Iyy:.6f} {Izz:.6f}"/>
      <geom name="payload_geom" type="box" size="{cuboid_width/2} {cuboid_length/2} {cuboid_height}"
            rgba="0.8 0.4 0.2 0.9" friction="0.8 0.005 0.0001"/>
    </body>
  </worldbody>

  <!-- Actuators -->
  <actuator>
{robot_actuators}
  </actuator>

  <!-- Sensors -->
  <sensor>
    <!-- Payload state sensors -->
    <framepos name="payload_pos" objtype="body" objname="payload"/>
    <framequat name="payload_quat" objtype="body" objname="payload"/>
    <framelinvel name="payload_vel" objtype="body" objname="payload"/>
  </sensor>
</mujoco>
'''

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write(xml_content)
        print(f"MPC scene generated: {output_path}")
        print(f"  - Robots: {num_robots}")
        print(f"  - Cuboid dimensions: {cuboid_width:.2f} × {cuboid_length:.2f} × {2*cuboid_height:.2f} m")
        print(f"  - Cuboid mass: {cuboid_mass:.1f} kg")
        print(f"  - Push distance: {push_distance:.1f} m")
        print(f"  - Goal position: ({goal_x:.1f}, {cuboid_start_y:.1f})")

    return xml_content


def main():
    parser = argparse.ArgumentParser(
        description='Generate MuJoCo scene for MPC scaling experiments'
    )
    parser.add_argument('-n', '--num-robots', type=int, default=4,
                       help='Number of robots (default: 4)')
    parser.add_argument('-d', '--distance', type=float, default=10.0,
                       help='Push distance in meters (default: 10.0)')
    parser.add_argument('-o', '--output', type=str,
                       default='../scenes/mpc_scene.xml',
                       help='Output XML file path')

    args = parser.parse_args()

    # Get absolute path relative to script location
    script_dir = Path(__file__).parent
    output_path = script_dir / args.output

    generate_mpc_scene(
        num_robots=args.num_robots,
        push_distance=args.distance,
        output_path=str(output_path)
    )


if __name__ == '__main__':
    main()
