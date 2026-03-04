#!/usr/bin/env python3
"""
Scene generator for multi-robot object pushing demonstrations.
Creates MuJoCo XML scenes with configurable numbers of robots and objects.
"""

import argparse
import os
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


def generate_box(box_id, pos_x, pos_y, size_x, size_y, size_z, mass=1.0):
    """Generate XML for a pushable box."""
    return f'''
    <!-- Box {box_id} -->
    <body name="box_{box_id}" pos="{pos_x} {pos_y} {size_z}">
      <freejoint name="box_{box_id}_joint"/>
      <inertial pos="0 0 0" mass="{mass}" diaginertia="{mass/3} {mass/3} {mass/3}"/>
      <geom name="box_{box_id}_geom" type="box" size="{size_x} {size_y} {size_z}"
            rgba="0.8 0.3 0.3 0.9" friction="0.7 0.005 0.0001"/>
    </body>'''


def generate_scene(num_robots=2, num_boxes=3, output_path=None):
    """
    Generate a complete MuJoCo scene XML file.

    Args:
        num_robots: Number of robots to spawn
        num_boxes: Number of boxes to spawn
        output_path: Path to save the XML file
    """

    # Robot positions - arranged in a circle around central box, facing inward
    import math
    robot_positions = []
    radius = 0.8  # Closer to box for quicker collision
    box_center = (0.0, 0.0)  # Central box position

    for i in range(num_robots):
        # Distribute robots evenly around circle
        angle = 2 * math.pi * i / num_robots
        x = box_center[0] + radius * math.cos(angle)
        y = box_center[1] + radius * math.sin(angle)

        # Calculate yaw to face toward box center
        # Robot's forward is +x in local frame, so yaw should point toward center
        yaw = math.atan2(box_center[1] - y, box_center[0] - x)

        robot_positions.append((x, y, 0.033, yaw))

    # Box positions - arranged in center
    box_configs = [
        # (x, y, size_x, size_y, size_z, mass)
        (0.0, 0.0, 0.2, 0.2, 0.15, 2.0),    # Large center box
        (0.5, 0.3, 0.15, 0.15, 0.1, 1.0),   # Medium box
        (-0.4, 0.2, 0.1, 0.1, 0.08, 0.5),   # Small box
        (0.3, -0.5, 0.25, 0.15, 0.12, 1.5), # Rectangular box
        (-0.5, -0.3, 0.12, 0.12, 0.1, 0.8), # Another small box
    ]

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

    # Generate boxes
    boxes = "\n".join([
        generate_box(i, x, y, sx, sy, sz, m)
        for i, (x, y, sx, sy, sz, m) in enumerate(box_configs[:num_boxes])
    ])

    # Complete scene XML
    xml_content = f'''<mujoco model="multi_robot_pushing_demo">
  <compiler angle="radian" meshdir="../models/assets" autolimits="true"/>

  <option integrator="Euler" timestep="0.005">
    <flag contact="enable" warmstart="enable"/>
  </option>

  <size njmax="500" nconmax="100"/>

  <statistic center="0 0 0.4" extent="2.5"/>

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
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>

    <!-- Robots -->
{robot_bodies}

    <!-- Objects to push -->
{boxes}
  </worldbody>

  <!-- Actuators -->
  <actuator>
{robot_actuators}
  </actuator>

  <!-- Sensors for contact detection -->
  <sensor>
    <!-- Contact sensors could be added here for monitoring -->
  </sensor>
</mujoco>
'''

    if output_path:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write(xml_content)
        print(f"Scene generated: {output_path}")
        print(f"  - Robots: {num_robots}")
        print(f"  - Boxes: {num_boxes}")

    return xml_content


def main():
    parser = argparse.ArgumentParser(
        description='Generate MuJoCo scene for multi-robot object pushing'
    )
    parser.add_argument('-r', '--robots', type=int, default=2,
                       help='Number of robots (default: 2)')
    parser.add_argument('-b', '--boxes', type=int, default=3,
                       help='Number of boxes (default: 3)')
    parser.add_argument('-o', '--output', type=str,
                       default='../scenes/scene.xml',
                       help='Output XML file path')

    args = parser.parse_args()

    # Get absolute path relative to script location
    script_dir = Path(__file__).parent
    output_path = script_dir / args.output

    generate_scene(
        num_robots=args.robots,
        num_boxes=args.boxes,
        output_path=str(output_path)
    )


if __name__ == '__main__':
    main()
