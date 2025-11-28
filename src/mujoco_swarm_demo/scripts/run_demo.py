#!/usr/bin/env python3
"""
Multi-robot object pushing demonstration using MuJoCo.

This script demonstrates MuJoCo's capabilities for multi-agent robotic manipulation:
- Contact dynamics between robots and objects
- Multi-body physics simulation
- Real-time visualization
- Simple trajectory-based control
"""

import argparse
import time
import numpy as np
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: MuJoCo Python bindings not found.")
    print("Install with: pip install mujoco")
    exit(1)


class PushingDemoController:
    """Controller for multi-robot pushing demonstration."""

    def __init__(self, model_path, num_robots=2):
        """Initialize the demo controller."""
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # Load model and create data
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.num_robots = num_robots

        # Get actuator indices
        self.actuator_ids = self._get_actuator_ids()

        # Performance tracking
        self.start_time = None
        self.sim_time = 0
        self.step_count = 0

        print(f"✓ Model loaded: {self.model_path.name}")
        print(f"✓ Robots: {num_robots}")
        print(f"✓ Bodies: {self.model.nbody}")
        print(f"✓ Actuators: {self.model.nu}")
        print(f"✓ Contacts (max): {self.model.nconmax}")

    def _get_actuator_ids(self):
        """Get actuator IDs for all robots."""
        actuators = {}
        for robot_id in range(self.num_robots):
            left_name = f"robot_{robot_id}_left_actuator"
            right_name = f"robot_{robot_id}_right_actuator"

            try:
                left_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_name)
                right_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, right_name)
                actuators[robot_id] = {'left': left_id, 'right': right_id}
            except Exception as e:
                print(f"Warning: Could not find actuators for robot {robot_id}: {e}")

        return actuators

    def set_robot_velocity(self, robot_id, left_vel, right_vel):
        """Set wheel velocities for a specific robot."""
        if robot_id in self.actuator_ids:
            self.data.ctrl[self.actuator_ids[robot_id]['left']] = left_vel
            self.data.ctrl[self.actuator_ids[robot_id]['right']] = right_vel

    def get_box_position(self, box_id):
        """Get position of a box."""
        try:
            body_name = f"box_{box_id}"
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            return self.data.xpos[body_id].copy()
        except:
            return None

    def get_contact_info(self):
        """Get current contact information."""
        contacts = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1 = self.model.geom(contact.geom1).name
            geom2 = self.model.geom(contact.geom2).name
            force = np.linalg.norm(contact.frame[:3])  # Contact force magnitude
            contacts.append({
                'geom1': geom1,
                'geom2': geom2,
                'pos': contact.pos.copy(),
                'force': force
            })
        return contacts

    def demo_scenario_convergent_push(self, t):
        """
        Demo scenario: Robots converge on boxes from different sides.

        This demonstrates:
        - Multiple robots approaching targets
        - Contact dynamics (robot-box collisions)
        - Object manipulation through pushing
        """
        # Simple time-based control
        if t < 3.0:
            # Phase 1: Move forward toward center (0-3s)
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 5.0, 5.0)

        elif t < 6.0:
            # Phase 2: Slow push (3-6s)
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 2.0, 2.0)

        elif t < 8.0:
            # Phase 3: Turn and reposition (6-8s)
            for robot_id in range(self.num_robots):
                if robot_id % 2 == 0:
                    self.set_robot_velocity(robot_id, 3.0, -3.0)  # Turn left
                else:
                    self.set_robot_velocity(robot_id, -3.0, 3.0)  # Turn right

        elif t < 11.0:
            # Phase 4: Push from new angle (8-11s)
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 4.0, 4.0)

        else:
            # Phase 5: Stop (11s+)
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 0.0, 0.0)

    def demo_scenario_collision_test(self, t):
        """
        Demo scenario: Robots move in patterns that cause collisions.

        This demonstrates:
        - Robot-robot collisions
        - Robot-box collisions
        - Contact stability
        - Physics realism
        """
        period = 4.0  # Movement cycle period

        for robot_id in range(self.num_robots):
            phase = (t + robot_id * period / self.num_robots) % period

            if phase < period / 4:
                # Move forward
                self.set_robot_velocity(robot_id, 6.0, 6.0)
            elif phase < period / 2:
                # Turn
                self.set_robot_velocity(robot_id, 5.0, -5.0)
            elif phase < 3 * period / 4:
                # Move forward again
                self.set_robot_velocity(robot_id, 6.0, 6.0)
            else:
                # Turn opposite direction
                self.set_robot_velocity(robot_id, -5.0, 5.0)

    def demo_scenario_straight_lines(self, t):
        """
        Demo scenario: Simple straight-line motion.

        Simplest demo - robots just drive forward until hitting obstacles.
        Good for basic contact testing.
        """
        if t < 5.0:
            # All robots drive straight
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 5.0, 5.0)
        else:
            # Stop
            for robot_id in range(self.num_robots):
                self.set_robot_velocity(robot_id, 0.0, 0.0)

    def run_with_viewer(self, duration=15.0, scenario='convergent'):
        """
        Run simulation with interactive viewer.

        Args:
            duration: Simulation duration in seconds
            scenario: Demo scenario to run ('convergent', 'collision', 'straight')
        """
        print(f"\n▶ Running demo scenario: {scenario}")
        print(f"Duration: {duration}s")
        print("Controls: Double-click to pause, drag to rotate view\n")

        # Select scenario
        scenarios = {
            'convergent': self.demo_scenario_convergent_push,
            'collision': self.demo_scenario_collision_test,
            'straight': self.demo_scenario_straight_lines,
        }

        if scenario not in scenarios:
            print(f"Warning: Unknown scenario '{scenario}', using 'convergent'")
            scenario = 'convergent'

        control_function = scenarios[scenario]

        # Performance tracking
        self.start_time = time.time()
        self.sim_time = 0
        self.step_count = 0
        last_report = 0

        # Contact tracking
        total_contacts = 0
        max_contacts = 0

        # Launch viewer
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.distance = 4.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25

            while viewer.is_running() and self.data.time < duration:
                step_start = time.time()

                # Apply control based on scenario
                control_function(self.data.time)

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                self.step_count += 1

                # Track contacts
                num_contacts = self.data.ncon
                total_contacts += num_contacts
                max_contacts = max(max_contacts, num_contacts)

                # Sync viewer
                viewer.sync()

                # Performance reporting every 2 seconds
                if self.data.time - last_report > 2.0:
                    real_time = time.time() - self.start_time
                    rtf = self.data.time / real_time if real_time > 0 else 0
                    print(f"  t={self.data.time:.1f}s | RTF={rtf:.2f}x | "
                          f"Contacts={num_contacts}/{max_contacts}")
                    last_report = self.data.time

                # Maintain real-time if possible
                elapsed = time.time() - step_start
                if elapsed < self.model.opt.timestep:
                    time.sleep(self.model.opt.timestep - elapsed)

        # Final statistics
        real_time = time.time() - self.start_time
        avg_contacts = total_contacts / self.step_count if self.step_count > 0 else 0

        print("\n" + "="*60)
        print("DEMO SUMMARY")
        print("="*60)
        print(f"Simulation time:     {self.data.time:.2f}s")
        print(f"Real time:           {real_time:.2f}s")
        print(f"Real-time factor:    {self.data.time / real_time:.2f}x")
        print(f"Steps:               {self.step_count}")
        print(f"Avg contacts:        {avg_contacts:.1f}")
        print(f"Max contacts:        {max_contacts}")
        print(f"Timestep:            {self.model.opt.timestep*1000:.2f}ms")
        print("="*60)

        # Check if boxes moved
        print("\nBox displacement:")
        for i in range(3):  # Check first 3 boxes
            pos = self.get_box_position(i)
            if pos is not None:
                displacement = np.linalg.norm(pos[:2])  # XY displacement from origin
                print(f"  Box {i}: {displacement:.3f}m from origin")


def main():
    parser = argparse.ArgumentParser(
        description='Run multi-robot pushing demo in MuJoCo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Default demo with 2 robots
  python run_demo.py

  # Collision testing with 3 robots
  python run_demo.py -s collision -r 3

  # Quick straight-line test
  python run_demo.py -s straight -d 8

Scenarios:
  convergent - Robots converge on boxes and push them (demonstrates coordination)
  collision  - Robots move in patterns causing collisions (stress test)
  straight   - Simple straight-line motion (basic contact test)
        """
    )

    parser.add_argument('-s', '--scenario', type=str, default='convergent',
                       choices=['convergent', 'collision', 'straight'],
                       help='Demo scenario to run')
    parser.add_argument('-d', '--duration', type=float, default=15.0,
                       help='Simulation duration in seconds')
    parser.add_argument('-r', '--robots', type=int, default=2,
                       help='Number of robots (will regenerate scene)')
    parser.add_argument('-b', '--boxes', type=int, default=3,
                       help='Number of boxes (will regenerate scene)')
    parser.add_argument('--scene', type=str, default=None,
                       help='Path to scene XML file (overrides generation)')

    args = parser.parse_args()

    # Determine scene path
    script_dir = Path(__file__).parent
    if args.scene:
        scene_path = Path(args.scene)
    else:
        scene_path = script_dir / '../scenes/scene.xml'

        # Generate scene if needed or if robot/box count specified
        if not scene_path.exists() or args.robots != 2 or args.boxes != 3:
            print(f"Generating scene: {args.robots} robots, {args.boxes} boxes...")
            from generate_scene import generate_scene
            generate_scene(
                num_robots=args.robots,
                num_boxes=args.boxes,
                output_path=str(scene_path)
            )
            print()

    # Run demo
    try:
        controller = PushingDemoController(scene_path, num_robots=args.robots)
        controller.run_with_viewer(duration=args.duration, scenario=args.scenario)

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nMake sure to generate a scene first:")
        print("  python generate_scene.py")
        exit(1)

    except Exception as e:
        print(f"Error running demo: {e}")
        import traceback
        traceback.print_exc()
        exit(1)


if __name__ == '__main__':
    main()
