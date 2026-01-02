#!/usr/bin/env python3
"""
Quick validation script to test the MPC experiments package.

Run this to verify:
1. All dependencies are installed
2. Scene generation works
3. MPC controller initializes
4. Basic simulation runs
"""

import sys
from pathlib import Path

print("=" * 60)
print("MPC Experiments Package Validation")
print("=" * 60)

# Test 1: Import dependencies
print("\n[1/5] Testing dependencies...")
try:
    import numpy as np
    print("  ✓ numpy")
except ImportError as e:
    print(f"  ✗ numpy: {e}")
    sys.exit(1)

try:
    import mujoco
    print("  ✓ mujoco")
except ImportError as e:
    print(f"  ✗ mujoco: {e}")
    sys.exit(1)

try:
    import casadi as ca
    print("  ✓ casadi")
except ImportError as e:
    print(f"  ✗ casadi: {e}")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    print("  ✓ matplotlib")
except ImportError as e:
    print(f"  ✗ matplotlib: {e}")
    sys.exit(1)

# Test 2: Scene generation
print("\n[2/5] Testing scene generation...")
try:
    from scenarios.generate_mpc_scene import generate_mpc_scene

    test_scene_path = Path(__file__).parent / "test_scene.xml"
    generate_mpc_scene(num_robots=4, push_distance=5.0, output_path=str(test_scene_path))

    if test_scene_path.exists():
        print("  ✓ Scene generated successfully")
        test_scene_path.unlink()  # Clean up
    else:
        print("  ✗ Scene file not created")
        sys.exit(1)
except Exception as e:
    print(f"  ✗ Scene generation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 3: Controller initialization
print("\n[3/5] Testing MPC controller...")
try:
    from controllers import CentralizedMPC

    controller = CentralizedMPC(
        num_robots=4,
        config={'horizon': 10, 'dt': 0.1}
    )
    print("  ✓ MPC controller initialized")
except Exception as e:
    print(f"  ✗ MPC controller failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 4: Control computation
print("\n[4/5] Testing control computation...")
try:
    payload_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot_states = np.array([
        [-0.5, -0.5, 0.0, 0.0],
        [-0.5, 0.0, 0.0, 0.0],
        [-0.5, 0.5, 0.0, 0.0],
        [-0.5, 1.0, 0.0, 0.0],
    ])
    goal_state = np.array([5.0, 0.0, 0.0])

    controls = controller.compute_control(payload_state, robot_states, goal_state, 0.05)

    if controls.shape == (4, 2):
        print(f"  ✓ Control computed: {controls.shape}")
        print(f"    Solve time: {controller.get_solve_time()*1000:.2f} ms")
    else:
        print(f"  ✗ Unexpected control shape: {controls.shape}")
        sys.exit(1)
except Exception as e:
    print(f"  ✗ Control computation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 5: Verify package structure
print("\n[5/5] Verifying package structure...")
base_dir = Path(__file__).parent
required_dirs = [
    "controllers",
    "scenarios",
    "experiments",
    "analysis",
]

all_exist = True
for dir_name in required_dirs:
    dir_path = base_dir / dir_name
    if dir_path.exists():
        print(f"  ✓ {dir_name}/")
    else:
        print(f"  ✗ {dir_name}/ missing")
        all_exist = False

if not all_exist:
    sys.exit(1)

print("\n" + "=" * 60)
print("✓ All tests passed!")
print("=" * 60)
print("\nPackage is ready to use. Next steps:")
print("1. cd experiments")
print("2. python run_scaling_experiment.py -n 2,4,8")
print("3. cd ../analysis")
print("4. python plot_scaling_laws.py")
print("=" * 60)
