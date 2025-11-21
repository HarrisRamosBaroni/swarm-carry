# VSCode Development Setup (Optional)

This guide covers setting up VSCode for development with this project. **VSCode is completely optional**—you can use any editor you prefer.

## Prerequisites

- VSCode installed
- Dev Containers extension (`ms-vscode-remote.remote-containers`)

## Quick Setup

1. **Open folder in VSCode**: `File -> Open Folder` (select repo root)
2. **Reopen in container**: Command palette (`Ctrl+Shift+P`) → "Dev Containers: Reopen in Container"
3. **Wait for build**: First time takes ~5-10 minutes
4. **Verify**: Terminal should show `ros@...` and bottom-left should say "Dev Container"

## VSCode Tasks

The `.vscode/tasks.json` file provides shortcuts for common operations:

### Build Tasks
- `Ctrl+Shift+B`: Default build
- Command palette → "Tasks: Run Task" → "build" (release build)
- Command palette → "Tasks: Run Task" → "debug" (debug build)

### Test Tasks
- Command palette → "Tasks: Run Task" → "test"

### Linting Tasks
- Command palette → "Tasks: Run Task" → "lint all"

### Workspace Tasks
- "install dependencies": Run rosdep to install package dependencies
- "purge": Clean all build artifacts
- "new ament_cmake package": Create new C++ ROS2 package
- "new ament_python package": Create new Python ROS2 package

## Recommended VSCode Configuration

Create `.vscode/` directory in repo root (it's gitignored, so personal preferences stay local):

### `.vscode/tasks.json`

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "detail": "Build workspace (default)",
            "type": "shell",
            "command": "./scripts/build.sh",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": "$gcc"
        },
        {
            "label": "test",
            "detail": "Run all unit tests",
            "type": "shell",
            "command": "./scripts/test.sh",
            "group": {
                "kind": "test",
                "isDefault": true
            }
        },
        {
            "label": "setup",
            "detail": "Install dependencies",
            "type": "shell",
            "command": "./scripts/setup.sh",
            "problemMatcher": []
        },
        {
            "label": "purge",
            "detail": "Clean all build artifacts",
            "type": "shell",
            "command": "sudo rm -fr build install log; sudo py3clean .",
            "problemMatcher": []
        }
    ]
}
```

### `.vscode/settings.json`

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "/opt/ros/jazzy/local/lib/python3.12/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/jazzy/lib/python3.12/site-packages",
        "/opt/ros/jazzy/local/lib/python3.12/dist-packages"
    ],
    "cmake.configureOnOpen": false,
    "editor.rulers": [100],
    "files.associations": {
        "*.repos": "yaml",
        "*.world": "xml",
        "*.xacro": "xml"
    }
}
```

### `.vscode/launch.json`

For debugging ROS2 nodes:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "env": {
                "PYTHONPATH": "${workspaceFolder}/install/lib/python3.12/site-packages:${env:PYTHONPATH}"
            }
        },
        {
            "name": "ROS: Launch File",
            "type": "ros",
            "request": "launch",
            "target": "${file}"
        }
    ]
}
```

### `.vscode/c_cpp_properties.json`

For C++ intellisense:

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/jazzy/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64",
            "compileCommands": "${workspaceFolder}/build/compile_commands.json"
        }
    ],
    "version": 4
}
```

## Recommended Extensions

Install these extensions for best experience:

### Essential
- `ms-vscode-remote.remote-containers` - Dev Containers support
- `ms-iot.vscode-ros` - ROS2 support
- `ms-python.python` - Python language support
- `ms-vscode.cpptools` - C++ language support

### Helpful
- `twxs.cmake` - CMake syntax highlighting
- `redhat.vscode-yaml` - YAML language support
- `DotJoshJohnson.xml` - XML formatting
- `smilerobotics.urdf` - URDF/SDF preview
- `streetsidesoftware.code-spell-checker` - Spell checker
- `zachflower.uncrustify` - Code formatting

## Debugging

### Python Nodes
1. Set breakpoints in Python code
2. Run "Python: Current File" debug configuration
3. Or attach to running node (advanced)

### C++ Nodes
1. Build in debug mode: `BUILD_TYPE=Debug ./scripts/build.sh`
2. Use gdb in terminal: `gdb --args ros2 run <package> <executable>`

### Launch Files
1. Open launch file
2. Run "ROS: Launch File" debug configuration

## Terminal Access

### Multiple Terminals
- VSCode integrated terminal automatically sources ROS2 workspace
- Open new terminals with `Ctrl+Shift+` \`
- Each terminal is inside the container

### External Terminal Access
If you want to use a different terminal emulator:

```bash
# Find container name
docker ps

# Exec into container
docker exec -it <container-name> -u ros /bin/bash

# Source workspace
source install/setup.bash
```

## GUI Applications (noVNC)

When using the devcontainer, GUI applications (Gazebo, RViz) are accessed via browser:

- **Web interface**: http://localhost:6080
- **VNC client**: `localhost:5901`

The desktop-lite feature provides a full desktop environment in the browser.

## Tips

1. **First build**: After reopening in container, run setup task before building
2. **Intellisense**: After first build, reload window for C++ intellisense to pick up compile_commands.json
3. **Permissions**: If you get permission errors, the container is running as `ros` user
4. **Restart**: If container misbehaves, rebuild: Command palette → "Dev Containers: Rebuild Container"

## Troubleshooting

### "Cannot find ROS packages"
- Run setup task to install dependencies
- Source workspace: `. install/setup.bash` or restart terminal

### "C++ intellisense not working"
- Build workspace to generate `compile_commands.json`
- Reload VSCode window

### "GUI doesn't show up"
- Access noVNC at http://localhost:6080
- Check port forwarding is active (Ports tab)

### "Container build fails"
- Check Docker is running
- Check internet connection (downloads packages)
- Try rebuild with no cache

## Alternative: Native VSCode (No Container)

You can also use VSCode without containers:

1. Install ROS2 Jazzy and Gazebo Harmonic natively
2. Open folder in VSCode
3. Install recommended extensions
4. Use tasks.json to run scripts
5. Terminal will use your host system

This requires manual ROS2/Gazebo installation but can be faster for development.
