
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "/home/sabari/Workspace/ThesisV2/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for i, command in enumerate(commands):
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    if i == 1:
        time.sleep(10)

    print("Executed command ", command)
