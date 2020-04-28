from typing import List, Tuple, NamedTuple
import time

from evasdk import Eva

from grid2d import Grid2D, GridCorners, XYPoint


# Define the x and y coordinates for 3 corners of the grid
grid_corners1: GridCorners = [
    XYPoint(x = 0.25, y = 0), # Starting position
    XYPoint(x = 0.46, y = 0.1), # Length of the grid
    XYPoint(x = 0.42, y = 0.35), # width of the grid
]
# Symmetrical grid
grid_corners2: GridCorners = [
    XYPoint(x = 0.25, y = 0),
    XYPoint(x = 0.46,y = -0.1),
    XYPoint(x = 0.42, y = -0.35),
]

# Using the corners and an amount of rows and columns, make the Grid2D
my_test_grid1 = Grid2D(grid_corners1, rows = 4, columns = 4)
my_test_grid2 = Grid2D(grid_corners2, rows = 4, columns = 4)


# Connect to Eva
host_ip = "your host"
token = "your token"
eva = Eva(host_ip, token)

# Set some default poses and a default orientation
pose_home = [0.057526037, 0.7658633, -1.9867575, 0.026749607, -1.732109, -0.011505207]
end_effector_orientation = {'w': 0.0, 'x': 0.0, 'y': 1.0, 'z': 0.0}

# Be carefull with this dimension - depending on the type of gripper
grid_z_position: float = 0.27

print("Waiting for Robot lock")
with eva.lock():

    print('Eva moving to home position')
    eva.control_go_to(pose_home)


    for grid_position in my_test_grid1:
        # Calculate joint angles for the grid position and goto those joint angles
        print('Eva going to grid position x={:f}, y={:f}'.format(grid_position.x, grid_position.y))

        # Initial hover position
        hover_position1 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position+0.1}
        position_hover_angles = eva.calc_inverse_kinematics(pose_home, hover_position1, end_effector_orientation)
        eva.control_go_to(position_hover_angles['ik']['joints'])

        # Target position on the grid
        target_position1 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position}
        position_joint_angles = eva.calc_inverse_kinematics(pose_home, target_position1, end_effector_orientation)
        eva.control_go_to(position_joint_angles['ik']['joints'])

        # Perform an action on a gripper - pick up
        eva.gpio_set('d1', True)
        eva.gpio_set('d0', False)

        time.sleep(0.5)
        print('Eva performing action at grid waypoint')

        # Hover the end effector after the pick up
        hover_position1 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position+0.1}
        position_hover_angles = eva.calc_inverse_kinematics(pose_home, hover_position1, end_effector_orientation)
        eva.control_go_to(position_hover_angles['ik']['joints'])
        eva.control_go_to(pose_home)

        for grid_position in my_test_grid2:

            # Initial hover position on the second grid
            hover_position2 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position+0.1}
            position_hover_angles = eva.calc_inverse_kinematics(pose_home, hover_position2, end_effector_orientation)
            eva.control_go_to(position_hover_angles['ik']['joints'])

            # Calculate joint angles for the grid position and goto those joint angles
            print('Eva going to grid position x={:f}, y={:f}'.format(grid_position.x, grid_position.y))
            target_position2 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position}
            position_joint_angles = eva.calc_inverse_kinematics(pose_home, target_position2,  end_effector_orientation)
            eva.control_go_to(position_joint_angles['ik']['joints'])

            # Perform an action on a gripper - drop
            eva.gpio_set('d0', True)
            eva.gpio_set('d1', False)
            time.sleep(0.5)
            print('Eva performing action at grid waypoint')

            # Hover the end effector
            hover_position2 = {'x': grid_position.x, 'y': grid_position.y, 'z': grid_z_position+0.1}
            position_hover_angles = eva.calc_inverse_kinematics(pose_home, hover_position2, end_effector_orientation)
            eva.control_go_to(position_hover_angles['ik']['joints'])
            eva.control_go_to(pose_home)
            break

print("Grid movement complete, lock released")
