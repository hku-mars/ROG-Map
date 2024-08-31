#!/usr/bin/env /usr/bin/python3

import rospy
from quadrotor_msgs.msg import PositionCommand
import sys
import termios
import tty

velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
position_command = PositionCommand()

def get_key():
    """
    Capture keyboard input.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def update_velocity():
    """
    Update velocity based on keyboard input.
    """
    global velocity
    key = get_key()

    if key == 'w':  # Increase forward velocity
        velocity[0] += 0.1
    elif key == 's':  # Decrease forward velocity
        velocity[0] -= 0.1
    elif key == 'a':  # Increase left velocity
        velocity[1] += 0.1
    elif key == 'd':  # Decrease left velocity
        velocity[1] -= 0.1
    elif key == 'r':  # Increase upward velocity
        velocity[2] += 0.1
    elif key == 'f':  # Decrease upward velocity
        velocity[2] -= 0.1
    elif key == 'q':  # Quit
        rospy.signal_shutdown("User requested shutdown")
    # prees space to stop
    elif key==' ':
        velocity = [0.0, 0.0, 0.0]

    # clear the terminal and print help information
    # including : current postion, current velocity, how to stop and how to quit
    print("\033c")
    print("Current Position: ({:.2f}, {:.2f}, {:.2f})".format(position_command.position.x, position_command.position.y, position_command.position.z))
    print("Current Velocity: ({:.2f}, {:.2f}, {:.2f})".format(velocity[0], velocity[1], velocity[2]))
    print("Press 'Space' to stop the drone")
    print("Press 'Q' to quit the program")


def publish_position_command(event):
    """
    Timer callback to publish PositionCommand messages.
    """
    global velocity, position_command

    # Check if event.last_real is None (first callback call)
    if event.last_real is None:
        delta_time = 0.1  # Assume the first delta time as 0.1 seconds (10 Hz)
    else:
        delta_time = event.current_real.to_sec() - event.last_real.to_sec()

    # Update position based on velocity
    position_command.position.x += velocity[0] * delta_time
    position_command.position.y += velocity[1] * delta_time
    position_command.position.z += velocity[2] * delta_time

    # Update the message fields
    position_command.header.stamp = rospy.Time.now()
    position_command.velocity.x = velocity[0]
    position_command.velocity.y = velocity[1]
    position_command.velocity.z = velocity[2]

    pub.publish(position_command)

if __name__ == '__main__':
    rospy.init_node('keyboard_position_command_node')
    pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)


    position_command.position.x = 0
    position_command.position.y = 0
    position_command.position.z = 1.5

    # Set a timer to call the publish_position_command function at 10 Hz
    rospy.Timer(rospy.Duration(0.1), publish_position_command)

    # Continuously check for keyboard input to update the velocity
    try:
        while not rospy.is_shutdown():
            update_velocity()
    except rospy.ROSInterruptException:
        pass
