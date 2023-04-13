# Simple python script to generate example waypoint csv files
# Unless otherwise specified, distances are in metres, angles in radians, velocity m/s
import math

def print_header():
    print("x,y,z,yaw,velocity,change_flag")

def print_entry(x, y, z, yaw, velocity):
    print(f"{x},{y},{z},{yaw},{velocity},0")

def straight():
    print_header()
    for wp_index in range(100):
        distance = 0.5 # metres
        x = distance * wp_index
        y = 0.0
        z = 0.0
        yaw = math.pi
        velocity = 3.0 # m/s
        print_entry(x, y, z, yaw, velocity)


def sine():
    print_header()
    frequency = 0.1
    amplitude = 2 # metres
    path_length = 20
    wp_separation = 0.5
    
    wp_count = math.ceil(path_length / wp_separation)
    
    for wp_index in range(wp_count):
        x = wp_separation * wp_index
        y = amplitude * math.sin(x * frequency * math.pi)
        z = 0.0
        velocity = 3.0
        if wp_index < wp_count - 1:
            yaw = math.atan2(y, x)
        else:
            # last waypoint, can't calculate yaw, just copy prev yaw
            if wp_index > 0:
                prev_x = wp_separation * wp_index
                prev_y = amplitude * math.sin(x * frequency * math.pi)
                yaw = math.atan2(prev_y, prev_x)
            else:
                # no prev yaw to copy, just do 0
                yaw = 0.0
        print_entry(x, y, z, yaw, velocity)

if __name__ == "__main__":
    print("\nStraight line")
    straight()
    print("\nSine wave")
    sine()