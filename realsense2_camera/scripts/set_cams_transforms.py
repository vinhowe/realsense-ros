import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from scipy.spatial.transform import Rotation as R
import sys
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

import termios
import tty
import os
import time
import math
import json
import argparse


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_status(transforms, mode, message):
    sys.stdout.write(
        "%-8s%-8s%-8s%-40s\r"
        % (
            mode,
            transforms[mode]["value"],
            transforms[mode]["step"],
            message,
        )
    )

class CamTransformPublisher(Node):
    """Based largely on
    https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
    """

    def __init__(self, from_frame, to_frame):
        super().__init__("cam_publisher")
        self._from_frame = from_frame
        self._to_frame = to_frame
        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def publish_transforms(self, transforms):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = self._from_frame

        static_transformStamped.child_frame_id = self._to_frame
        static_transformStamped.transform.translation.x = transforms["x"]["value"]
        static_transformStamped.transform.translation.y = transforms["y"]["value"]
        static_transformStamped.transform.translation.z = transforms["z"]["value"]

        quat = R.from_euler(
            "xyz",
            [
                math.radians(transforms["roll"]["value"]),
                math.radians(transforms["pitch"]["value"]),
                math.radians(transforms["azimuth"]["value"]),
            ],
        ).as_quat()

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        self._broadcaster.sendTransform(static_transformStamped)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Publishes tf2 data")
    parser.add_argument('from_frame', help='reference frame')
    parser.add_argument('to_frame', help='frame to publish')
    parser.add_argument('--file', type=str, help='file to save/load transforms data')
    parser.add_argument('--adjust', action='store_true')
    parser.add_argument('--transforms', type=float, metavar=("x", "y", "z", "azimuth", "pitch", "roll"), nargs=6, help='initial camera transformation, in meters and degrees')

    args = parser.parse_args()

    from_frame, to_frame = args.from_frame, args.to_frame
    filename = args.file
    print("Using file %s" % os.path.abspath(filename))

    keybinds = {
        "a": "x",
        "s": "y",
        "d": "z",
        "f": "azimuth",
        "g": "pitch",
        "h": "roll",
    }

    mode = "pitch"
    message = ""

    if args.transforms:
        x, y, z, yaw, pitch, roll = args.transforms
        transforms = {
            "x": {"value": x, "step": 0.1},
            "y": {"value": y, "step": 0.1},
            "z": {"value": z, "step": 0.1},
            "azimuth": {"value": yaw, "step": 1},
            "pitch": {"value": pitch, "step": 1},
            "roll": {"value": roll, "step": 1},
        }
        print("Use given initial values.")
    elif filename:
        try:
            with open(filename) as transforms_file:
                transforms = json.load(transforms_file)
            print("Read initial values from file.")
        except IOError as e:
            print("Failed reading initial parameters from file %s" % filename)
            print(
                "Initial parameters must be given for initial run or if an un-initialized file has been given."
            )
            sys.exit(-1)

    rclpy.init()

    publisher = CamTransformPublisher(from_frame, to_frame)

    if args.adjust:
        print
        print(
            "Press the following keys to change mode"
        )
        print("x: a")
        print("y: s")
        print("z: d")
        print("azimuth: f")
        print("pitch: g")
        print("roll: h")
        print("For each mode, press k to increase by step and j to decrease")
        print("Press K to multiply step by 2 or J to divide")
        print
        print("Press Q to quit")
        print

        print("%-8s%-8s%-8s%s" % ("Mode", "value", "step", "message"))
        print_status(transforms, mode, message)

    publisher.publish_transforms(transforms)

    while True:
        if args.adjust:
            kk = getch()

            if kk in keybinds:
                mode = keybinds[kk]
            else:
                if kk.upper() == "Q" or (len(kk) and bytes(kk, "utf-8")[0] == 3):
                    sys.stdout.write("\n")
                    exit(0)
                elif kk == "j":
                    transforms[mode]["value"] -= transforms[mode]["step"]
                elif kk == "k":
                    transforms[mode]["value"] += transforms[mode]["step"]
                elif kk == "J":
                    transforms[mode]["step"] /= 2.0
                elif kk == "K":
                    transforms[mode]["step"] *= 2.0
                else:
                    message = "Invalid key: " + kk
                # Round value because we run into floating point errors that mess up the output
                # TODO: See if this is precise enough
                transforms[mode]["value"] = round(transforms[mode]["value"], 8)

            print_status(transforms, mode, message)

            if filename:
                with open(filename, "w") as transforms_file:
                    json.dump(transforms, transforms_file, indent=4)

        publisher.publish_transforms(transforms)

        if not args.adjust:
            time.sleep(0.05)
