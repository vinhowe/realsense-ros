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


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_status(status):
    sys.stdout.write(
        "%-8s%-8s%-8s%-40s\r"
        % (
            status["mode"],
            status[status["mode"]]["value"],
            status[status["mode"]]["step"],
            status["message"],
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

    def publish_status(self, status):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = self._from_frame

        static_transformStamped.child_frame_id = self._to_frame
        static_transformStamped.transform.translation.x = status["x"]["value"]
        static_transformStamped.transform.translation.y = status["y"]["value"]
        static_transformStamped.transform.translation.z = status["z"]["value"]

        quat = R.from_euler(
            "xyz",
            [
                math.radians(status["roll"]["value"]),
                math.radians(status["pitch"]["value"]),
                math.radians(status["azimuth"]["value"]),
            ],
        ).as_quat()

        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        self._broadcaster.sendTransform(static_transformStamped)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("USAGE:")
        print("set_cams_transforms.py from_frame to_frame x y z azimuth pitch roll")
        print("x, y, z: in meters")
        print("azimuth, pitch, roll: in degrees")
        print
        print("If parameters are not given read last used parameters.")
        print
        print("[OPTIONS]")
        print("--file <file name> : if given, default values are loaded from file")
        sys.exit(-1)

    from_frame, to_frame = sys.argv[1:3]
    try:
        filename = sys.argv[sys.argv.index("--file") + 1]
        print("Using file %s" % os.path.abspath(filename))
    except:
        filename = os.path.join(os.path.dirname(__file__), "_set_cams_info_file.txt")
        print("Using default file %s" % os.path.abspath(filename))

    if len(sys.argv) >= 9:
        x, y, z, yaw, pitch, roll = [float(arg) for arg in sys.argv[3:9]]
        status = {
            "mode": "pitch",
            "x": {"value": x, "key": "a", "step": 0.1},
            "y": {"value": y, "key": "s", "step": 0.1},
            "z": {"value": z, "key": "d", "step": 0.1},
            "azimuth": {"value": yaw, "key": "k", "step": 1},
            "pitch": {"value": pitch, "key": "l", "step": 1},
            "roll": {"value": roll, "key": ";", "step": 1},
            "message": "",
        }
        print("Use given initial values.")
    else:
        try:
            status = json.load(open(filename, "r"))
            print("Read initial values from file.")
        except IOError as e:
            print("Failed reading initial parameters from file %s" % filename)
            print(
                "Initial parameters must be given for initial run or if an un-initialized file has been given."
            )
            sys.exit(-1)

    rclpy.init()

    publisher = CamTransformPublisher(from_frame, to_frame)

    print
    print(
        "Press the following keys to change mode: x, y, z, (a)zimuth, (p)itch, (r)oll"
    )
    print("x: a")
    print("y: s")
    print("z: d")
    print("azimuth: k")
    print("pitch: l")
    print("roll: ;")
    print("For each mode, press j to increase by step and f to decrease")
    print("Press h to multiply step by 2 or g to divide")
    print
    print("Press Q to quit")
    print

    mode_keymap = {value["key"]: key for key, value in status.items() if "key" in value}
    print("%-8s%-8s%-8s%s" % ("Mode", "value", "step", "message"))
    print_status(status)
    publisher.publish_status(status)
    while True:
        kk = getch()
        status["message"] = ""

        if kk in mode_keymap:
            status["mode"] = mode_keymap[kk]
        else:
            if kk.upper() == "Q" or (len(kk) and bytes(kk, "utf-8")[0] == 3):
                sys.stdout.write("\n")
                exit(0)
            elif kk == "f":
                status[status["mode"]]["value"] -= status[status["mode"]]["step"]
            elif kk == "j":
                status[status["mode"]]["value"] += status[status["mode"]]["step"]
            elif kk == "g":
                status[status["mode"]]["step"] /= 2.0
            elif kk == "h":
                status[status["mode"]]["step"] *= 2.0
            else:
                status["message"] = "Invalid key:" + kk
            # Round value because we run into floating point errors that mess up the output
            # TODO: See if this is precise enough
            status[status["mode"]]["value"] = round(status[status["mode"]]["value"], 8)

        print_status(status)
        publisher.publish_status(status)
        json.dump(status, open(filename, "w"), indent=4)
