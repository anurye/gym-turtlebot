from abc import ABC, abstractmethod

from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Twist,
    TwistStamped,
)
import numpy as np
from std_msgs.msg import Header


class BaseConverter(ABC):
    """Abstract base class for ROS message conversion."""

    def __init__(self):
        super().__init__()

    @abstractmethod
    def to_dict(self, msg):
        pass

    @abstractmethod
    def from_dict(self, data):
        pass

    @abstractmethod
    def to_numpy(self, msg):
        pass

    @abstractmethod
    def from_numpy(self, data):
        pass


class TwistConverter(BaseConverter):
    """Converter for Twist and TwistStamped messages."""

    def __init__(self):
        super().__init__()

    def to_dict(self, msg: Twist) -> dict:
        return {
            'linear': [msg.linear.x, msg.linear.y, msg.linear.z],
            'angular': [msg.angular.x, msg.angular.y, msg.angular.z]
        }

    def from_dict(self, data: dict) -> Twist:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = data['linear']
        msg.angular.x, msg.angular.y, msg.angular.z = data['angular']
        return msg

    def to_numpy(self, msg: Twist) -> np.ndarray:
        return np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])

    def from_numpy(self, data: np.ndarray) -> Twist:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = data[:3]
        msg.angular.x, msg.angular.y, msg.angular.z = data[3:]
        return msg


class TwistStampedConverter(TwistConverter):
    """Converter for TwistStamped messages."""

    def __init__(self):
        super().__init__()

    def to_dict(self, msg: TwistStamped) -> dict:
        return {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'twist': super().to_dict(msg.twist)
        }

    def from_dict(self, data: dict) -> TwistStamped:
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp.sec = int(data['stamp'])
        msg.header.stamp.nanosec = int((data['stamp'] % 1) * 1e9)
        msg.header.frame_id = data['frame_id']
        msg.twist = super().from_dict(data['twist'])
        return msg


class PoseConverter(BaseConverter):
    """Converter for Pose and PoseStamped messages."""

    def __init__(self):
        super().__init__()

    def to_dict(self, msg: Pose) -> dict:
        return {
            'position': [msg.position.x, msg.position.y, msg.position.z],
            'orientation': [
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ]
        }

    def from_dict(self, data: dict) -> Pose:
        msg = Pose()
        (msg.position.x,
         msg.position.y,
         msg.position.z) = data['position']

        (msg.orientation.x,
         msg.orientation.y,
         msg.orientation.z,
         msg.orientation.w) = data['orientation']

        return msg

    def to_numpy(self, msg: Pose) -> np.ndarray:
        return np.array([
            msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])

    def from_numpy(self, data: np.ndarray) -> Pose:
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = data[:3]
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = data[3:]
        return msg


class PoseStampedConverter(PoseConverter):
    """Converter for PoseStamped messages."""

    def __init__(self):
        super().__init__()

    def to_dict(self, msg: PoseStamped) -> dict:
        return {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'pose': super().to_dict(msg.pose)
        }

    def from_dict(self, data: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp.sec = int(data['stamp'])
        msg.header.stamp.nanosec = int((data['stamp'] % 1) * 1e9)
        msg.header.frame_id = data['frame_id']
        msg.pose = super().from_dict(data['pose'])
        return msg
