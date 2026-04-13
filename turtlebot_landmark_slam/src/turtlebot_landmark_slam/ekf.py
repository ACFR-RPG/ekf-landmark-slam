import numpy as np
from turtlebot_landmark_slam.types import LandmarkMeasurement, ControlMeasurement
import turtlebot_landmark_slam.utils as utils
from copy import deepcopy


class ExtendedKalmanFilter(object):

    def __init__(self) -> None:
        self._state_vector = np.array([[0.0], [0.0], [0.0]])
        sigma_position = np.sqrt(10 ** (-3))
        sigma_orientation = np.sqrt(10 ** (-3))
        self._state_covariance = np.array(
            [
                [sigma_position**2,               0.0,                  0.0],
                [              0.0, sigma_position**2,                  0.0],
                [              0.0,               0.0, sigma_orientation**2]
            ]
        )

        self._landmark_index = {}

    @property
    def x(self):
        return deepcopy(self._state_vector[0])

    @property
    def y(self):
        return deepcopy(self._state_vector[1])

    @property
    def yaw(self):
        return deepcopy(self._state_vector[2])

    @property
    def pose(self):
        return np.array([self.x, self.y, self.yaw], copy=True)

    @property
    def pose_covariance(self):
        return np.array(self._state_covariance[0:3, 0:3], copy=True)

    @property
    def state_mean(self):
        return np.array(self._state_vector, copy=True)

    @property
    def state_covariance(self):
        return np.array(self._state_covariance, copy=True)

    def predict(self, control_meaurement: ControlMeasurement):
        motion_command = control_meaurement.motion_vector
        motion_covariance = control_meaurement.covariance
        pose = self._state_vector[0:3]

        self._predict_call_count = getattr(self, "_predict_call_count", 0) + 1
        if self._predict_call_count % 50 == 1:  # log every 50 calls (~1 Hz at 50 Hz odom)
            print(f"[EKF predict #{self._predict_call_count}] "
                  f"dx={motion_command[0,0]:.4f} dy={motion_command[1,0]:.4f} "
                  f"dtheta={motion_command[2,0]:.4f}  "
                  f"pose=({pose[0,0]:.3f}, {pose[1,0]:.3f}, {pose[2,0]:.3f})")

        # TODO: Implement the EKF prediction step using the process model.
        #       prediction, x_pred = f(x, u) + noise
        #       Use the helper in utils Relative2AbsolutePose to compute the predicted pose and the Jacobians F and W.
        #       Then compute the predicted state mean X and state covariance P using the EKF prediction equations.

        pass

    def update(self, landmark_measurement: LandmarkMeasurement, is_new: bool):
        pose = self.pose
        state_covariance = self.state_covariance
        state_mean = self.state_mean

        print(f"[EKF update] label={landmark_measurement.label}  "
              f"rel=({landmark_measurement.x:.3f}, {landmark_measurement.y:.3f})  "
              f"{'NEW' if is_new else 'seen'}")

        # TODO: Implement the EKF update step using measurement helpers in utils.
        #       measurement, z = h(x, l) + noise
        #       For a new landmark, use the helper in utils,
        #           Relative2AbsoluteXY to compute the landmark position in the absolute frame of reference and the Jacobians G1 and G2.
        #       For an observed landmark, use the helper in utils,
        #           Absolute2RelativeXY to compute the expected measurement and the Jacobians H and J.
        #       Then compute the innovation y, innovation covariance S, Kalman gain K, and update the state mean X and covariance P.

        pass
