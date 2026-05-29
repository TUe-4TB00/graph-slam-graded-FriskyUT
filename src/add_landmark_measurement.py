import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # TODO: Determine the correct rotation (bearing) and distance from X(4) to L(2) 

     # Get optimized pose X(4) and landmark L(2)
    pose4 = result.atPose2(X(4))
    landmark2 = result.atPoint2(L(2))

    # Difference from pose to landmark in global coordinates
    dx = landmark2[0] - pose4.x()
    dy = landmark2[1] - pose4.y()

    # Bearing from X(4) to L(2), relative to the robot orientation
    rotation_rad = math.atan2(dy, dx) - pose4.theta()
    rotation = math.degrees(rotation_rad)

    # Distance from X(4) to L(2)
    distance = np.sqrt(dx**2 + dy**2)

    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE))
    return graph