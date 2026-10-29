"""Pure pose-control strategy, independent of ROS transport."""
import math

HOME = 5.544445
POSITION_TOLERANCE = 0.04
YAW_TOLERANCE = 0.03

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def valid_target(x, y, yaw):
    return all(math.isfinite(v) for v in (x, y, yaw)) and 0.2 <= x <= 10.8 and 0.2 <= y <= 10.8

def command(pose, target, max_linear, max_angular):
    x, y, theta = pose
    tx, ty, yaw = target
    distance = math.hypot(tx-x, ty-y)
    final_error = wrap(yaw-theta)
    if distance > POSITION_TOLERANCE:
        heading_error = wrap(math.atan2(ty-y, tx-x)-theta)
        # Turn in place first; this keeps the square's sides straight.
        linear = min(max_linear, 1.5*distance) if abs(heading_error) < 0.04 else 0.0
        angular = max(-max_angular, min(max_angular, 5.0*heading_error))
        return linear, angular, distance, final_error, False
    angular = max(-max_angular, min(max_angular, 4.0*final_error))
    return 0.0, angular, distance, final_error, abs(final_error) <= YAW_TOLERANCE
