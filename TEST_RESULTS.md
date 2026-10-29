# Local verification - 29 October 2026

Environment: Ubuntu 24.04 WSL 2, ROS 2 Jazzy, real turtlesim node with Qt offscreen, isolated ROS_DOMAIN_ID=76. This was an automated runtime test, not a mock simulator.

## Clean build

A fresh Ubuntu workspace at `/tmp/mec356_lab1_verified` built both packages successfully with `colcon build`:

```text
Finished <<< lab1_interfaces [11.4s]
Finished <<< lab1_control [1.85s]
Summary: 2 packages finished [13.4s]
```

The clean build completed without package warnings.

## Integration test output

```text
PASS repeated reset
PASS parameter and service validation
PASS four directions and final yaw
PASS live speed caps, overlap rejection and motion cancellation
PASS square run 1
PASS square run 2
PASS square bounds, cancellation and reset recovery
ALL INTEGRATION CHECKS PASSED
```

The test checked final position within 0.06 units and yaw within 0.05 rad, feedback delivery, negative/NaN/zero speed rejection, overlapping goal rejection, out-of-bounds goal rejection, reset rejection during motion, and velocity limits after a live update to 0.5 units/s and 0.6 rad/s.

Each square used side length 0.7. For each completed run, server logs showed exactly these four accepted pose goals, in order (rounded for display):

```text
(6.244, 5.544,  1.571)
(6.244, 6.244,  3.142)
(5.544, 6.244, -1.571)
(5.544, 5.544,  0.000)
```

## Verification scope

- Both installed packages and the installed launch/config files were exercised.
- The Qt GUI ran offscreen. Visual inspection of drawn lines, a visible desktop demonstration and screenshots have not been performed.
- A rosbag recording/replay has not been performed; README includes the procedure.
- Tests used a fresh local Ubuntu workspace, not a teammate's machine.
- The public repository contains source packages, documentation and tests; build/install/log output is excluded.
