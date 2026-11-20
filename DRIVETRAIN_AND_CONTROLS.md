# Drivetrain, Controls, and Testing

## Current validated port map

Do not change these values without retesting the affected mechanism.

| Mechanism | Smart or ADI ports | Notes |
| --- | --- | --- |
| Left drivetrain | `-2`, `-11`, `-12` | Three motors. A negative value reverses that motor. |
| Right drivetrain | `9`, `20`, `19` | Three motors. |
| Lift | `1`, `5`, `-6`, `-10` | Four synchronized motors. Negative values reverse those motors. |
| Claw flipper pneumatic | ADI `A` | Starts retracted; its extended state is electrically low. |
| Claw clipper pneumatic | ADI `B` | Starts retracted; its extended state is electrically low. |

## Selecting the drivetrain system

In `src/main.cpp`, choose the system before building and uploading:

```cpp
constexpr DriveSystem kSelectedDriveSystem = DriveSystem::Direct;
```

Use one of these values:

| Value | Behavior |
| --- | --- |
| `DriveSystem::Direct` | Tank drive: left joystick Y controls the left motor group and right joystick Y controls the right motor group. |
| `DriveSystem::Steer` | Arcade-style steering: left joystick Y is forward/reverse throttle and right joystick X is left/right steering. |

The direct and steer systems use the same motor-port map. Change only `kSelectedDriveSystem`, rebuild, and upload to test the other behavior.

## Controller mapping

| Controller input | Direct drivetrain | Steer drivetrain |
| --- | --- | --- |
| Left joystick Y | Left drivetrain | Forward and reverse throttle |
| Right joystick Y | Right drivetrain | Not used for driving |
| Right joystick X | Not used for driving | Steering left and right |
| D-pad Up | Select normal mode | Select normal mode |
| D-pad Left | Select minimal mode | Select minimal mode |
| L1, held | Raise lift | Raise lift |
| L2, held | Lower lift | Lower lift |
| R1, pressed | Toggle claw flipper | Toggle claw flipper |
| R2, pressed | Toggle claw clipper | Toggle claw clipper |

Normal mode uses full commanded motor power. Minimal mode scales drivetrain and lift commands to 50%. D-pad mode selections use a new press, so holding either button does not repeatedly switch modes. Pneumatic actions always use full output and do not scale in minimal mode.

## Steer-drive behavior

The steer system mixes throttle and steering as follows:

```text
left motor group  = throttle + steering
right motor group = throttle - steering
```

Commands are limited to the valid motor range of `-127` through `127` before minimal-mode scaling.

- With the left joystick centered, moving the right joystick rotates the robot in place.
- With the left joystick forward or backward, moving the right joystick makes the two sides run at different speeds, creating an arc.
- A small steering input creates a gentle turn; a full steering input creates the strongest turn that the motor limit allows.

## Code modules

| File | Responsibility |
| --- | --- |
| `src/main.cpp` | Hardware map, controller mapping, drive-system selection, and PROS competition callbacks. |
| `src/autonomous.cpp` | Autonomous routine body in `runAutonomous()`. |
| `include/autonomous.hpp` | Declares `runAutonomous()` for `main.cpp`. |
| `include/hivelib/` | Installed HiveLib localization library. It is not enabled yet. |

`main.cpp` keeps PROS's required `autonomous()` callback and forwards it to `runAutonomous()` in `src/autonomous.cpp`.

## HiveLib preparation

Before enabling HiveLib, change its default rotation-sensor and IMU ports. Its current defaults use smart ports `1` and `2`, which conflict with the validated lift and left-drive map. After the sensor ports are tested and updated, add `#include "hivelib/api.hpp"` in `src/autonomous.cpp` and configure the localizer there.

## Test checklist

1. With the robot safely elevated, select `DriveSystem::Direct`, build, upload, and verify each drivetrain side follows its joystick correctly.
2. Test normal mode, then press D-pad Left and confirm drive and lift move at half speed. Press D-pad Up to return to normal mode.
3. Hold L1 and L2 separately to verify lift direction and synchronization. Test R1 and R2 one press at a time for the intended pneumatic movement.
4. Change only `kSelectedDriveSystem` to `DriveSystem::Steer`, then rebuild and upload.
5. With the right joystick centered, move the left joystick forward and backward to confirm straight travel.
6. With the left joystick centered, move the right joystick X left and right to confirm in-place rotation.
7. Drive slowly with the left joystick and apply progressively larger right-joystick X inputs. Confirm the turn becomes sharper as the steering input increases.
8. If a mechanism behaves unexpectedly, stop the robot before changing code. Keep the validated port map unchanged unless a focused motor or pneumatic retest identifies a wiring or direction issue.
