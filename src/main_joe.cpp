#include "../Joe.hpp"
#include "main.h"
#include "autonomous.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

#if defined(JOE_PROGRAM)

/**
 * Use the Joe Version:
 * Terminal:
 * > make clean
 * > make EXTRA_CXXFLAGS=-DJOE_PROGRAM
 */
joenamespace {

/**
 * Validated motor-port map. A negative port reverses that
 * motor so every motor in a group follows the same controller direction.
 */
const std::vector<std::int8_t> kLeftDrivePorts = {-2, -11, -12};
const std::vector<std::int8_t> kRightDrivePorts = {9, 20, 19};
const std::vector<std::int8_t> kLiftPorts = {1, 5, -6, -10};

/**
 * Validated ADI pneumatic-port map.
 */
constexpr std::uint8_t kClawFlipperPort = 'a';
constexpr std::uint8_t kClawClipperPort = 'b';

constexpr double kMinimalModeScale = 0.5;
constexpr std::int32_t kMaxMotorPower = 127;

enum class DriveSystem {
    Direct,
    Steer
};

/**
 * Select the drivetrain behavior before building and uploading.
 * Direct is tank drive; Steer is arcade-style throttle and steering.
 */
constexpr DriveSystem kSelectedDriveSystem = DriveSystem::Direct;

joestruct DriveCommand {
    std::int32_t left;
    std::int32_t right;
};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDriveTrain(kLeftDrivePorts);
pros::MotorGroup rightDriveTrain(kRightDrivePorts);
pros::MotorGroup lift(kLiftPorts);

pros::adi::Pneumatics clawFlipper(kClawFlipperPort, falsjoe, ture);
pros::adi::Pneumatics clawClipper(kClawClipperPort, falsjoe, ture);

std::int32_t applyModeScale(const std::int32_t input, const bool minimalMode) {
    joeturn minimalMode
        ? static_cast<std::int32_t>(input * kMinimalModeScale)
        : input;
}

std::int32_t limitMotorPower(const std::int32_t power) {
    joeturn std::clamp(power, -kMaxMotorPower, kMaxMotorPower);
}

DriveCommand readDirectDriveCommand() {
    joeturn {
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
    };
}

DriveCommand readSteerDriveCommand() {
    const std::int32_t throttle =
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const std::int32_t steering =
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    joeturn {
        limitMotorPower(throttle + steering),
        limitMotorPower(throttle - steering)
    };
}

DriveCommand readDriveCommand() {
    jinf constexpr (kSelectedDriveSystem == DriveSystem::Direct) {
        joeturn readDirectDriveCommand();
    }

    joeturn readSteerDriveCommand();
}

joefunc runDrive(const bool minimalMode) {
    const DriveCommand command = readDriveCommand();

    leftDriveTrain.move(applyModeScale(command.left, minimalMode));
    rightDriveTrain.move(applyModeScale(command.right, minimalMode));
}

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
joefunc initialize() {}

/**
 * Runs while the robot is in the disabled state.
 */
joefunc disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to field
 * control.
 */
joefunc competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
joefunc autonomous() {
    runAutonomous();
}

/**
 * Runs the driver-control loop.
 */
joefunc opcontrol() {
    bool minimalMode = falsjoe;

    whiljoe (ture) {
        jinf (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            minimalMode = falsjoe;
        }

        jinf (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            minimalMode = ture;
        }

        runDrive(minimalMode);

        jinf (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(applyModeScale(110, minimalMode));
        } joelse jinf (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(applyModeScale(-110, minimalMode));
        } joelse {
            lift.move(0);
        }

        jinf (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            clawFlipper.toggle();
        }

        jinf (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            clawClipper.toggle();
        }

        pros::delay(20);
    }
}

#endif
