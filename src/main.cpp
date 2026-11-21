#include "main.h"
#include "autonomous.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {

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
 * Note: Change DriveSystem::Direct to DriveSystem:Steer in order to switch system.
 */
constexpr DriveSystem kSelectedDriveSystem = DriveSystem::Direct;

struct DriveCommand {
    std::int32_t left;
    std::int32_t right;
};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftDriveTrain(kLeftDrivePorts);
pros::MotorGroup rightDriveTrain(kRightDrivePorts);
pros::MotorGroup lift(kLiftPorts);

pros::adi::Pneumatics clawFlipper(kClawFlipperPort, false, true);
pros::adi::Pneumatics clawClipper(kClawClipperPort, false, true);

std::int32_t applyModeScale(const std::int32_t input, const bool minimalMode) {
	return minimalMode
		? static_cast<std::int32_t>(input * kMinimalModeScale)
		: input;
}
std::int32_t limitMotorPower(const std::int32_t power) {
    return std::clamp(power, -kMaxMotorPower, kMaxMotorPower);
}

DriveCommand readDirectDriveCommand() {
    return {
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
    };
}

DriveCommand readSteerDriveCommand() {
    const std::int32_t throttle =
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const std::int32_t steering =
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    return {
        limitMotorPower(throttle + steering),
        limitMotorPower(throttle - steering)
    };
}

DriveCommand readDriveCommand() {
    if constexpr (kSelectedDriveSystem == DriveSystem::Direct) {
        return readDirectDriveCommand();
    }

    return readSteerDriveCommand();
}

void runDrive(const bool minimalMode) {
    const DriveCommand command = readDriveCommand();

    leftDriveTrain.move(applyModeScale(command.left, minimalMode));
    rightDriveTrain.move(applyModeScale(command.right, minimalMode));
}

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    runAutonomous();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    bool minimalMode = false;

    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            minimalMode = false;
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            minimalMode = true;
        }

        runDrive(minimalMode);

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(applyModeScale(110, minimalMode));
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(applyModeScale(-110, minimalMode));
        } else {
            lift.move(0);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            clawFlipper.toggle();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            clawClipper.toggle();
        }

        pros::delay(20);
    }
}
