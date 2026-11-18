#include "main.h"

#include <cstdint>
#include <vector>

namespace {

/**
 * Update these lists after motor-port testing. A negative port reverses that
 * motor so every motor in a group follows the same controller direction.
 */
const std::vector<std::int8_t> kLeftDrivePorts = {-11, -12, -13};
const std::vector<std::int8_t> kRightDrivePorts = {1, 2, 3};
const std::vector<std::int8_t> kLiftPorts = {4, 5};

/**
 * Update these ADI ports after pneumatic-port testing.
 */
constexpr std::uint8_t kClawFlipperPort = 'a';
constexpr std::uint8_t kClawClipperPort = 'b';

constexpr double kMinimalModeScale = 0.5;

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
void autonomous() {}

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

        leftDriveTrain.move(applyModeScale(
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), minimalMode));
        rightDriveTrain.move(applyModeScale(
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), minimalMode));

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift.move(applyModeScale(127, minimalMode));
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            lift.move(applyModeScale(-127, minimalMode));
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
