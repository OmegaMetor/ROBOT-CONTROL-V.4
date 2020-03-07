#include "H:\Programs\VEX programming\V.4\include\main.h"
//
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
//void on_center_button() {

//}
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor left_mtr(1);
pros::Motor right_mtr(3);
pros::Motor right_back_mtr(4);
pros::Motor left_back_mtr(2);
pros::Motor Tray(10);
pros::Motor IntakeLeft(7);
pros::Motor IntakeRight(9);
pros::Motor Arms(8);
int TraySpeed;
bool A_IsPressed = DIGITAL_A;
bool B_IsPressed = DIGITAL_B;
bool X_IsPressed = DIGITAL_X;
bool UP_IsPressed = DIGITAL_UP;
bool DOWN_IsPressed = DIGITAL_DOWN;
bool R1_IsPressed = DIGITAL_R1;
bool R2_IsPressed = DIGITAL_R2;
bool L1_IsPressed = DIGITAL_L1;
bool L2_IsPressed = DIGITAL_L2;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */




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

 auto chassisMotion =
 	ChassisControllerBuilder()
 	.withMotors({1, 2},{3,4})
 	.withDimensions(AbstractMotor::gearset::green, {{4_in, 9.5_in}, imev5GreenTPR})
 .withOdometry()

 .buildOdometry();
auto Pathfind =
 AsyncMotionProfileControllerBuilder()

 .withOutput(chassisMotion)
 .withLimits({.5, 1.0, 10.0})
 .buildMotionProfileController();
 auto SlowPath =
 AsyncMotionProfileControllerBuilder()
 .withOutput(chassisMotion)
 .withLimits({.25,.75,10})
 .buildMotionProfileController();
 void initialize() {

   right_mtr.set_reversed(true);
   right_back_mtr.set_reversed(true);

   Pathfind->generatePath({
   {0_ft, 0_ft, 0_deg},
   {45_in, 0_ft, 0_deg}
   },
   "Part1"
   );
   Pathfind->generatePath({
  {0_ft, 0_ft, 0_deg},
  {30_in, -10_in, 90_deg}
},
  "Part2"
 );
 Pathfind->generatePath({
   {0_in, 0_in,0_deg},
   {30_in,15_in,0_deg}
 },
 "Part3"
);
SlowPath->generatePath(
  {
  {0_in,0_in,0_deg},
  {10_in,0_in,0_deg}
},
  "Part4"
);
Pathfind->generatePath(
  {
  {0_in,0_in,0_deg},
  {5_in,-5_in,270_deg}
},
  "Part5"
);
Pathfind->generatePath(
  {
    {0_in,0_in,0_deg},
    {2.5_in,-2.5_in,270_deg}
  },
  "CIRCLE"
);

 }
double AllowedError = .75;
double TrayUpAngle = 0;

void TrayUp()
{
  double CurrentPosition = Tray.get_position();
  double ABSError = abs(TrayUpAngle - CurrentPosition);
  while (ABSError>AllowedError) {
    double CurrentPosition = Tray.get_position();
    Tray = (TrayUpAngle - CurrentPosition);
    pros::delay(20);
  }
}

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

IntakeLeft = 100;
IntakeRight.move_velocity(-100);
TrayUpAngle = 90;
TrayUp();
TrayUpAngle = 0;
TrayUp();
TrayUpAngle = 90;
pros::delay(500);
Pathfind->setTarget("Part1");
Pathfind->waitUntilSettled();
Pathfind->removePath("Part1");
Pathfind->setTarget("Part2", true);
Pathfind->waitUntilSettled();
Pathfind->removePath("Part2");
Pathfind->setTarget("Part3");
Pathfind->waitUntilSettled();
TrayUp();
pros::delay(1000);
Pathfind->removePath("Part3");
IntakeLeft = -25;
IntakeRight.move_velocity(25);
SlowPath->setTarget("Part4",true);
SlowPath->waitUntilSettled();
SlowPath->removePath("Part4");
Pathfind->setTarget("Part5");
Pathfind->waitUntilSettled();
while (true) {
  Pathfind->setTarget("CIRCLE");
  Pathfind->waitUntilSettled();
}
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

	while (true) {
		int left_y_joystick = master.get_analog(ANALOG_LEFT_Y);
		int right_x_joystick = master.get_analog(ANALOG_RIGHT_X);
		int left_x_joystick = master.get_analog(ANALOG_LEFT_X);
		int right_y_joystick = master.get_analog(ANALOG_RIGHT_Y);
bool A_IsPressed = master.get_digital(DIGITAL_A);
bool B_IsPressed = master.get_digital(DIGITAL_B);
bool X_IsPressed = master.get_digital(DIGITAL_X);
bool UP_IsPressed = master.get_digital(DIGITAL_UP);
bool DOWN_IsPressed = master.get_digital(DIGITAL_DOWN);
bool R1_IsPressed = master.get_digital(DIGITAL_R1);
bool R2_IsPressed = master.get_digital(DIGITAL_R2);
bool L1_IsPressed = master.get_digital(DIGITAL_L1);
bool L2_IsPressed = master.get_digital(DIGITAL_L2);
Arms.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
Tray.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
/*
left_mtr = ((left_y_joystick)+(left_x_joystick));
left_back_mtr = (left_y_joystick);
right_mtr = ((left_y_joystick*-1)+(left_x_joystick));
right_back_mtr = (left_y_joystick);
*/



left_mtr.move_velocity(left_y_joystick+left_x_joystick);
right_mtr.move_velocity(left_y_joystick-left_x_joystick);
left_back_mtr.move_velocity(left_y_joystick+left_x_joystick);
right_back_mtr.move_velocity(left_y_joystick-left_x_joystick);


		if (X_IsPressed==true) {
			Tray = 80;
		} else if (B_IsPressed==true) {
			Tray = -80;
		} else {
			Tray = 0;
		}
		if (UP_IsPressed==true) {
			Arms = -50;
		} else if (DOWN_IsPressed==true) {
			Arms = 50;
		} else {
			Arms = 0;
		}
if (R1_IsPressed==true) {
	IntakeLeft = 100;
	IntakeRight = -100;
} else if (L1_IsPressed==true) {
	IntakeLeft = -100;
	IntakeRight = 100;
} else {
	IntakeRight = 0;
	IntakeLeft = 0;

}
	}

		pros::delay(20);
}
