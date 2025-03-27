#include "main.h"
//Device setup
pros::Imu IMU(6);
//pros::ADIANALOGIN leftEncoder(ADI_ANALOG_IN)
pros::ADIAnalogIn leftEncoder('A');
pros::ADIAnalogIn rightEncoder('B');
pros::ADIAnalogIn backEncoder('C');


const double disL = 5.3;
const double disR = 5.3;
const double disB = 6.3;
const double wheelCircum = 4.2;
const double pi = 3.14159265359;
double leftDeg,rightDeg,backDeg,leftInch,rightInch,backInch,sideHeading,imuHeading,leftDegLast,rightDegLast,backDegLast,leftInchLast,rightInchLast,backInchLast,headingLast,headingDelta,backInchDelta, rightInchDelta, leftInchDelta, orientation = 0;
std::array <double, 2> localOffset = {0,0};
std::array <double, 2> robopos = {0,0};
/**Last
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
/// @brief  purpose of this function is to prevent the IMU from giving a heading of 360, because it is a heading of 0
/// @param deg the degree to check for overflow.
double overflowCheck(double deg){
	if (deg == 360){
		return 0;
	}
	else{
		return deg;
	}
}

/// @brief gets the delta (abs val) of 2 radians
/// @param rad1 Radian 1
/// @param rad2 Radian 2
/// @return Change between the 2
double subRadians(double rad1, double rad2){
	rad1 = tan((rad1 * pi / 180.0));
	rad2 = tan((rad2 * pi / 180.0));
	double delta = fabs(rad1 - rad2);
	return delta * pi/180;
}
/// @brief Converts a coordinate set from cartesian degrees to polar 
/// @param coordSet an array with the size of 2
void cartesianToPolar(std::array<double, 2>& coordSet){
	coordSet[0] = sqrt(pow(coordSet[0], 2) + pow(coordSet[1], 2));
	coordSet[1] = atan2(coordSet[1],coordSet[0]);
	
}
void polarToCartesian(double& radius, double& theta){
	double X = radius * cos(theta);
	double Y = radius * sin(theta);
	radius = X;
	theta = Y;
}

/// @brief Update deg and inch values based off encoder readings
void updateDistances(){
	double leftInchLast = leftInchLast;
	double rightInchLast = rightInchLast;
	double backInchLast = backInchLast;
	double leftDeg = leftEncoder.get_value();
	double rightDeg = rightEncoder.get_value();
	double backDeg = backEncoder.get_value();
	double leftInch = leftEncoder.get_value() / 360 * wheelCircum;
	double rightInch = rightEncoder.get_value() / 360 * wheelCircum;
	double backInch = backEncoder.get_value() / 360 * wheelCircum;
	double leftInchDelta = leftInch - leftInchLast;
	double rightInchDelta = rightInch - rightInchLast;
	double backInchDelta = backInch - backInchLast;
}
/// @brief Updates the IMU and side heading variables
void updateHeading(){
	double headingLast = imuHeading;
	double imuHeading = overflowCheck(IMU.get_heading());
	double sideHeading = (leftInch - rightInch) /(disL + disR);
	double headingDelta = subRadians(headingLast, imuHeading);
}
/// @brief updates the change in X and Y values
void calculateLocalOffset(){
	std::array <double, 2> localOffset = {backInchDelta, rightInchDelta};
	if(headingDelta != 0){
		localOffset[0] = 2 * sin(headingDelta/2) * ((backInchDelta / headingDelta) + disB);
		localOffset[1] = 2 * sin(headingDelta/2) * ((rightInchDelta / headingDelta) + disR);
	}
	else{
		localOffset[0] = backInchDelta;
		localOffset[1] = rightInchDelta;
	}
}
/// @brief Fixes the rotation offset and updates the global position values
void rotateToGlobalFrame(){
	double orientation = (headingLast + headingDelta) / 2;
	cartesianToPolar(localOffset);
	localOffset[1] = localOffset[1] - orientation;
	polarToCartesian(localOffset[0],localOffset[1]);
	robopos[0] += localOffset[0];
	robopos[1] += localOffset[1];
}

void on_center_button() {
	
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, std::to_string(sideHeading));
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	IMU.reset();
}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}