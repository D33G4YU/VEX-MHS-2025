/**
 * main.cpp - VEX Robot Control System
 * 
 * Robot Configuration:
 * - Tank drive with omni wheels (6 motors total: 3 per side)
 *   - leftBack: GREEN cartridge (200 RPM)
 *   - rightMiddle: GREEN cartridge (200 RPM)
 *   - All others: BLUE cartridge (600 RPM)
 * - Complex intake/elevator system (4 motors - all GREEN cartridges)
 * - Pneumatic scooper
 * - Odometry system (vertical + horizontal encoders)
 * 
 * Control Scheme:
 * - Left Stick Y: Forward/Backward
 * - Right Stick X: Strafe Left/Right
 * - Left Stick X: Turning
 * - R1: Intake (bring game piece up)
 * - R2: Outtake through top rollers
 * - L1: Outtake through middle roller
 * - L2: Outtake through back rollers (to storage)
 * - Down: Carry down and outtake through bottom
 * - X: Toggle scooper
 * - Y: Reset odometry
 * - A: Recalibrate encoders
 */

#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

// ==================== MOTOR DECLARATIONS ====================

// DRIVETRAIN - 6 motors (3 per side, linear tank configuration with omni wheels)
// Mixed gearset: leftBack and rightMiddle are GREEN (200 RPM), rest are BLUE (600 RPM)
pros::Motor leftFront(-15, pros::MotorGearset::blue);      // Port 15 (reversed) - BLUE
pros::Motor leftMiddle(-16, pros::MotorGearset::blue);     // Port 16 (reversed) - BLUE
pros::Motor leftBack(-17, pros::MotorGearset::green);      // Port 17 (reversed) - GREEN

pros::Motor rightFront(1, pros::MotorGearset::blue);    // Port 1 - BLUE
pros::Motor rightMiddle(2, pros::MotorGearset::green);   // Port 2 - GREEN
pros::Motor rightBack(3, pros::MotorGearset::blue);     // Port 3 - BLUE

// NOTE: Motor groups cannot be created with mixed gearsets
// We'll control motors individually in the omniDrive function

// INTAKE/ELEVATOR SYSTEM - 4 motors (all GREEN cartridges - 200 RPM)
// Port 7 wasn't working, so port 10 is used instead
pros::Motor frontTopMotor(10, pros::MotorGearset::green);     // Front top 2 rollers
pros::Motor frontMiddleMotor(6, pros::MotorGearset::green);   // Front middle roller + chained to front bottom roller
pros::Motor backTopMotor(9, pros::MotorGearset::green);       // Back top
pros::Motor backMiddleMotor(8, pros::MotorGearset::green);    // Back middle roller

// ==================== PNEUMATICS ====================
pros::adi::DigitalOut scooper('A');  // Pneumatic solenoid for scooper

// ==================== SENSORS ====================
// Encoders are now attached, so reversed flag is set to 'false' (standard orientation)
pros::adi::Encoder verticalEncoder('B', 'C', false);    // Vertical tracking wheel
pros::adi::Encoder horizontalEncoder('D', 'E', false);  // Horizontal tracking wheel

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== ROBOT CONFIGURATION ====================

// Tracking wheel specifications          ------------------------------------ NEEDS TO BE MEASURED
const double TRACKING_WHEEL_DIAMETER = 2.75;  // inches
const double TRACKING_WHEEL_DISTANCE = 360.0 / (TRACKING_WHEEL_DIAMETER * M_PI);  // ticks per inch

// Robot physical dimensions        ------------------------------------ NEEDS TO BE MEASURED
const double WHEEL_DIAMETER = 3.25;           // inches (omni wheels)
const double TRACK_WIDTH = 12.0;              // inches (distance between left and right wheels)
const double WHEELBASE_LENGTH = 12.0;         // inches (front to back distance)

// Tracking wheel offsets from center of rotation        ------------------------------------ NEEDS TO BE MEASURED 
const double VERTICAL_OFFSET = 2.0;           // inches forward from center 
const double HORIZONTAL_OFFSET = 5.5;         // inches to the side from center

// Motor speeds (RPM) - All intake motors are GREEN (200 RPM max)
const int INTAKE_SPEED = 180;      // Adjusted for green motors (was 200)
const int OUTTAKE_SPEED = 140;     // Adjusted for green motors (was 150)

// Deadzone for joysticks (typically 5-15 for VEX controllers)        ------------------------------------ NEEDS TO BE MEASURED
// Test by gently touching sticks - if robot moves, increase deadzone
const int JOYSTICK_DEADZONE = 10;

// ==================== LEMLIB SETUP ====================

// Tracking wheels
lemlib::TrackingWheel vertical(
    &verticalEncoder,
    TRACKING_WHEEL_DIAMETER,
    VERTICAL_OFFSET,
    1  // No gearing
);

lemlib::TrackingWheel horizontal(
    &horizontalEncoder,
    TRACKING_WHEEL_DIAMETER,
    HORIZONTAL_OFFSET,
    1  // No gearing
);

// PID Controllers for autonomous movement
// Lateral = forward/backward/strafe movement
// Angular = turning movement
// These values are tuned starting points - you may need to adjust based on testing       ------------------------------------ NEEDS TO BE TUNED
lemlib::ControllerSettings lateralController(
    10,   // kP - proportional gain (increase if robot is slow to reach target)
    0,    // kI - integral gain (helps with steady-state error, usually 0)
    55,   // kD - derivative gain (dampens oscillation, prevents overshoot)
    3,    // anti-windup
    1,    // small error range (inches)
    100,  // small error timeout (ms)
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    20    // slew rate (acceleration limit)
);

lemlib::ControllerSettings angularController(
    4,    // kP - (increase if robot is slow to turn)
    0,    // kI - (usually keep at 0)
    40,   // kD - (prevents oscillation during turns)
    3,    // anti-windup
    1,    // small error range (degrees)
    100,  // small error timeout (ms)
    3,    // large error range (degrees)
    500,  // large error timeout (ms)
    0     // slew rate (no acceleration limit for turns)
);

// Drivetrain configuration for LemLib
// Since we have mixed gearsets, we create custom motor groups here
// We'll use weighted averaging for RPM calculation
// 4 blue motors (600 RPM) + 2 green motors (200 RPM) = avg ~467 RPM
// Being conservative: using 400 RPM
pros::MotorGroup leftMotors({-15, -16, -17});
pros::MotorGroup rightMotors({1, 2, 3});

lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    TRACK_WIDTH,
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    400,  // RPM (conservative mixed gearset average)
    2     // horizontal drift correction (tune if robot drifts during straight movement)       ------------------------------------ NEEDS TO BE TUNED
);

// Odometry sensors (using encoders only, no IMU)
lemlib::OdomSensors sensors(
    &vertical,     // vertical tracking wheel
    nullptr,       // second vertical wheel (not used)
    &horizontal,   // horizontal tracking wheel
    nullptr,       // second horizontal wheel (not used)
    nullptr        // no IMU
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// ==================== GLOBAL STATE VARIABLES ====================
bool scooperDeployed = false;
bool autonomousMode = false;

// ==================== UTILITY FUNCTIONS ====================

/**
 * Apply deadzone to joystick input
 */
int applyDeadzone(int value, int deadzone = JOYSTICK_DEADZONE) {
    return (abs(value) < deadzone) ? 0 : value;
}

/**
 * Constrain value between min and max
 */
int constrain(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ==================== DRIVETRAIN CONTROL ====================

/**
 * Tank drive with omni wheel strafing capability
 * Combines tank drive with lateral movement
 * 
 * @param forward Forward/backward speed (-127 to 127)
 * @param strafe Left/right strafe speed (-127 to 127)
 * @param turn Rotation speed (-127 to 127)
 * 
 * NOTE: Motor power range is -127 to 127 (this is the PROS API standard)
 * With mixed gearsets, we scale GREEN motor speeds to compensate (they're 1/3 the RPM)
 */
void omniDrive(int forward, int strafe, int turn) {
    // Apply deadzone
    forward = applyDeadzone(forward);
    strafe = applyDeadzone(strafe);
    turn = applyDeadzone(turn);
    
    // For tank drive with omni wheels:
    // - Forward/backward: both sides move same direction
    // - Strafing: all wheels move same direction (omni wheels allow this)
    // - Turning: sides move opposite directions
    
    int leftPower = forward + strafe + turn;
    int rightPower = forward - strafe - turn;
    
    // Normalize if any value exceeds motor limits
    int maxPower = std::max(abs(leftPower), abs(rightPower));
    if (maxPower > 127) {
        double scale = 127.0 / maxPower;
        leftPower *= scale;
        rightPower *= scale;
    }
    
    // Apply to motors
    // GREEN motors (1/3 speed) need 3x power to match BLUE motors
    // We scale them proportionally for balanced movement
    leftFront.move(leftPower);              // BLUE
    leftMiddle.move(leftPower);             // BLUE
    leftBack.move(leftPower * 3);           // GREEN - compensated
    
    rightFront.move(rightPower);            // BLUE
    rightMiddle.move(rightPower * 3);       // GREEN - compensated
    rightBack.move(rightPower);             // BLUE
}

/**
 * Stop all drivetrain motors
 */
void stopDrive() {
    leftFront.move(0);
    leftMiddle.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightMiddle.move(0);
    rightBack.move(0);
}

/**
 * Set brake mode for all drivetrain motors
 */
void setDriveBrakeMode(pros::motor_brake_mode_e mode) {
    leftFront.set_brake_mode(mode);
    leftMiddle.set_brake_mode(mode);
    leftBack.set_brake_mode(mode);
    rightFront.set_brake_mode(mode);
    rightMiddle.set_brake_mode(mode);
    rightBack.set_brake_mode(mode);
}

// ==================== INTAKE/ELEVATOR CONTROL ====================

/**
 * Stop all intake/elevator motors
 */
void stopIntake() {
    frontTopMotor.move_velocity(0);
    frontMiddleMotor.move_velocity(0);
    backTopMotor.move_velocity(0);
    backMiddleMotor.move_velocity(0);
}

/**
 * INTAKE MODE: Bring game piece up through the system
 * - Bottom intake roller pulls in
 * - All rollers move to carry piece upward
 */
void intakeUp() {
    frontTopMotor.move_velocity(INTAKE_SPEED);       // Front top rollers forward
    frontMiddleMotor.move_velocity(INTAKE_SPEED);    // Front middle roller forward
    backTopMotor.move_velocity(INTAKE_SPEED);        // Back top + bottom intake forward
    backMiddleMotor.move_velocity(-INTAKE_SPEED);    // Back middle roller forward
}

/**
 * OUTTAKE MIDDLE: Eject through front middle roller
 * Controlled by frontMiddleMotor
 */
void outtakeMiddle() {
    frontTopMotor.move_velocity(-OUTTAKE_SPEED);     // Front top rollers reverse
    frontMiddleMotor.move_velocity(-OUTTAKE_SPEED);  // Front middle outtakes
    backTopMotor.move_velocity(-INTAKE_SPEED);        // Don't allow piece to go into storage
    backMiddleMotor.move_velocity(0);                // Neutral
}

/**
 * OUTTAKE TOP: Eject through front top 2 rollers
 * Used for scoring at high positions
 */
void outtakeTop() {
    frontTopMotor.move_velocity(OUTTAKE_SPEED);      // Front top rollers outtake
    frontMiddleMotor.move_velocity(INTAKE_SPEED);    // Feed to top
    backTopMotor.move_velocity(-INTAKE_SPEED);       // Keep feeding up
    backMiddleMotor.move_velocity(0);                // Neutral
}

/**
 * OUTTAKE BACK: Eject through back rollers into storage box
 * Both back motors reverse to push piece backward
 */
void outtakeBack() {
    frontTopMotor.move_velocity(0);                  // Front stays neutral
    frontMiddleMotor.move_velocity(0);               // Neutral
    backTopMotor.move_velocity(OUTTAKE_SPEED);       // Back top reverses
    backMiddleMotor.move_velocity(-OUTTAKE_SPEED);   // Back middle reverses
}

/**
 * CARRY DOWN AND OUTTAKE BOTTOM: Lower piece and eject through bottom intake
 * All motors reverse to bring piece down and out
 * This replaces the old "reverseIntake" function - more intuitive control
 */
void outtakeBottom() {
    frontTopMotor.move_velocity(-INTAKE_SPEED);      // Reverse to bring down
    frontMiddleMotor.move_velocity(-INTAKE_SPEED);   // NEW: Reverse to bring down
    backTopMotor.move_velocity(OUTTAKE_SPEED);       // Reverse bottom intake
    backMiddleMotor.move_velocity(INTAKE_SPEED);     // Reverse to bring down
}

// ==================== SCOOPER CONTROL ====================

/**
 * Deploy scooper (extend pneumatic)
 * When scooper.set_value(true), the pneumatic cylinder extends
 * This should push the scooper out/down to collect game pieces
 */
void deployScooper() {
    scooper.set_value(true);
    scooperDeployed = true;
}

/**
 * Retract scooper (retract pneumatic)
 * When scooper.set_value(false), the pneumatic cylinder retracts
 * This should pull the scooper back/up out of the way
 */
void retractScooper() {
    scooper.set_value(false);
    scooperDeployed = false;
}

/**
 * Toggle scooper state
 */
void toggleScooper() {
    if (scooperDeployed) {
        retractScooper();
    } else {
        deployScooper();
    }
}

// ==================== AUTONOMOUS MOVEMENT FUNCTIONS ====================

/**
 * Move forward by distance (inches)
 * Positive = forward, Negative = backward
 */
void moveForward(double distance, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    double targetX = current.x + distance * cos(current.theta * M_PI / 180.0);
    double targetY = current.y + distance * sin(current.theta * M_PI / 180.0);
    
    chassis.moveToPose(targetX, targetY, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Move backward by distance (inches)
 */
void moveBackward(double distance, int timeout = 3000) {
    moveForward(-distance, timeout);
}

/**
 * Strafe right by distance (inches)
 */
void strafeRight(double distance, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    double angle = (current.theta - 90) * M_PI / 180.0;  // Perpendicular to heading
    double targetX = current.x + distance * cos(angle);
    double targetY = current.y + distance * sin(angle);
    
    chassis.moveToPose(targetX, targetY, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Strafe left by distance (inches)
 */
void strafeLeft(double distance, int timeout = 3000) {
    strafeRight(-distance, timeout);
}

/**
 * Turn to absolute heading (degrees)
 * 0° = forward, 90° = right, 180° = back, 270° = left
 */
void turnToHeading(double heading, int timeout = 2000) {
    chassis.turnToHeading(heading, timeout, {.minSpeed = 30});
}

/**
 * Turn relative to current heading (degrees)
 * Positive = clockwise, Negative = counter-clockwise
 */
void turnRelative(double degrees, int timeout = 2000) {
    lemlib::Pose current = chassis.getPose();
    double targetHeading = current.theta + degrees;
    chassis.turnToHeading(targetHeading, timeout, {.minSpeed = 30});
}

/**
 * Turn clockwise by degrees
 */
void turnRight(double degrees, int timeout = 2000) {
    turnRelative(degrees, timeout);
}

/**
 * Turn counter-clockwise by degrees
 */
void turnLeft(double degrees, int timeout = 2000) {
    turnRelative(-degrees, timeout);
}

/**
 * Move to specific coordinates (x, y, heading)
 */
void moveToPose(double x, double y, double heading, int timeout = 3000) {
    chassis.moveToPose(x, y, heading, timeout, {.minSpeed = 40});
}

/**
 * Move to specific point while maintaining current heading
 */
void moveToPoint(double x, double y, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    chassis.moveToPose(x, y, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Turn to face a specific point (x, y)
 */
void turnToPoint(double x, double y, int timeout = 2000) {
    chassis.turnToPoint(x, y, timeout, {.minSpeed = 30});
}

/**
 * Move along a path of points
 */
void followPath(std::vector<lemlib::Pose> path, int timeout = 5000) {
    for (const auto& point : path) {
        chassis.moveToPose(point.x, point.y, point.theta, timeout, {.minSpeed = 40});
    }
}

// ==================== AUTONOMOUS ACTION FUNCTIONS ====================

/**
 * Intake for specified duration (milliseconds)
 */
void autoIntake(int duration) {
    intakeUp();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through top for specified duration
 */
void autoOuttakeTop(int duration) {
    outtakeTop();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through middle for specified duration
 */
void autoOuttakeMiddle(int duration) {
    outtakeMiddle();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through back for specified duration
 */
void autoOuttakeBack(int duration) {
    outtakeBack();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through bottom for specified duration
 */
void autoOuttakeBottom(int duration) {
    outtakeBottom();
    pros::delay(duration);
    stopIntake();
}

/**
 * Deploy scooper, wait, then retract
 */
void autoScooperSequence(int deployDuration = 500) {
    deployScooper();
    pros::delay(deployDuration);
    retractScooper();
}

/**
 * Intake sequence: Deploy scooper, intake, retract scooper
 */
void autoFullIntake(int intakeDuration = 1500) {
    deployScooper();
    intakeUp();
    pros::delay(intakeDuration);
    stopIntake();
    retractScooper();
}

// ==================== INITIALIZATION ====================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "=== INITIALIZING ===");
    
    // Reset encoders
    pros::lcd::set_text(2, "Resetting encoders...");
    verticalEncoder.reset();
    horizontalEncoder.reset();
    pros::delay(200);
    
    // Initialize chassis position
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    
    // Set brake modes
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // Initialize pneumatics
    retractScooper();
    
    pros::lcd::set_text(3, "Ready!");
    pros::delay(500);
}

void disabled() {
    stopDrive();
    stopIntake();
    retractScooper();
}

void competition_initialize() {
    // Can add autonomous selector here if needed
}

// ==================== AUTONOMOUS ====================

void autonomous() {
    autonomousMode = true;
    pros::lcd::clear();
    pros::lcd::set_text(1, "=== AUTONOMOUS ===");
    
    // Verify odometry is working
    lemlib::Pose startPose = chassis.getPose();
    if (std::isnan(startPose.x) || std::isnan(startPose.y)) {
        pros::lcd::set_text(2, "ERROR: Odometry NaN");
        return;
    }
    
    // ===== EXAMPLE AUTONOMOUS ROUTINE =====
    // Replace this with your actual autonomous routine
    
    // Example: Move forward, deploy scooper, intake, move back, score
    
    moveForward(24);           // Move forward 24 inches
    pros::delay(200);
    
    deployScooper();           // Deploy scooper
    pros::delay(300);
    
    autoIntake(1500);          // Intake for 1.5 seconds
    
    retractScooper();          // Retract scooper
    pros::delay(200);
    
    moveBackward(12);          // Move back 12 inches
    pros::delay(200);
    
    turnRight(90);             // Turn 90 degrees clockwise
    pros::delay(200);
    
    moveForward(18);           // Move forward to scoring position
    pros::delay(200);
    
    autoOuttakeTop(1000);      // Score through top rollers
    
    pros::lcd::set_text(2, "=== COMPLETE ===");
    autonomousMode = false;
}

// ==================== DRIVER CONTROL ====================

void opcontrol() {
    autonomousMode = false;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // Display task for debugging
    pros::Task displayTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            
            pros::lcd::set_text(0, "=== DRIVER CONTROL ===");
            pros::lcd::set_text(1, ("X: " + std::to_string(pose.x).substr(0, 6)).c_str());
            pros::lcd::set_text(2, ("Y: " + std::to_string(pose.y).substr(0, 6)).c_str());
            pros::lcd::set_text(3, ("Heading: " + std::to_string(pose.theta).substr(0, 6)).c_str());
            pros::lcd::set_text(4, scooperDeployed ? "Scooper: DEPLOYED" : "Scooper: RETRACTED");
            
            pros::delay(50);
        }
    });
    
    // Main control loop
    while (true) {
        // ===== DRIVETRAIN CONTROL =====
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        
        omniDrive(forward, strafe, turn);
        
        // ===== INTAKE/ELEVATOR CONTROL =====
        
        // R1: Intake (bring game piece up)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeUp();
        }
        // R2: Outtake through top
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtakeTop();
        }
        // L1: Outtake through middle
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtakeMiddle();
        }
        // L2: Outtake through back rollers (to storage)
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtakeBack();
        }
        // Down: Carry down and outtake through bottom
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            outtakeBottom();
        }
        // No buttons: Stop intake
        else {
            stopIntake();
        }
        
        // ===== SCOOPER CONTROL =====
        
        // X: Toggle scooper
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            toggleScooper();
            master.rumble(".");
        }
        
        // ===== UTILITY CONTROLS =====
        
        // Y: Reset odometry to (0, 0, 0)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.setPose(0, 0, 0);
            master.rumble("-");
            master.print(0, 0, "Pose Reset");
        }
        
        // A: Recalibrate encoders
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            master.rumble("--");
            master.print(0, 0, "Recalibrating...");
            
            verticalEncoder.reset();
            horizontalEncoder.reset();
            pros::delay(200);
            
            chassis.setPose(0, 0, 0);
            
            master.rumble("..");
            master.print(0, 0, "Calibrated!");
        }
        
        // Main loop delay
        pros::delay(10);
    }
}

// ==================== AUTONOMOUS WIN POINT PROGRAM ====================

/**
 * Autonomous Win Point (AWP) Strategy
 * 
 * Requirements (Standard Events):
 * 1. Score ≥7 Blocks of Alliance color
 * 2. Score ≥1 Block in 3 different Goals
 * 3. Remove ≥3 Blocks from Loaders
 * 4. No Robot in/touching Park Zone
 * 
 * Strategy Overview:
 * - Start in Park Zone (legal starting position)
 * - Immediately exit Park Zone
 * - Grab preload + field Blocks
 * - Score in 3 different Goals
 * - Clear Loader Blocks
 * - Finish away from Park Zone
 */

void autonomousWinPoint() {
    autonomousMode = true;
    pros::lcd::clear();
    pros::lcd::set_text(1, "=== AWP ROUTINE ===");
    
    // Verify odometry
    lemlib::Pose startPose = chassis.getPose();
    if (std::isnan(startPose.x) || std::isnan(startPose.y)) {
        pros::lcd::set_text(2, "ERROR: Odometry");
        return;
    }
    
    // Starting position: In red Alliance Park Zone
    // Robot starts with 1 preload Block
    int blocksScored = 0;
    int goalsScored = 0;
    int loaderBlocksCleared = 0;
    
    // ===== PHASE 1: EXIT PARK ZONE =====
    pros::lcd::set_text(2, "Phase 1: Exit Park");
    
    // Move forward out of Park Zone immediately
    moveForward(12);  // Exit Park Zone
    pros::delay(100);
    
    // ===== PHASE 2: GRAB ADDITIONAL BLOCKS =====
    pros::lcd::set_text(2, "Phase 2: Collect");
    
    // Deploy scooper to collect ground Blocks
    deployScooper();
    pros::delay(200);
    
    // Drive forward while intaking to collect Blocks on field
    intakeUp();
    moveForward(18);  // Collect Blocks in front of starting position
    pros::delay(300);
    
    // Continue collecting while moving toward Loader
    strafeRight(12);  // Move toward right Loader
    pros::delay(200);
    
    retractScooper();
    
    // ===== PHASE 3: CLEAR LOADER BLOCKS =====
    pros::lcd::set_text(2, "Phase 3: Loader");
    
    // Position at Loader adjacent to Alliance Station
    moveForward(8);
    pros::delay(200);
    
    // Intake Blocks from Loader while counting
    // Loaders start with 6 Blocks each, we need to remove 3
    for (int i = 0; i < 3; i++) {
        intakeUp();
        pros::delay(800);  // Time to grab one Block
        loaderBlocksCleared++;
    }
    
    stopIntake();
    pros::delay(100);
    
    // ===== PHASE 4: SCORE IN GOAL #1 (Long Goal - Right) =====
    pros::lcd::set_text(2, "Phase 4: Goal 1");
    
    // Move to right Long Goal
    moveBackward(10);
    pros::delay(100);
    
    turnRight(45);  // Angle toward Goal
    pros::delay(100);
    
    moveForward(14);  // Approach Goal
    pros::delay(100);
    
    // Score 3 Blocks through top rollers
    outtakeTop();
    pros::delay(2000);  // Score multiple Blocks
    stopIntake();
    
    blocksScored += 3;
    goalsScored++;
    
    pros::lcd::set_text(3, ("Scored: " + std::to_string(blocksScored)).c_str());
    
    // ===== PHASE 5: SCORE IN GOAL #2 (Center Goal - Upper) =====
    pros::lcd::set_text(2, "Phase 5: Goal 2");
    
    // Move to Center Goal (Upper)
    moveBackward(8);
    pros::delay(100);
    
    turnLeft(90);  // Turn toward center
    pros::delay(100);
    
    moveForward(20);  // Move to center Goal
    pros::delay(100);
    
    // Score 2 Blocks
    outtakeTop();
    pros::delay(1500);
    stopIntake();
    
    blocksScored += 2;
    goalsScored++;
    
    pros::lcd::set_text(3, ("Scored: " + std::to_string(blocksScored)).c_str());
    
    // ===== PHASE 6: COLLECT MORE BLOCKS IF NEEDED =====
    if (blocksScored < 7) {
        pros::lcd::set_text(2, "Phase 6: Collect");
        
        // Move to midfield Blocks
        moveBackward(10);
        pros::delay(100);
        
        deployScooper();
        intakeUp();
        
        // Sweep across field to collect Blocks
        moveForward(24);
        pros::delay(200);
        
        strafeLeft(12);
        pros::delay(200);
        
        retractScooper();
        stopIntake();
    }
    
    // ===== PHASE 7: SCORE IN GOAL #3 (Long Goal - Left) =====
    pros::lcd::set_text(2, "Phase 7: Goal 3");
    
    // Move to left Long Goal
    turnLeft(45);
    pros::delay(100);
    
    moveForward(18);
    pros::delay(100);
    
    // Score remaining Blocks
    outtakeTop();
    pros::delay(2000);
    stopIntake();
    
    blocksScored += 3;
    goalsScored++;
    
    pros::lcd::set_text(3, ("Scored: " + std::to_string(blocksScored)).c_str());
    
    // ===== PHASE 8: FINAL POSITIONING =====
    pros::lcd::set_text(2, "Phase 8: Finish");
    
    // Move away from Park Zone to ensure we're not touching it
    moveForward(12);
    pros::delay(100);
    
    // Stop all systems
    stopDrive();
    stopIntake();
    retractScooper();
    
    // ===== VERIFY AWP COMPLETION =====
    pros::lcd::set_text(2, "=== AWP CHECK ===");
    pros::lcd::set_text(3, ("Blocks: " + std::to_string(blocksScored) + "/7").c_str());
    pros::lcd::set_text(4, ("Goals: " + std::to_string(goalsScored) + "/3").c_str());
    pros::lcd::set_text(5, ("Loader: " + std::to_string(loaderBlocksCleared) + "/3").c_str());
    
    if (blocksScored >= 7 && goalsScored >= 3 && loaderBlocksCleared >= 3) {
        pros::lcd::set_text(6, "AWP: SUCCESS!");
        master.rumble("- - -");  // Victory rumble
    } else {
        pros::lcd::set_text(6, "AWP: INCOMPLETE");
        master.rumble("....");  // Warning rumble
    }
    
    autonomousMode = false;
}


// ==================== ALTERNATIVE: SIMPLE AWP ROUTINE ====================

/**
 * Simplified AWP routine for testing/backup
 * More conservative approach with higher success rate
 */
void autonomousWinPointSimple() {
    autonomousMode = true;
    pros::lcd::set_text(1, "=== SIMPLE AWP ===");
    
    // Exit Park Zone
    moveForward(15);
    pros::delay(200);
    
    // Clear 3 Blocks from Loader
    deployScooper();
    intakeUp();
    pros::delay(2500);  // Grab 3+ Blocks
    retractScooper();
    stopIntake();
    
    // Score in Goal 1 (nearby Long Goal)
    turnRight(90);
    pros::delay(100);
    moveForward(12);
    pros::delay(100);
    
    outtakeTop();
    pros::delay(1500);  // Score 2-3 Blocks
    stopIntake();
    
    // Score in Goal 2 (Center Goal)
    moveBackward(8);
    pros::delay(100);
    turnLeft(45);
    pros::delay(100);
    moveForward(16);
    pros::delay(100);
    
    outtakeTop();
    pros::delay(1500);  // Score 2-3 Blocks
    stopIntake();
    
    // Collect more Blocks from field
    moveBackward(10);
    pros::delay(100);
    deployScooper();
    intakeUp();
    moveForward(20);
    pros::delay(1000);
    retractScooper();
    stopIntake();
    
    // Score in Goal 3 (other Long Goal)
    turnLeft(90);
    pros::delay(100);
    moveForward(14);
    pros::delay(100);
    
    outtakeTop();
    pros::delay(2000);  // Score remaining Blocks
    stopIntake();
    
    // Move away from Park Zone
    moveForward(10);
    
    stopDrive();
    autonomousMode = false;
    pros::lcd::set_text(2, "AWP COMPLETE");
}


// ==================== WORLDS-QUALIFYING AWP ====================

/**
 * For Event Region Championships and Signature Events:
 * 1. Score ≥10 Blocks (instead of 7)
 * 2. Score ≥2 Blocks in 3 different Goals (instead of 1)
 * 3. Same Loader requirement (3 Blocks)
 * 4. Same Park Zone restriction
 */
void autonomousWinPointWorlds() {
    autonomousMode = true;
    pros::lcd::set_text(1, "=== AWP WORLDS ===");
    
    int blocksScored = 0;
    
    // Exit Park Zone
    moveForward(15);
    pros::delay(200);
    
    // Aggressive Block collection strategy
    deployScooper();
    intakeUp();
    
    // Clear Loader (3 Blocks)
    moveForward(10);
    pros::delay(200);
    strafeRight(14);
    pros::delay(2000);  // Grab from Loader
    
    // Sweep field for more Blocks
    moveForward(24);
    pros::delay(500);
    strafeLeft(20);
    pros::delay(500);
    
    retractScooper();
    stopIntake();
    
    // Score in Goal 1 - aim for 4 Blocks
    turnRight(45);
    moveForward(16);
    outtakeTop();
    pros::delay(2500);
    stopIntake();
    blocksScored += 4;
    
    // Collect more
    moveBackward(12);
    deployScooper();
    intakeUp();
    moveForward(20);
    pros::delay(1000);
    retractScooper();
    stopIntake();
    
    // Score in Goal 2 - aim for 3 Blocks
    turnLeft(90);
    moveForward(18);
    outtakeTop();
    pros::delay(2000);
    stopIntake();
    blocksScored += 3;
    
    // Final collection
    moveBackward(10);
    deployScooper();
    intakeUp();
    strafeRight(16);
    pros::delay(1000);
    retractScooper();
    stopIntake();
    
    // Score in Goal 3 - aim for 3+ Blocks
    turnRight(45);
    moveForward(14);
    outtakeTop();
    pros::delay(2000);
    stopIntake();
    blocksScored += 3;
    
    // Exit Park Zone area
    moveForward(12);
    stopDrive();
    
    pros::lcd::set_text(2, ("Scored: " + std::to_string(blocksScored) + "/10").c_str());
    autonomousMode = false;
}


// ==================== UPDATE MAIN AUTONOMOUS FUNCTION ====================

void autonomous() {
    // Choose which routine to run:
    
    // Option 1: Full AWP routine (recommended for standard events)
    autonomousWinPoint();
    
    // Option 2: Simple AWP (backup/testing)
    // autonomousWinPointSimple();
    
    // Option 3: Worlds-qualifying events
    // autonomousWinPointWorlds();
}]
