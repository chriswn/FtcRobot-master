package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    // Declare motors
    private DcMotor backLeftMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor shoulder = null; // Arm shoulder motor
    private DcMotor forearm = null; // Arm forearm motor

    // Declare servos for the gripper
    private Servo leftClaw;
    private Servo rightClaw;

    // Constants
    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double MOTOR_POWER = 0.5; // Default motor power

    private Telemetry telemetry;

    private double wheelCircumference; // Calculate once and reuse

    // Constructor to initialize hardware
    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeMotors(hardwareMap);
        initializeServos(hardwareMap); // Initialize servos here
        wheelCircumference = Math.PI * WHEEL_DIAMETER; // Calculate wheel circumference once
    }

    // Initialize motors and their directions
    private void initializeMotors(HardwareMap hardwareMap) {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        forearm = hardwareMap.get(DcMotor.class, "forearm");

        // Set motor directions for proper drive behavior
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set arm motor directions (check if reversed or not)
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        forearm.setDirection(DcMotor.Direction.REVERSE);
    }

    // Initialize servos for the gripper
    private void initializeServos(HardwareMap hardwareMap) {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    // Move robot forward for a specific distance
    public void forwardForDistance(double inches) {
        resetEncoders();

        // Calculate ticks needed for the desired distance
        int ticks = calculateTicks(inches);

        // Set target positions and move motors
        setTargetPositions(ticks);
        runMotorsToPosition(MOTOR_POWER);

        // Wait for motors to finish their movement
        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 30) {
            telemetry.addData("Motor Positions", "Left: %d, Right: %d",
                    frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    // Calculate the number of ticks for the given distance (in inches)
    private int calculateTicks(double inches) {
        double rotations = inches / wheelCircumference;  // Avoid recalculating circumference every time
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    // Set the target positions for the motors
    private void setTargetPositions(int ticks) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + ticks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + ticks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + ticks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + ticks);
    }

    // Run motors to the target position with given power
    private void runMotorsToPosition(double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    // Method to move the arm (shoulder and forearm)
    public void moveArm(double shoulderInches, double forearmInches) {
        int shoulderTicks = calculateTicks(shoulderInches);
        int forearmTicks = calculateTicks(forearmInches);

        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks);
        forearm.setTargetPosition(forearm.getCurrentPosition() + forearmTicks);

        shoulder.setPower(MOTOR_POWER);
        forearm.setPower(MOTOR_POWER);

        // Wait until the arm reaches the target positions
        ElapsedTime runtime = new ElapsedTime();
        while ((shoulder.isBusy() || forearm.isBusy()) && runtime.seconds() < 30) {
            telemetry.addData("Shoulder", shoulder.getCurrentPosition());
            telemetry.addData("Forearm", forearm.getCurrentPosition());
            telemetry.update();
        }

        shoulder.setPower(0);
        forearm.setPower(0);
    }

    // Set power for both left and right motors
    public void setMotorPower(double leftPower, double rightPower) {
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
    }

    // Reset motor encoders
    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Check if any of the motors are still busy (moving towards target)
    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() || backLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy();
    }

    // Stop all motors
    public void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        shoulder.setPower(0);
        forearm.setPower(0);
    }

    // Method to close the gripper (both servos)
    public void closeGripper() {
        leftClaw.setPosition(0.0);  // Adjust value as necessary for your specific gripper
        rightClaw.setPosition(1.0); // Adjust value as necessary for your specific gripper
    }

    // Method to open the gripper (both servos)
    public void openGripper() {
        leftClaw.setPosition(1.0);  // Adjust value as necessary for your specific gripper
        rightClaw.setPosition(0.0); // Adjust value as necessary for your specific gripper
    }
}
