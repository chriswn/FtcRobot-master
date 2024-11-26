package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    // Declare drivetrain motors
    private DcMotor backLeftMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;

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
        wheelCircumference = Math.PI * WHEEL_DIAMETER; // Calculate wheel circumference once
    }

    // Initialize motors and their directions
    private void initializeMotors(HardwareMap hardwareMap) {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        // Set motor directions for proper drive behavior
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
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
    }
}
