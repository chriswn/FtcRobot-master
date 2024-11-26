package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMovement {

    // Declare motors and servos
    private DcMotor forearm = null;
    private DcMotor shoulder = null;
    private Servo leftClaw = null;  // Left claw servo
    private Servo rightClaw = null; // Right claw servo

    // Constants
    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double MOTOR_POWER = 0.5;
    private static final double SERVO_OPEN_POSITION = 1.0;  // Open position for the claw
    private static final double SERVO_CLOSED_POSITION = 0.0; // Closed position for the claw

    private Telemetry telemetry;

    // Constructor to initialize hardware
    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        forearm = hardwareMap.get(DcMotor.class, "forearm");  // REV Robotics Core Hex Motor
        shoulder = hardwareMap.get(DcMotor.class, "shoulder"); // Tetrix Motor

        // Initialize servos for the gripper
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions (adjust as per your setup)
        forearm.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);
    }

    // Move arm by a given number of inches
    public void move(double inches) {
        resetEncoders();
        int ticks = calculateTicks(inches);
        setTargetPositions(ticks);
        runMotorsToPosition(MOTOR_POWER);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 30) {
            telemetry.addData("Forearm", forearm.getCurrentPosition());
            telemetry.addData("Shoulder", shoulder.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
    }

    // Open both claws (left and right)
    public void openGripper() {
        leftClaw.setPosition(SERVO_OPEN_POSITION);
        rightClaw.setPosition(SERVO_OPEN_POSITION);
    }

    // Close both claws (left and right)
    public void closeGripper() {
        leftClaw.setPosition(SERVO_CLOSED_POSITION);
        rightClaw.setPosition(SERVO_CLOSED_POSITION);
    }

    // Reset motor encoders
    private void resetEncoders() {
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Calculate ticks based on inches to move
    private int calculateTicks(double inches) {
        double wheelCircumference = WHEEL_DIAMETER * Math.PI;
        double rotations = inches / wheelCircumference;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    // Set target positions for the motors
    private void setTargetPositions(int ticks) {
        forearm.setTargetPosition(forearm.getCurrentPosition() + ticks);
        shoulder.setTargetPosition(shoulder.getCurrentPosition() + ticks);
    }

    // Run motors to the target positions
    private void runMotorsToPosition(double power) {
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setPower(power);
        shoulder.setPower(power);
    }

    // Check if motors are still moving (busy)
    private boolean motorsBusy() {
        return forearm.isBusy() || shoulder.isBusy();
    }

    // Stop all motors
    private void stopMotors() {
        forearm.setPower(0);
        shoulder.setPower(0);
    }
}

