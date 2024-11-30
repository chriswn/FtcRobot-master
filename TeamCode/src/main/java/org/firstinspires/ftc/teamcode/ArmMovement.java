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
    private static final double TICKS_PER_REVOLUTION = 560.0; // For REV Core Hex Motor
    private static final double MOTOR_POWER = 0.5;           // Motor power level
    private static final double SERVO_OPEN_POSITION = 1.0;  // Claw open position
    private static final double SERVO_CLOSED_POSITION = 0.0; // Claw closed position

    private Telemetry telemetry;

    // Constructor to initialize hardware
    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        forearm = hardwareMap.get(DcMotor.class, "forearm");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");

        // Initialize servos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions (adjust based on your robot design)
        forearm.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set motors to use encoders
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Move the shoulder and forearm motors by a specified number of ticks.
     *
     * @param shoulderTicks Number of encoder ticks to move the shoulder.
     * @param forearmTicks  Number of encoder ticks to move the forearm.
     */
    public void moveArmToPosition(int shoulderTicks, int forearmTicks) {
        // Set target positions
        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks);
        forearm.setTargetPosition(forearm.getCurrentPosition() + forearmTicks);

        // Run motors to position
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(MOTOR_POWER);
        forearm.setPower(MOTOR_POWER);

        // Wait for motors to reach their positions
        ElapsedTime runtime = new ElapsedTime();
        while ((shoulder.isBusy() || forearm.isBusy()) && runtime.seconds() < 5) {
            telemetry.addData("Shoulder Position", shoulder.getCurrentPosition());
            telemetry.addData("Forearm Position", forearm.getCurrentPosition());
            telemetry.update();
        }

        // Stop motors
        stopMotors();
    }
    public void moveShoulderToPosition(int ticks) {
        moveArmToPosition(ticks, 0); // Move only the shoulder
    }

    public void rotateForearmToAngle(int ticks) {
        moveArmToPosition(0, ticks); // Move only the forearm
    }

    /**
     * Open the gripper.
     */
    public void openGripper() {
        leftClaw.setPosition(SERVO_OPEN_POSITION);
        rightClaw.setPosition(SERVO_OPEN_POSITION);
        telemetry.addData("Gripper", "Opened");
        telemetry.update();
    }

    /**
     * Close the gripper.
     */
    public void closeGripper() {
        leftClaw.setPosition(SERVO_CLOSED_POSITION);
        rightClaw.setPosition(SERVO_CLOSED_POSITION);
        telemetry.addData("Gripper", "Closed");
        telemetry.update();
    }

    /**
     * Stop all motors.
     */
    public void stopMotors() {
        shoulder.setPower(0);
        forearm.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();
    }
}
