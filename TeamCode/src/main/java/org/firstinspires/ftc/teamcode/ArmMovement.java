package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private static final double LEFT_CLAW_OPEN_POSITION = 1.0;  // Left claw open position
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.0; // Left claw closed position
    private static final double RIGHT_CLAW_OPEN_POSITION = -1.0;  // Right claw open position (inverted)
    private static final double RIGHT_CLAW_CLOSED_POSITION = 0.0; // Right claw closed position (inverted)

    private static final double TICKS_PER_REVOLUTION = 1440.0; // For Tetrix
    private static final double MOTOR_POWER = 1.0; // Motor power level
    private static final double TIMEOUT_SECONDS = 5.0; // Timeout for movement

    private Telemetry telemetry;

    // Constructor to initialize hardware
    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors and servos
        forearm = initializeMotor(hardwareMap, "forearm", DcMotor.Direction.REVERSE);
        shoulder = initializeMotor(hardwareMap, "shoulder", DcMotor.Direction.FORWARD );

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }
    private void resetEncoders() {
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    /**
     * Helper method to initialize a motor with default settings.
     */
    private DcMotor initializeMotor(HardwareMap hardwareMap, String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    /**
     * Converts degrees of rotation to encoder ticks.
     */
    private int degreesToTicks(double degrees) {
        return (int) (degrees / 360.0 * TICKS_PER_REVOLUTION);
    }

    /**
     * Move the shoulder and forearm motors by a specified number of ticks.
     *
     * @param shoulderTicks Number of encoder ticks to move the shoulder.
     * @param forearmTicks  Number of encoder ticks to move the forearm.
     *
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
        waitForMotors(shoulder, forearm);

        // Stop motors
        stopMotors();
    }

    /**
     * Waits for specified motors to finish or until timeout.
     */
    private void waitForMotors(DcMotor... motors) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < TIMEOUT_SECONDS) {
            boolean allIdle = true;
            for (DcMotor motor : motors) {
                if (motor.isBusy()) {
                    allIdle = false;
                }
            }
            if (allIdle) break;

            // Update telemetry during the wait
            telemetry.addData("Shoulder Position", shoulder.getCurrentPosition());
            telemetry.addData("Forearm Position", forearm.getCurrentPosition());
            telemetry.update();
        }
    }

    /**
     * Move the shoulder motor by a specific number of ticks.
     */
    public void moveShoulderToPosition(int ticks) {
        moveArmToPosition(ticks, 0); // Move only the shoulder
    }

    /**
     * Rotate the forearm motor by a specific number of ticks.
     */
    public void rotateForearmToAngle(int ticks) {
        moveArmToPosition(0, ticks); // Move only the forearm
    }

    /**
     * Open the gripper.
     */
    public void openGripper() {
        leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_OPEN_POSITION);
        telemetry.addData("Gripper", "Opened");
        telemetry.update();
    }

    /**
     * Close the gripper.
     */
    public void closeGripper() {
        leftClaw.setPosition(LEFT_CLAW_CLOSED_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
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

    /**
     * Move the arm by degrees for the shoulder and forearm.
     *
     * @param shoulderDegrees Degrees to rotate the shoulder.
     * @param forearmDegrees  Degrees to rotate the forearm.
     */ 
    public void moveArmByDegrees(double shoulderDegrees, double forearmDegrees) {
        int shoulderTicks = degreesToTicks(shoulderDegrees);
        int forearmTicks = degreesToTicks(forearmDegrees);
        moveArmToPosition(shoulderTicks, forearmTicks);
    }
}
