package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMovement {

    private DcMotor forearm;
    private DcMotor shoulder;
    private Servo leftClaw;
    private Servo rightClaw;

    private static final double LEFT_CLAW_OPEN_POSITION = 1.0;
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.0;
    private static final double RIGHT_CLAW_OPEN_POSITION = -1.0;
    private static final double RIGHT_CLAW_CLOSED_POSITION = 0.0;

    private static final int SHOULDER_MIN_TICKS = 100;
    private static final int SHOULDER_MAX_TICKS = 1000;
    private static final int FOREARM_MIN_TICKS = 0;
    private static final int FOREARM_MAX_TICKS = 500;

    private static final double TICKS_PER_REVOLUTION = 1440.0;
    private static final double MOTOR_POWER = 1.0;
    private static final double TIMEOUT_SECONDS = 5.0;

    private Telemetry telemetry;

    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shoulder = initializeMotor(hardwareMap, "shoulder", DcMotor.Direction.FORWARD);
        forearm = initializeMotor(hardwareMap, "forearm", DcMotor.Direction.REVERSE);
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    private DcMotor initializeMotor(HardwareMap hardwareMap, String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    public void moveArmToPosition(int shoulderTicks, int forearmTicks) {
        shoulderTicks = Math.max(SHOULDER_MIN_TICKS, Math.min(SHOULDER_MAX_TICKS, shoulderTicks));
        forearmTicks = Math.max(FOREARM_MIN_TICKS, Math.min(FOREARM_MAX_TICKS, forearmTicks));

        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks);
        forearm.setTargetPosition(forearm.getCurrentPosition() + forearmTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(MOTOR_POWER);
        forearm.setPower(MOTOR_POWER);

        waitForMotors(shoulder, forearm);

        stopMotors();
    }

    public void moveShoulderToPosition(int ticks) {
        moveArmToPosition(ticks, 0);
    }

    public void rotateForearmToAngle(int ticks) {
        moveArmToPosition(0, ticks);
    }

    public void openGripper() {
        leftClaw.setPosition(LEFT_CLAW_OPEN_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_OPEN_POSITION);
        telemetry.addData("Gripper", "Opened");
        telemetry.update();
    }

    public void closeGripper() {
        leftClaw.setPosition(LEFT_CLAW_CLOSED_POSITION);
        rightClaw.setPosition(RIGHT_CLAW_CLOSED_POSITION);
        telemetry.addData("Gripper", "Closed");
        telemetry.update();
    }

    public void resetArmPosition() {
        moveShoulderToPosition(0);
        rotateForearmToAngle(0);
    }

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

            telemetry.addData("Shoulder Position", shoulder.getCurrentPosition());
            telemetry.addData("Forearm Position", forearm.getCurrentPosition());
            telemetry.update();
        }
    }

    private void stopMotors() {
        shoulder.setPower(0);
        forearm.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();
    }
}
