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

    private static final double LEFT_CLAW_OPEN_POSITION = 0.8;
    private static final double LEFT_CLAW_CLOSED_POSITION = 0.1;
    private static final double RIGHT_CLAW_OPEN_POSITION = 0.3;
    private static final double RIGHT_CLAW_CLOSED_POSITION = 1.0;

    private static final int SHOULDER_MIN_TICKS = -500;
    private static final int SHOULDER_MAX_TICKS = 3000;
    private static final int FOREARM_MIN_TICKS = -500;
    private static final int FOREARM_MAX_TICKS = 500;

    private static final double TIMEOUT_SECONDS = 5.0;

    private Telemetry telemetry;

    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shoulder = initializeMotor(hardwareMap, "shoulder", DcMotor.Direction.FORWARD);
        forearm = initializeMotor(hardwareMap, "forearm", DcMotor.Direction.FORWARD);
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
        // Pre-move calibration to address play
        //calibrateMotor(forearm, 50);

        shoulderTicks = Math.max(SHOULDER_MIN_TICKS, Math.min(SHOULDER_MAX_TICKS, shoulderTicks));
        forearmTicks = Math.max(FOREARM_MIN_TICKS, Math.min(FOREARM_MAX_TICKS, forearmTicks));

        shoulder.setTargetPosition(shoulderTicks);
        forearm.setTargetPosition(forearmTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(0.8);
        forearm.setPower(1);

        waitForMotors(shoulder, forearm);
       // retryIfNotReached(shoulder, shoulderTicks);
       // retryIfNotReached(forearm, forearmTicks);

        stopMotors();
    }


    private void calibrateMotor(DcMotor motor, int reverseTicks) {
        motor.setTargetPosition(motor.getCurrentPosition() - reverseTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.3);
        while (motor.isBusy()) {
            telemetry.addData("Calibration", "Calibrating %s", motor.getDeviceName());
            telemetry.update();
        }
    }

    private void retryIfNotReached(DcMotor motor, int targetTicks) {
        if (Math.abs(motor.getCurrentPosition() - targetTicks) > 10) {
            telemetry.addData("Retry", "Retrying %s to reach target", motor.getDeviceName());
            telemetry.update();
            motor.setTargetPosition(targetTicks);
            motor.setPower(1.0);
            waitForMotors(motor);
        }
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
        telemetry.addData("Status", "Resetting Arm Position...");
        telemetry.update();

        moveArmToPosition(0, 0);  // Move the arm to a neutral position
        telemetry.addData("Status", "Arm Reset Complete");
        telemetry.update();
    }

    private void waitForMotors(DcMotor... motors) {
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < TIMEOUT_SECONDS) {
            boolean allIdle = true;
            for (DcMotor motor : motors) {
                if (motor.isBusy()) {
                    allIdle = false;
                    telemetry.addData("Motor", "%s busy at %d ticks", motor.getDeviceName(), motor.getCurrentPosition());
                }
            }
            if (allIdle) break;
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
