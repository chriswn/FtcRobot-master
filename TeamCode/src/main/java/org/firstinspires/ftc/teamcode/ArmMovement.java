package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMovement {
    private  DcMotor forearm = null;
    private  DcMotor shoulder = null;


    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double SERVO_OPEN_POSITION = 1.0; // Open position
    private static final double SERVO_CLOSED_POSITION = 0.0; // Closed position
    private static final double MOTOR_POWER = 0.5; // Default motor power

    private Telemetry telemetry;

public ArmMovement(HardwareMap hardwareMap,Telemetry telemetry ){
    this. telemetry= telemetry;
    forearm = hardwareMap. get(DcMotor.class, "Forearm");
    shoulder = hardwareMap. get(DcMotor.class, "shoulder");

    forearm.setDirection(DcMotor.Direction.REVERSE);
    shoulder.setDirection(DcMotor.Direction.REVERSE);
}

    public void move (double inches) {
        resetEncoders();

        // Calculate ticks needed for the desired distance
        int ticks = calculateTicks(inches);

        // Set target positions
        setTargetPositions(ticks);

        // Set motors to run to position
        runMotorsToPosition(MOTOR_POWER);

        // Wait until motors finish their movement
        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 30) {
            telemetry.addData("forearm", forearm.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
}

    private void resetEncoders() {
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int calculateTicks(double inches) {
        double wheelCircumference = WHEEL_DIAMETER * Math.PI;
        double rotations = inches / wheelCircumference;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    private void setTargetPositions(int ticks) {
        forearm.setTargetPosition(forearm.getCurrentPosition() + ticks);
        shoulder.setTargetPosition(shoulder.getCurrentPosition() + ticks);

    }
    private void runMotorsToPosition(double motorPower) {
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMotorPower(double leftPower, double rightPower) {
            // Set power for left motors
            forearm.setPower(leftPower);
            shoulder.setPower(leftPower);
    }

    private boolean motorsBusy() {
        return forearm.isBusy() || shoulder.isBusy();
    }

    private void stopMotors() {
            forearm.setPower(0);
            shoulder.setPower(0);
        }

}