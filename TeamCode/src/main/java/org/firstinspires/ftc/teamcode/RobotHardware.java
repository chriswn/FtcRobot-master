package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    private DcMotor backLeftMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;

    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private Telemetry telemetry;
    private double wheelCircumference;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeMotors(hardwareMap);
        wheelCircumference = Math.PI * WHEEL_DIAMETER;
    }

    private void initializeMotors(HardwareMap hardwareMap) {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void forwardForDistance(double inches) {
        resetEncoders();
        int ticks = calculateTicks(inches);
        setTargetPositions(ticks);
        runMotorsToPosition(0.5);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 30) {
            telemetry.addData("Motor Positions", "Left: %d, Right: %d",
                    frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    private int calculateTicks(double inches) {
        double rotations = inches / wheelCircumference;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    private void setTargetPositions(int ticks) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + ticks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + ticks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + ticks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + ticks);
    }

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

    public void turn(int degrees, boolean clockwise) {
        double wheelBase = 16.0; // inches between wheels
        double turnCircumference = Math.PI * wheelBase;
        double ticksPerDegree = (TICKS_PER_REVOLUTION / wheelCircumference) * turnCircumference / 360.0;
        int ticks = (int) (ticksPerDegree * degrees);

        if (clockwise) {
            setTargetPositionsForTurn(ticks, -ticks);
        } else {
            setTargetPositionsForTurn(-ticks, ticks);
        }

        runMotorsToPosition(0.5);

        while (motorsBusy()) {
            telemetry.addData("Turning", "In Progress...");
            telemetry.update();
        }

        stopMotors();
    }

    private void setTargetPositionsForTurn(int leftTicks, int rightTicks) {
        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + leftTicks);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + leftTicks);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + rightTicks);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + rightTicks);
    }

    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean motorsBusy() {
        return frontLeftMotor.isBusy() || backLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy();
    }

    public void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
