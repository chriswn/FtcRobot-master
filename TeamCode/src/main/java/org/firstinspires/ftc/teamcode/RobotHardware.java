package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    // Declare motors and servos
    private DcMotor backLeftMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontRightMotor = null;
    private  DcMotor Shoulder = null;
    private DcMotor Forearm = null;
  //  private Servo servo1 = null;

    // Constants
    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double SERVO_OPEN_POSITION = 1.0; // Open position
    private static final double SERVO_CLOSED_POSITION = 0.0; // Closed position
    private static final double MOTOR_POWER = 0.5; // Default motor power

    private Telemetry telemetry;

    // Constructor to initialize hardware
    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
      //  servo1 = hardwareMap.get(Servo.class, "Servo1");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the servo position
      //  servo1.setPosition(SERVO_CLOSED_POSITION);
    }

    public void forwardForDistance(double inches) {
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
            telemetry.addData("backLeftMotor", backLeftMotor.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

   /* public void openServo() {
        servo1.setPosition(SERVO_OPEN_POSITION);
    }
*/
    // Encoder and motor control methods
    private void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int calculateTicks(double inches) {
        double wheelCircumference = WHEEL_DIAMETER * Math.PI;
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

    public void setMotorPower(double leftPower, double rightPower) {
        // Set power for left motors
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);

        // Set power for right motors
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
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
