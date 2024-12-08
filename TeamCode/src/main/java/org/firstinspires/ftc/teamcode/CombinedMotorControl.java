package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "combinedMotorControl", group = "Competition")
public class CombinedMotorControl extends LinearOpMode {

    private DcMotor shoulder;
    private DcMotor forearm;
    private static final double TICKS_PER_REVOLUTION = 1440.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware for both motors
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        forearm = hardwareMap.get(DcMotor.class, "forearm");

        // Set initial direction and modes for both motors
        shoulder.setDirection(DcMotor.Direction.FORWARD);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forearm.setDirection(DcMotor.Direction.FORWARD);
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the start signal
        waitForStart();
        moveMotorToPosition(shoulder, 167, 2);  // Move the shoulder

        // Example of moving both shoulder and forearm to their respective positions
        telemetry.addData("Starting motor movement", "");
        telemetry.update();

        moveMotorToPosition(shoulder, 2374, 2);  // Move the shoulder
        moveMotorToPosition(forearm, -300, 5);   // Move the forearm

        telemetry.addData("Movement completed", "");
        telemetry.update();
    }

    /**
     * Generalized method to move any motor to a specific encoder position.
     * @param motor Motor to move
     * @param targetPosition Target encoder tick position
     * @param power Power level for the movement
     */
    private void moveMotorToPosition(DcMotor motor, int targetPosition, double power) {
        motor.setTargetPosition(motor.getCurrentPosition() + targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

        while (opModeIsActive() && motor.isBusy()) {
            telemetry.addData("Motor Target", motor.getTargetPosition());
            telemetry.addData("Motor Current", motor.getCurrentPosition());
            telemetry.update();
        }

        motor.setPower(0); // Stop the motor when the target is reached
    }
}
