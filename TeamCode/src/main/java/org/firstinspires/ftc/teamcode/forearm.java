package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "forearm", group = "Competition")
public class forearm extends LinearOpMode {
    private DcMotor forearm;
    private static final double TICKS_PER_REVOLUTION = 1440.0;
    private static final double MOTOR_POWER = 5;           // Motor power level

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        forearm = hardwareMap.get(DcMotor.class, "forearm");
        forearm.setDirection(DcMotor.Direction.FORWARD); // Reverse direction if needed
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoders
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder to track position

        // Wait for the start of the autonomous period
        waitForStart();

        while (opModeIsActive() && forearm.isBusy()) {
            telemetry.addData("Target", forearm.getTargetPosition());
            telemetry.addData("Current", forearm.getCurrentPosition());
            telemetry.update();
        }

        // Move shoulder to a specific position
        moveShoulderToPosition(-264);

        // Add any additional operations after the movement
    }

    // Method to move the shoulder to a specific position in encoder ticks
    public void moveShoulderToPosition(int shoulderTicks) {
        forearm.setTargetPosition(forearm.getCurrentPosition() + shoulderTicks); // Set target position
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set motor mode to run to position
        forearm.setPower(MOTOR_POWER); // Set motor power to move to the target position

        // Wait for the shoulder to reach the target position
        while (opModeIsActive() && forearm.isBusy()) {
            telemetry.addData("Target", forearm.getTargetPosition());
            telemetry.addData("Current", forearm.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor once the target is reached
        // stopMotors();
    }

    // Method to stop the motors
    // public void stopMotors() {
    //   shoulder.setPower(0); // Stop the shoulder motor
    //  }
}
