package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "shoulder", group = "Competition")
public class shoulder extends LinearOpMode {
    private DcMotor shoulder;
    private static final double TICKS_PER_REVOLUTION = 1440.0;
    private static final double MOTOR_POWER = 0;           // Motor power level

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        shoulder.setDirection(DcMotor.Direction.FORWARD); // Reverse direction if needed
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoders
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder to track position

        // Wait for the start of the autonomous period
        waitForStart();

        // Move shoulder to a specific position
        moveShoulderToPosition(3000);

        // Add any additional operations after the movement
    }

    // Method to move the shoulder to a specific position in encoder ticks
    public void moveShoulderToPosition(int shoulderTicks) {
        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks); // Set target position
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set motor mode to run to position
        shoulder.setPower(MOTOR_POWER); // Set motor power to move to the target position

        // Wait for the shoulder to reach the target position
        while (opModeIsActive() && shoulder.isBusy()) {
            telemetry.addData("Target", shoulder.getTargetPosition());
            telemetry.addData("Current", shoulder.getCurrentPosition());
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
