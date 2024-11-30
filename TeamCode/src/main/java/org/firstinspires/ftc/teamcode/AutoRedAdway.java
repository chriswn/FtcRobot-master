package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedAdway", group = "Competition")
public class AutoRedAdway extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        waitForStart();

        runtime.reset();

        if (opModeIsActive()) {
            // Move to sample line
            moveToPosition(24);

            // Pick up the sample using the claw
            armMovement.moveShoulderToPosition(-5); // Lower shoulder to reach the sample
            armMovement.rotateForearmToAngle(90); // Rotate forearm for alignment
            armMovement.closeGripper(); // Grab the sample
            sleep(1000);

            // Reset arm to neutral position for transport
            armMovement.rotateForearmToAngle(0); // Reset forearm to neutral
            armMovement.moveShoulderToPosition(0); // Move shoulder back to neutral

            // Deliver the sample to the lower basket
            turnRight(90);
            moveToPosition(12);
            armMovement.moveShoulderToPosition(25); // Raise shoulder to basket height
            sleep(1000);
            armMovement.openGripper(); // Drop the sample
            sleep(1000);

            // Optionally attempt to pick up and score a second sample
            if (runtime.seconds() < 20) {
                moveToPosition(-12);
                armMovement.moveShoulderToPosition(-5); // Lower shoulder for second sample
                armMovement.rotateForearmToAngle(90);
                armMovement.closeGripper();
                sleep(1000);

                armMovement.rotateForearmToAngle(0);
                armMovement.moveShoulderToPosition(25); // Raise shoulder for scoring
                moveToPosition(12);
                armMovement.openGripper();
                sleep(1000);
            }

            // Park in the designated zone if time is short
            if (runtime.seconds() >= 20 || runtime.seconds() + 10 > 30) {
                moveToPosition(-12);
                turnLeft(90);
                moveToPosition(-36);
            }

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    private void moveToPosition(double inches) {
        robotHardware.forwardForDistance(inches);
    }

    private void turnRight(int degrees) {
        robotHardware.turn(degrees, true);
    }

    private void turnLeft(int degrees) {
        robotHardware.turn(degrees, false);
    }
}
