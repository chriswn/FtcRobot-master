package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedPreload", group = "Competition")
public class AutoRedPreload extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement;
    private ElapsedTime runtime;

    private static final int PRELOAD_ARM_SHOULDER_TICKS = 1000; // Example position
    private static final int PRELOAD_ARM_FOREARM_TICKS = -200;  // Example position
    private static final double RED_PARK_DISTANCE = 24;         // Distance to park in the Red Zone

    @Override
    public void runOpMode() {
        // Initialize hardware and components
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Preloading Sample");
            telemetry.update();
            preloadSample();

            telemetry.addData("Status", "Starting Parking Routine");
            telemetry.update();
            parkInRedZone();

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    /**
     * Logic for preloading a sample.
     */
    private void preloadSample() {
        // Move robot to preloading position
        moveToPosition(35);
        turnRight(90); // Turn to face the basket
        moveToPosition(25);

        // Handle arm movement and gripping
        armMovement.moveArmToPosition(PRELOAD_ARM_SHOULDER_TICKS, PRELOAD_ARM_FOREARM_TICKS);
        armMovement.closeGripper(); // Secure the sample
        sleepNonBlocking(500);

        armMovement.resetArmPosition(); // Prepare arm for transport
        sleepNonBlocking(500);
    }

    /**
     * Logic for parking in the Red Alliance zone.
     */
    private void parkInRedZone() {
        moveToPosition(RED_PARK_DISTANCE);
        telemetry.addData("Action", "Parked in Red Zone");
        telemetry.update();
    }

    /**
     * Moves the robot a specified distance forward.
     *
     * @param inches Distance to move in inches.
     */
    private void moveToPosition(double inches) {
        robotHardware.forwardForDistance(inches);
        sleepNonBlocking(500);
    }

    /**
     * Turns the robot to the right.
     *
     * @param degrees Degrees to turn.
     */
    private void turnRight(int degrees) {
        robotHardware.turn(degrees, true);
        sleepNonBlocking(500);
    }

    /**
     * Turns the robot to the left.
     *
     * @param degrees Degrees to turn.
     */
    private void turnLeft(int degrees) {
        robotHardware.turn(degrees, false);
        sleepNonBlocking(500);
    }

    /**
     * Adds a non-blocking delay.
     *
     * @param milliseconds Delay duration.
     */
    private void sleepNonBlocking(int milliseconds) {
        double startTime = runtime.milliseconds();
        while (opModeIsActive() && runtime.milliseconds() - startTime < milliseconds) {
            idle();
        }
    }
}
