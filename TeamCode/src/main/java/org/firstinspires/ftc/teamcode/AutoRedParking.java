package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedParking", group = "Competition")
public class AutoRedParking extends LinearOpMode {

    private RobotHardware robotHardware;
    private ElapsedTime runtime;

    // Distance to Red Parking Zone in inches
    private static final double RED_PARK_DISTANCE = 28;

    @Override
    public void runOpMode() {
        // Initialize hardware and runtime
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Parking in Red Zone");
            telemetry.update();

            parkInRedZone();

            telemetry.addData("Status", "Parking Complete");
            telemetry.update();
        }
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
        turnRight(90);
        robotHardware.forwardForDistance(inches);
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
    private void turnRight(int degrees) {
        robotHardware.turn(degrees, true);
        sleep(500); // Allow time for the turn to complete
    }

    private void turnLeft(int degrees) {
        robotHardware.turn(degrees, false);
        sleep(500);
    }
}
