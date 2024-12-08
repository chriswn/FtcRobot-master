package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoParking", group = "Competition")
public class AutoParking extends LinearOpMode {

    private RobotHardware robotHardware;
    private ElapsedTime runtime;

    // Define parking positions for Red and Blue Alliances
    private static final double RED_PARK_DISTANCE = 24; // Distance to Red Parking Zone in inches
    private static final double BLUE_PARK_DISTANCE = 24; // Distance to Blue Parking Zone in inches

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Autonomous Running");
            telemetry.update();

            // Strategy: Determine alliance and park accordingly
            if (isRedAlliance()) {
                parkInRedZone();
            } else {
                parkInBlueZone();
            }

            telemetry.addData("Status", "Parking Complete");
            telemetry.update();
        }
    }

    /**
     * Determines if the robot is on the Red Alliance.
     * This could be updated with real-time input or configuration.
     */
    private boolean isRedAlliance() {
        // Example: Return true for Red Alliance, false for Blue Alliance
        // Replace with your method of alliance determination
        return true; // Default to Red Alliance
    }

    /**
     * Logic for parking in the Red Alliance zone.
     */
    private void parkInRedZone() {
        telemetry.addData("Alliance", "Red");
        telemetry.update();

        moveToPosition(RED_PARK_DISTANCE);
        telemetry.addData("Action", "Parked in Red Zone");
        telemetry.update();
    }

    /**
     * Logic for parking in the Blue Alliance zone.
     */
    private void parkInBlueZone() {
        telemetry.addData("Alliance", "Blue");
        telemetry.update();

        moveToPosition(BLUE_PARK_DISTANCE);
        telemetry.addData("Action", "Parked in Blue Zone");
        telemetry.update();
    }

    /**
     * Moves the robot a specified distance forward.
     * @param inches Distance to move in inches.
     */
    private void moveToPosition(double inches) {
        robotHardware.forwardForDistance(inches);
        sleep(500); // Allow time for the movement to complete
    }

    /**
     * Adds a delay in milliseconds.
     * @param milliseconds Delay duration.
     */
    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Sleep Interrupted");
            telemetry.update();
        }
    }
}
