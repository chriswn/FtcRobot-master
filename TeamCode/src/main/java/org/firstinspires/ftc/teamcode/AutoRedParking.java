package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedParking", group = "Competition")
public class AutoRedParking extends LinearOpMode {

    private RobotHardware robotHardware;

    // Distance to Red Parking Zone in inches
    private static final double RED_PARK_DISTANCE = 24;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);

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
