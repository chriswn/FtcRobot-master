package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "AutoBlueParking", group = "Competition")
public class AutoBlueParking extends LinearOpMode {

    private RobotHardware robotHardware;

    // Distance to Blue Parking Zone in inches
    private static final double BLUE_PARK_DISTANCE = 24;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Parking in Blue Zone");
            telemetry.update();

            parkInBlueZone();

            telemetry.addData("Status", "Parking Complete");
            telemetry.update();
        }
    }

    /**
     * Logic for parking in the Blue Alliance zone.
     */
    private void parkInBlueZone() {
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
