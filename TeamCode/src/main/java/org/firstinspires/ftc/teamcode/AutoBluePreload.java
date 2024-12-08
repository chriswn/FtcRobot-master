package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "AutoBluePreload", group = "Competition")
public class AutoBluePreload extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement; // Assume this class is provided to control the arm
    private ElapsedTime runtime;

    // Calibrated positions for preloading tasks
    private static final int PRELOAD_ARM_SHOULDER_TICKS = 1000; // Example position
    private static final int PRELOAD_ARM_FOREARM_TICKS = -200; // Example position
    private static final double BLUE_PARK_DISTANCE = 24;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Preloading Sample");
            telemetry.update();

            preloadSample();  // Preload the sample
            sleep(500);       // Allow some time for the arm adjustment

            telemetry.addData("Status", "Starting Parking Routine");
            telemetry.update();

            parkInBlueZone();  // Move to parking zone after preloading
        }

        telemetry.addData("Status", "Parking Complete");
        telemetry.update();
    }

    /**
     * Logic for preloading a sample.
     */
    private void preloadSample() {
        moveToPosition(35);  // Shortened forward movement
        turnRight(90);       // Align directly with Low Basket
        moveToPosition(25);  // Move closer to basket
        sleep(500);

        armMovement.moveArmToPosition(PRELOAD_ARM_SHOULDER_TICKS, PRELOAD_ARM_FOREARM_TICKS);
        armMovement.closeGripper();  // Secure preloaded sample
        sleep(500); // Wait for the gripper to fully close
        armMovement.resetArmPosition(); // Set arm to transport position
        sleep(500); // Allow time for the arm to move
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

    private void turnRight(int degrees) {
        robotHardware.turn(degrees, true);
        sleep(500); // Allow time for the turn to complete
    }

    private void turnLeft(int degrees) {
        robotHardware.turn(degrees, false);
        sleep(500);
}
