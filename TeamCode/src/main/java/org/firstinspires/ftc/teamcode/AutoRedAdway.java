package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedAdway", group = "Competition")
public class AutoRedAdway extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement;
    private ElapsedTime runtime;

    // Calibrated positions for specific tasks
    private static final int LOW_BASKET_SHOULDER_TICKS = 2374;
    private static final int LOW_BASKET_FOREARM_TICKS = -500;
    private static final int PICKUP_SHOULDER_TICKS = 0;
    private static final int PICKUP_FOREARM_TICKS = -300;

    @Override
    public void runOpMode() {
        robotHardware = new RobotHardware(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        preloadSetup();  // Preload the sample before match start
        armMovement.moveArmToPosition(100, 500);  // Adjust the second value as appropriate
        sleep(500); // Allow some time for the arm adjustment
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Starting Autonomous...");
            telemetry.update();
            performAutonomousRoutine();
        }

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void preloadSetup() {
        telemetry.addData("Status", "Preloading Sample...");
        telemetry.update();
        armMovement.closeGripper();  // Secure preloaded sample
        sleep(500); // Wait for the gripper to fully close
        armMovement.resetArmPosition();  // Set arm to transport position
        sleep(500); // Allow time for the arm to move
        telemetry.addData("Status", "Preload Setup Complete");
        telemetry.update();
    }

    private void performAutonomousRoutine() {
        // Step 1: Deliver Preloaded Sample to Low Basket
        deliverSampleToBasket();

        // Step 2: Attempt to pick up another sample if time allows
        if (runtime.seconds() < 20) {
            pickUpSample();
            deliverSampleToBasket();
        }

        // Step 3: Park in Observation Zone
        if (runtime.seconds() + 10 <= 30) {
            parkInObservationZone();
        }
    }

    private void deliverSampleToBasket() {
        telemetry.addData("Status", "Delivering Sample to Basket...");
        telemetry.update();

        moveToPosition(35);  // Shortened forward movement
        turnRight(90);       // Align directly with Low Basket
        moveToPosition(25);  // Move closer to basket
        sleep(500);

        armMovement.moveArmToPosition(LOW_BASKET_SHOULDER_TICKS, LOW_BASKET_FOREARM_TICKS);
        sleep(500);
        armMovement.openGripper();  // Drop the sample
        sleep(500);
        armMovement.resetArmPosition();
        sleep(500);
    }

    private void pickUpSample() {
        telemetry.addData("Status", "Picking up Sample...");
        telemetry.update();
        armMovement.moveArmToPosition(PICKUP_SHOULDER_TICKS, PICKUP_FOREARM_TICKS);
        sleep(500);
        armMovement.closeGripper();
        sleep(500);
        armMovement.resetArmPosition();
        sleep(500);
    }

    private void moveToPosition(double inches) {
        robotHardware.forwardForDistance(inches);
        sleep(500); // Wait for robot to reach position
    }

    private void turnRight(int degrees) {
        robotHardware.turn(degrees, true);
        sleep(500); // Allow time for the turn to complete
    }

    private void turnLeft(int degrees) {
        robotHardware.turn(degrees, false);
        sleep(500);
    }

    private void parkInObservationZone() {
        telemetry.addData("Status", "Parking in Observation Zone...");
        telemetry.update();

        moveToPosition(-15);  // Reverse
        turnLeft(90);         // Turn toward observation zone
        moveToPosition(24);   // Move forward to park
        sleep(500);
    }

    private void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            telemetry.addData("Error", "Sleep Interrupted");
            telemetry.update();
        }
    }
}
