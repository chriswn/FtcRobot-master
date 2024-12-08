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
    private static final int LOW_BASKET_FOREARM_TICKS = 300;
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

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            performAutonomousRoutine();
        }

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void preloadSetup() {
        telemetry.addData("Status", "Preloading Sample...");
        telemetry.update();
        armMovement.closeGripper();  // Secure preloaded sample
        armMovement.resetArmPosition();  // Set arm to transport position
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

        moveToPosition(-8);                              // Short backward adjustment
        turnRight(90);                                    // Turn to align with basket
        moveToPosition(28);                               // Approach low basket
        armMovement.moveArmToPosition(LOW_BASKET_SHOULDER_TICKS, LOW_BASKET_FOREARM_TICKS);  // Adjust arm for Low Basket
        armMovement.openGripper();                       // Release sample
        armMovement.resetArmPosition();                  // Reset arm for safe navigation
    }

    private void pickUpSample() {
        telemetry.addData("Status", "Picking up Sample...");
        telemetry.update();
        armMovement.moveArmToPosition(PICKUP_SHOULDER_TICKS, PICKUP_FOREARM_TICKS);  // Align for pickup
        armMovement.closeGripper();              // Grab the sample
        armMovement.resetArmPosition();          // Reset to transport position
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

    private void parkInObservationZone() {
        telemetry.addData("Status", "Parking in Observation Zone...");
        telemetry.update();

        moveToPosition(-15);                       // Back away from basket
        turnLeft(90);                              // Align with observation zone
        moveToPosition(24);                        // Move into the observation zone
    }
}
