package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedAdway", group = "Competition")
public class AutoRedAdway extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement;
    private ElapsedTime runtime;

    private static final int LOW_BASKET_TICKS = 600;  // Adjust based on your arm calibration
    private static final int HIGH_BASKET_TICKS = 1200;

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
            performAutonomousRoutine();
        }

        telemetry.addData("Status", "Autonomous Complete");
        telemetry.update();
    }

    private void performAutonomousRoutine() {
        // Step 1: Approach sample line
        moveToPosition(12);  // Distance to sample line

        // Step 2: Pick up sample
        pickUpSample();

        // Step 3: Deliver to Low Basket
        deliverSampleToBasket(LOW_BASKET_TICKS);

        // Step 4: Optional - Attempt another sample if time allows
        if (runtime.seconds() < 20) {
            attemptSecondSample(LOW_BASKET_TICKS);
        }

        // Step 5: Park in Observation Zone
        if (runtime.seconds() + 10 <= 30) {
            parkInObservationZone();
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

   // private void PreLoad{

   // }
    private void pickUpSample() {
        telemetry.addData("Status", "pickUpSample...");
        telemetry.update();
        armMovement.moveShoulderToPosition(100);  // rise shoulder to reach sample
        armMovement.rotateForearmToAngle(-300);       // Align forearm
        armMovement.closeGripper();                // Grab the sample
        armMovement.resetArmPosition();            // Reset to transport position
    }

    private void deliverSampleToBasket(int basketHeightTicks) {
        telemetry.addData("Status", "deliverSampleToBasket...");
        telemetry.update();
        moveToPosition(-10);
        turnRight(90);                             // Turn towards basket
        moveToPosition(36);                        // Approach basket
        armMovement.moveShoulderToPosition(basketHeightTicks);  // Raise shoulder to basket height
        armMovement.openGripper();                // Drop the sample
        armMovement.resetArmPosition();           // Reset arm
    }

    private void attemptSecondSample(int basketHeightTicks) {
        telemetry.addData("Status", "attemptSecondSample...");
        telemetry.update();
        moveToPosition(-12);                       // Move back to sample area
        pickUpSample();                            // Repeat pickup process
        deliverSampleToBasket(basketHeightTicks);  // Score second sample
    }

    private void parkInObservationZone() {
        telemetry.addData("Status", "parkInObservationZone...");
        telemetry.update();
        moveToPosition(-12);                       // Back away from basket
        turnLeft(90);                              // Align with Observation Zone
        moveToPosition(-36);                       // Park in zone
    }
}
