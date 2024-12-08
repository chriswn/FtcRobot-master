package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoBluePreload", group = "Competition")
public class AutoBluePreload extends LinearOpMode {

    private RobotHardware robotHardware;
    private ArmMovement armMovement;
    private ElapsedTime runtime;

    private static final int PRELOAD_ARM_SHOULDER_TICKS = 1000;
    private static final int PRELOAD_ARM_FOREARM_TICKS = -200;
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
            preloadSample();

            telemetry.addData("Status", "Starting Parking Routine");
            telemetry.update();
            parkInBlueZone();

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    private void preloadSample() {
        robotHardware.forwardForDistance(35);
        robotHardware.turn(90, true);
        robotHardware.forwardForDistance(25);

        armMovement.moveArmToPosition(PRELOAD_ARM_SHOULDER_TICKS, PRELOAD_ARM_FOREARM_TICKS);
        armMovement.closeGripper();
        sleepNonBlocking(500);

        armMovement.resetArmPosition();
        sleepNonBlocking(500);
    }

    private void parkInBlueZone() {
        robotHardware.forwardForDistance(BLUE_PARK_DISTANCE);
        telemetry.addData("Action", "Parked in Blue Zone");
        telemetry.update();
    }

    private void sleepNonBlocking(int milliseconds) {
        double startTime = runtime.milliseconds();
        while (opModeIsActive() && runtime.milliseconds() - startTime < milliseconds) {
            idle();
        }
    }
}
