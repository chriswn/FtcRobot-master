package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue Team Auto", group = "Autonomous")
public class BlueTeamAuto extends LinearOpMode {

    private ObjectDetection objectDetection;
    private ArmMovement armMovement;
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap, telemetry);
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double objectCentroidX = objectDetection.getPipeline().getCentroidX();
                double objectCentroidY = objectDetection.getPipeline().getCentroidY();
                double objectWidth = objectDetection.getPipeline().getWidth();
                double objectDistance = objectDetection.getPipeline().getDistance();

                telemetry.addData("Object Centroid X", objectCentroidX);
                telemetry.addData("Object Centroid Y", objectCentroidY);
                telemetry.addData("Object Width (pixels)", objectWidth);
                telemetry.addData("Object Distance", objectDistance);
                telemetry.update();

                // If blue object is detected, proceed with grabbing and scoring
                if (objectWidth > 0) {
                    armMovement.closeGripper(); // Close gripper to grab the object
                    armMovement.move(12); // Move to scoring position (adjust distance as needed)
                    armMovement.openGripper(); // Release the object into the scoring basket
                    telemetry.addData("Status", "Scoring completed");
                    telemetry.update();
                    break;
                } else {
                    // Search for the object
                    telemetry.addData("Status", "Searching for Blue Object");
                    telemetry.update();
                    robot.setMotorPower(0.3, -0.3); // Turn to search for the object
                }

                sleep(100); // Sleep to prevent overloading the system with updates
            }
        }
    }
}
