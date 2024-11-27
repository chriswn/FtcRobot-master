package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Team Auto", group = "Autonomous")
public class BlueTeamAuto extends LinearOpMode {

    private ObjectDetection objectDetection;
    private ArmMovement armMovement;
    private RobotHardware robot;

    @Override
    public void runOpMode() {
        // Initialize hardware and subsystems
        robot = new RobotHardware(hardwareMap, telemetry);
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            // Start the object detection process
            objectDetection.getCamera().startStreaming(ObjectDetection.CAMERA_WIDTH, ObjectDetection.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

            while (opModeIsActive()) {
                // Retrieve object detection results from the pipeline
                double objectCentroidX = objectDetection.getPipeline().cX;
                double objectCentroidY = objectDetection.getPipeline().cY;
                double objectWidth = objectDetection.getPipeline().width;

                telemetry.addData("Object Centroid X", objectCentroidX);
                telemetry.addData("Object Centroid Y", objectCentroidY);
                telemetry.addData("Object Width (pixels)", objectWidth);
                telemetry.update();

                // If object is detected (width > 0), proceed with grabbing and scoring
                if (objectWidth > 0) {
                    armMovement.closeGripper(); // Close gripper to grab the object
                    armMovement.move(12); // Move to the scoring position (adjust distance as needed)
                    armMovement.openGripper(); // Release the object into the scoring basket
                    telemetry.addData("Status", "Scoring completed");
                    telemetry.update();
                    break; // Exit the loop after scoring
                } else {
                    // If no object detected, search for the object
                    telemetry.addData("Status", "Searching for Blue Object");
                    telemetry.update();

                    // Simple turning to search for the object
                    robot.setMotorPower(0.3, -0.3); // Turn at a low speed to search
                    sleep(100); // Sleep to prevent overwhelming the control loop
                    robot.setMotorPower(0, 0); // Stop turning after one cycle

                    // Add logic to adjust robot motion, such as using sensor feedback or controlled turns
                }
            }
        }
    }
}
