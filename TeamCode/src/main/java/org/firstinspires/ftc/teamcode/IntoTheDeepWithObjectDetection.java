package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "IntoTheDeepWithObjectDetection")
public class IntoTheDeepWithObjectDetection extends LinearOpMode {

    private RobotHardware robot;
    private ObjectDetection objectDetection;
    private ArmMovement armMovement;
    private Apriltaguse apriltaguse;

    // Camera variables
    private static final double IMAGE_CENTER_X = 320; // Camera resolution width / 2
    private static final double TAG_THRESHOLD = 50; // Threshold to move the robot based on the tag's position
    private static final double DISTANCE_THRESHOLD = 12; // Distance from the tag to move forward or stop

    @Override
    public void runOpMode() {
        // Initialize hardware and vision components
        robot = new RobotHardware(hardwareMap, telemetry);
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        armMovement = new ArmMovement(hardwareMap, telemetry);
        apriltaguse = new Apriltaguse(hardwareMap, telemetry); // Correct instantiation

        // Initialize FtcDashboard for real-time monitoring
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start Vision Portal stream (disabled live view, so this is safe)
        dashboard.startCameraStream(objectDetection.getCamera(), 30); // Stream the camera from ObjectDetection

        // Wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            // Perform object detection
            while (opModeIsActive()) {
                // Get data from object detection pipeline
                double objectCentroidX = objectDetection.getPipeline().getCentroidX();
                double objectCentroidY = objectDetection.getPipeline().getCentroidY();
                double objectDistance = objectDetection.getPipeline().getDistance();
                double objectWidth = objectDetection.getPipeline().getWidth();

                // Display object detection info in telemetry
                telemetry.addData("Object Centroid X", objectCentroidX);
                telemetry.addData("Object Centroid Y", objectCentroidY);
                telemetry.addData("Object Distance (inches)", objectDistance);
                telemetry.addData("Object Width (pixels)", objectWidth);
                telemetry.update();

                // Behavior based on object distance
                if (objectDistance < DISTANCE_THRESHOLD) {
                    telemetry.addData("Status", "Object too close! Stopping.");
                    telemetry.update();
                    robot.stopMotors(); // Stop moving if too close
                    sleep(1000); // Wait for a second
                    break; // End loop if object is too close
                } else {
                    // Move forward towards the object
                    telemetry.addData("Status", "Moving forward.");
                    telemetry.update();
                    robot.setMotorPower(0.5, 0.5); // Move forward at 50% power
                }

                // If no object is detected, search for it
                if (objectWidth == 0) { // No object detected
                    telemetry.addData("Status", "Searching for object.");
                    telemetry.update();
                    robot.setMotorPower(0.3, -0.3); // Turn slowly to search
                }

                // Sleep for a short time before the next loop iteration
                sleep(100); // Adjust sleep time as needed
            }

            // Once the object is detected, move the arm to interact with the object
            armMovement.closeGripper(); // Close the gripper to pick up the object
            armMovement.move(10); // Example distance in inches to move the arm

            // After moving, release the object into the basket (drop it)
            armMovement.openGripper(); // Open the gripper to drop the object

            // After object detection and arm movement, start AprilTag detection to score
            processAprilTags();
        }

        // Close vision systems
        if (objectDetection.getCamera() != null) {
            objectDetection.getCamera().closeCameraDevice();
        }
        if (apriltaguse != null) {
            apriltaguse.close();
        }
    }

    // Process AprilTags once the object detection loop ends
    private void processAprilTags() {
        // Get detected AprilTags
        List<AprilTagDetection> aprilTags = apriltaguse.getDetectedTags(); // Use the new method to get detected tags

        if (aprilTags != null && !aprilTags.isEmpty()) {
            // Process the first detected AprilTag
            AprilTagDetection targetTag = aprilTags.get(0); // Get the first detected tag
            telemetry.addData("AprilTag Detected", targetTag.id);
            telemetry.update();

            // Process the detected tag's position and move towards it
            moveTowardsAprilTag(targetTag);
        } else {
            telemetry.addData("AprilTag Status", "No AprilTags detected");
            telemetry.update();
        }
    }

    // Method to move robot towards the detected AprilTag
    private void moveTowardsAprilTag(AprilTagDetection targetTag) {
        // Assuming the tag pose contains translation and rotation (check your library for correct usage)
        double tagX = targetTag.ftcPose.x;  // Tag's position in X
        double tagY = targetTag.ftcPose.y;  // Tag's position in Y
        double tagZ = targetTag.ftcPose.z;  // Tag's position in Z

        // Adjust robot's movement based on tag's position
        if (Math.abs(tagX - IMAGE_CENTER_X) > TAG_THRESHOLD) {
            if (tagX < IMAGE_CENTER_X) {
                robot.setMotorPower(-0.3, 0.3);  // Turn left
                telemetry.addData("Turning", "Left");
            } else {
                robot.setMotorPower(0.3, -0.3);  // Turn right
                telemetry.addData("Turning", "Right");
            }
        } else {
            // Move forward towards the tag if it's aligned
            double moveSpeed = 0.5;
            if (tagZ > DISTANCE_THRESHOLD) {
                moveSpeed = 0.8;  // Speed up to approach the tag
            }
            robot.setMotorPower(moveSpeed, moveSpeed);  // Move forward
            telemetry.addData("Moving", "Forward");
        }

        telemetry.update();

        // If close enough to the tag, stop the robot
        if (tagZ < 10) {
            robot.stopMotors();
            telemetry.addData("Arrived", "At the AprilTag!");
            telemetry.update();
        }
    }
}
