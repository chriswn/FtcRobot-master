package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV Autonomous")
public class OpenCVAutonomous extends LinearOpMode {
    private ObjectDetection objectDetection;
    private Tracking tracking;

    @Override
    public void runOpMode() {
        // Initialize object detection system and tracking system
        objectDetection = new ObjectDetection(hardwareMap, telemetry);
        tracking = new Tracking(hardwareMap, telemetry);

        // Set up telemetry to display data to both the Driver Station and the FtcDashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start streaming camera feed to dashboard
        dashboard.startCameraStream(objectDetection.getCamera(), 30);

        // Wait for the start of the autonomous period
        waitForStart();

        // Autonomous loop
        while (opModeIsActive()) {
            // Get the detected object width and distance
            double detectedWidth = objectDetection.getPipeline().getWidth();
            double distance = objectDetection.getPipeline().getDistance();

            // If an object is detected (width > 0), process its data
            if (detectedWidth > 0) {
                telemetry.addData("Distance to object (inches)", distance);
                telemetry.addData("Object Width (pixels)", detectedWidth);
                telemetry.update();

                // Move towards the object
                tracking.moveTowardsObject(distance);
            } else {
                // If no object detected, search for the object
                telemetry.addData("Status", "No object detected");
                telemetry.update();
                tracking.searchForObject();  // Start searching if no object is detected
            }

            // Optionally, you can add a small sleep to reduce the telemetry update rate
            sleep(100); // Sleep to allow telemetry to refresh
        }

        // Stop robot movement after the autonomous period ends
        tracking.stopMovement();
    }
}
