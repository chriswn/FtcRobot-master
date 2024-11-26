package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Object Detection Example", group="Autonomous")
public class ObjectDetectionauto extends LinearOpMode {

    private ObjectDetection objectDetection;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the object detection system before waitForStart() to avoid delay
        objectDetection = new ObjectDetection(hardwareMap, telemetry);

        // Initialize the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start streaming the camera feed to the dashboard
        dashboard.startCameraStream(objectDetection.getCamera(), 30);  // Stream at 30 fps

        // Wait for the start of the autonomous period
        waitForStart();

        // Start the opmode loop once the robot starts
        while (opModeIsActive()) {
            // Get the object detection data
            double distance = objectDetection.getPipeline().getDistance();
            double centroidX = objectDetection.getPipeline().getCentroidX();
            double centroidY = objectDetection.getPipeline().getCentroidY();
            double objectWidth = objectDetection.getPipeline().getWidth();

            // Display the data on the driver station telemetry
            telemetry.addData("Object Distance (inches)", distance);
            telemetry.addData("Centroid X", centroidX);
            telemetry.addData("Centroid Y", centroidY);
            telemetry.addData("Object Width (pixels)", objectWidth);
            telemetry.update();  // Update telemetry data for the driver station

            // Sleep briefly to prevent overwhelming the system (100ms is a good delay)
            sleep(100);
        }
    }
}
