package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;
import java.util.List;

public class Apriltaguse {

    private AprilTagProcessor aprilTagProcessor;
    private final HashMap<Integer, String> arenaCorners = new HashMap<>();
    private final Telemetry telemetry;
    private VisionPortal visionPortal;

    // Constructor
    public Apriltaguse(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeVision(hardwareMap);
    }

    // Retrieve the list of detected tags
    public List<AprilTagDetection> getDetectedTags() {
        if (aprilTagProcessor == null) {
            telemetry.addLine("Vision system not initialized.");
            telemetry.update();
            return null;
        }
        return aprilTagProcessor.getDetections();
    }

    // Initialize the vision processing pipeline
    private void initializeVision(HardwareMap hardwareMap) {
        try {
            // Initialize the AprilTagProcessor and set up the camera for detection
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

            // Initialize the vision portal with the webcam
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcamName)
                    .addProcessor(aprilTagProcessor)
                    .build();

            // Define known corner tags (adjust IDs based on your game setup)
            arenaCorners.put(11, "Top Left");
            arenaCorners.put(12, "Top Right");
            arenaCorners.put(13, "Bottom Left");
            arenaCorners.put(14, "Bottom Right");

            telemetry.addLine("AprilTag Vision initialized successfully.");
        } catch (Exception e) {
            telemetry.addLine("Error initializing AprilTag vision: " + e.getMessage());
        }
        telemetry.update();
    }

    // Process detected tags to identify arena corners
    public void findArenaCorners() {
        if (aprilTagProcessor == null) {
            telemetry.addLine("Vision system not initialized.");
            telemetry.update();
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections == null || detections.isEmpty()) {
            telemetry.addLine("No tags detected.");
            telemetry.update();
            return;
        }

        for (AprilTagDetection tag : detections) {
            String cornerName = arenaCorners.get(tag.id);
            if (cornerName != null) {
                displayTagInfo(tag, cornerName);
            } else {
                telemetry.addLine(String.format("Tag ID: %d detected but not mapped to a corner.", tag.id));
            }
        }
        telemetry.update();
    }

    // Display detailed information for detected tags
    private void displayTagInfo(AprilTagDetection tag, String cornerName) {
        telemetry.addLine(String.format("Corner Detected: %s", cornerName));
        telemetry.addLine(String.format("Tag ID: %d", tag.id));
        telemetry.addLine(String.format("Position (XYZ): %.1f, %.1f, %.1f (inches)",
                tag.robotPose.getPosition().x,
                tag.robotPose.getPosition().y,
                tag.robotPose.getPosition().z));
        telemetry.addLine(String.format("Orientation (PRY): %.1f, %.1f, %.1f (degrees)",
                tag.robotPose.getOrientation().getPitch(),
                tag.robotPose.getOrientation().getRoll(),
                tag.robotPose.getOrientation().getYaw()));
    }

    // Shutdown the vision portal when the robot is no longer active
    public void shutdownVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
