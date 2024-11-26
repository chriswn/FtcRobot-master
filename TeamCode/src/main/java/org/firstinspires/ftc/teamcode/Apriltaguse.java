package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;

public class Apriltaguse {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private final HashMap<Integer, String> arenaCorners = new HashMap<>();
    private final Telemetry telemetry;

    // Constructor
    public Apriltaguse(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeVision(hardwareMap);
    }

    // Initialize the vision pipeline
    private void initializeVision(HardwareMap hardwareMap) {
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Ensure "Webcam 1" matches the config
                    .addProcessor(aprilTagProcessor)
                    .build();

            // Define corner tags (adjust IDs based on your game setup)
            arenaCorners.put(11, "Top Left");
            arenaCorners.put(12, "Top Right");
            arenaCorners.put(13, "Bottom Left");
            arenaCorners.put(14, "Bottom Right");

            telemetry.addLine("Vision initialized successfully.");
        } catch (Exception e) {
            telemetry.addLine("Error initializing vision: " + e.getMessage());
        }
        telemetry.update();
    }

    // Retrieve the list of detected tags
    public List<AprilTagDetection> getDetectedTags() {
        return aprilTagProcessor.getDetections();
    }

    // Process detected tags to identify arena corners
    public void findArenaCorners() {
        if (visionPortal == null || aprilTagProcessor == null) {
            telemetry.addLine("Vision system not initialized.");
            telemetry.update();
            return;
        }

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("No tags detected.");
            telemetry.update();
            return;
        }

        for (AprilTagDetection tag : detections) {
            String cornerName = arenaCorners.get(tag.id);
            if (cornerName != null) {
                displayTagInfo(tag, cornerName);
            } else {
                telemetry.addLine(String.format("Tag ID: %d detected, not a known corner.", tag.id));
            }
        }
        telemetry.update();
    }

    // Display tag information in telemetry
    private void displayTagInfo(AprilTagDetection tag, String cornerName) {
        telemetry.addLine(String.format("Tag ID: %d (%s)", tag.id, cornerName));

        if (tag.ftcPose != null) {
            telemetry.addLine(String.format("Position (XYZ): %.1f, %.1f, %.1f inches",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            telemetry.addLine(String.format("Rotation (PRY): %.1f, %.1f, %.1f degrees",
                    tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            telemetry.addLine(String.format("Range: %.1f inches", tag.ftcPose.range));
            telemetry.addLine(String.format("Bearing: %.1f degrees", tag.ftcPose.bearing));
            telemetry.addLine(String.format("Elevation: %.1f degrees", tag.ftcPose.elevation));
        } else {
            telemetry.addLine("Pose data unavailable for this tag.");
        }
    }

    // Safely close the vision portal
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            telemetry.addLine("Vision portal closed.");
            telemetry.update();
        }
    }
}
