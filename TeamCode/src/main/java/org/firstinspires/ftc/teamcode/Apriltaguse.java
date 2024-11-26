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
    private HashMap<Integer, String> arenaCorners = new HashMap<>();
    private Telemetry telemetry;

    // Constructor accepts HardwareMap and Telemetry
    public Apriltaguse(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initializeVision(hardwareMap);
    }
    // Add this method to the Apriltaguse class to get the list of detected tags
    public List<AprilTagDetection> getDetectedTags() {
        return aprilTagProcessor.getDetections();
    }

    // Initialize the vision processing pipeline
    void initializeVision(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Ensure the camera name is correct
                .addProcessor(aprilTagProcessor)
                .build();

        // Define known corner tags
        arenaCorners.put(1, "Top Left");
        arenaCorners.put(2, "Top Right");
        arenaCorners.put(3, "Bottom Left");
        arenaCorners.put(4, "Bottom Right");
    }

    // Find arena corners based on detected tags
    public void findArenaCorners() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // If no tags are detected, just show a message and stop processing for now
        if (detections.isEmpty()) {
            telemetry.addData("Tags Detected", 0);
            telemetry.update();
            return;
        }

        // Process each detected tag
        for (AprilTagDetection tag : detections) {
            // Check if the detected tag ID corresponds to one of the known corners
            String cornerName = arenaCorners.get(tag.id);
            if (cornerName != null) {
                displayTagInfo(tag, cornerName);  // Display tag info if it's a corner tag
            } else {
                telemetry.addLine(String.format("Tag ID: %d (Not a corner tag)", tag.id));
                telemetry.update();
            }
        }
    }

    // Display tag information (ID, position, rotation)
    private void displayTagInfo(AprilTagDetection tag, String cornerName) {
        telemetry.addLine(String.format("Tag ID: %d (%s)", tag.id, cornerName));

        // If pose data is available, display it
        if (tag.ftcPose != null) {
            telemetry.addLine(String.format("Position (XYZ): %.1f, %.1f, %.1f inches",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
            telemetry.addLine(String.format("Rotation (PRY): %.1f, %.1f, %.1f degrees",
                    tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
            telemetry.addLine(String.format("Range: %.1f inches", tag.ftcPose.range));
            telemetry.addLine(String.format("Bearing: %.1f degrees", tag.ftcPose.bearing));
            telemetry.addLine(String.format("Elevation: %.1f degrees", tag.ftcPose.elevation));
        } else {
            telemetry.addLine("Pose data not available.");
        }
        telemetry.update();
    }

    // Close vision processing portal
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();  // Close VisionPortal when done
        }
    }
}


