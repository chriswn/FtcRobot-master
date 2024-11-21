package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;

public class AprilTagDetection {

    // Real-world size of an AprilTag in meters (example)
    private static final double TAG_SIZE = 0.162; // Example: 16.2 cm (0.162 meters)

    // Define camera and pipeline
    private OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    // Initialize the camera and pipeline
    public void init(HardwareMap hardwareMap) {
        pipeline = new AprilTagDetectionPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error if camera fails to open
                System.out.println("Camera error: " + errorCode);
            }
        });
    }

    // Process the detected tags
    public void processDetectedTags() {
        List<TagDetection> results = pipeline.getDetectedTags();
        if (results == null || results.isEmpty()) {
            System.out.println("No tags detected.");
            return;
        }

        for (TagDetection tag : results) {
            System.out.println("Detected tag ID: " + tag.getTagId());

            // Get the corner coordinates of the AprilTag (4 corners)
            Point[] corners = tag.getCorners();
            Point topLeft = corners[0];
            Point topRight = corners[1];
            Point bottomLeft = corners[2];
            Point bottomRight = corners[3];

            // Print the corner coordinates
            System.out.println("Top Left: " + topLeft);
            System.out.println("Top Right: " + topRight);
            System.out.println("Bottom Left: " + bottomLeft);
            System.out.println("Bottom Right: " + bottomRight);

            // Calculate distances using Pythagoras' theorem
            double distanceTopLeftBottomLeft = calculateDistance(topLeft, bottomLeft);
            double distanceTopRightBottomRight = calculateDistance(topRight, bottomRight);
            double distanceTopLeftTopRight = calculateDistance(topLeft, topRight);
            double distanceBottomLeftBottomRight = calculateDistance(bottomLeft, bottomRight);

            // Output the distances between corners
            System.out.println("Distance between top-left and bottom-left: " + distanceTopLeftBottomLeft);
            System.out.println("Distance between top-right and bottom-right: " + distanceTopRightBottomRight);
            System.out.println("Distance between top-left and top-right: " + distanceTopLeftTopRight);
            System.out.println("Distance between bottom-left and bottom-right: " + distanceBottomLeftBottomRight);

            // Calculate approximate distance to tag using width of the detected tag in pixels
            double distanceToTag = estimateDistanceToTag(distanceTopLeftTopRight);
            System.out.println("Estimated distance to the tag: " + distanceToTag + " meters");
        }
    }

    // Function to calculate the distance between two points using the Pythagorean theorem
    public double calculateDistance(Point corner1, Point corner2) {
        return Math.sqrt(Math.pow(corner2.x - corner1.x, 2) + Math.pow(corner2.y - corner1.y, 2));
    }

    // Estimate distance to the tag based on its size in the image
    private double estimateDistanceToTag(double tagWidthInPixels) {
        // Using the pinhole camera model: distance = (real size of tag * focal length) / tag size in pixels
        // You need the camera's focal length in pixels. This can be estimated from calibration.
        double focalLength = 500; // Example focal length in pixels (depends on your camera)
        return (TAG_SIZE * focalLength) / tagWidthInPixels;
    }

    // AprilTag detection pipeline class for EasyOpenCV
    public class AprilTagDetectionPipeline extends OpenCvPipeline {

        private List<TagDetection> detectedTags = new ArrayList<>();

        @Override
        public Mat processFrame(Mat input) {
            detectedTags.clear(); // Clear previous detections

            // Here, you would call an AprilTag detection library or algorithm to process the input image
            // Example: Use AprilTag detection code to find tags in the frame
            // This requires a third-party library or OpenCV techniques for actual tag detection.

            // Mock detection:
            detectedTags.add(new TagDetection()); // Example of adding a detected tag

            return input; // Return the processed frame
        }

        // Get detected tags
        public List<TagDetection> getDetectedTags() {
            return detectedTags;
        }
    }

    // Mock AprilTagDetection and TagDetection classes for the sake of this example
    public static class TagDetection {
        private int tagId = 1;  // Example tag ID
        private Point[] corners = new Point[] {
                new Point(100, 100),
                new Point(200, 100),
                new Point(100, 200),
                new Point(200, 200)
        };

        public int getTagId() {
            return tagId;
        }

        public Point[] getCorners() {
            return corners;
        }
    }
}
