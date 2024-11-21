package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;

public class ObjectDetection {

    // Camera dimensions
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;

    // Real-world dimensions of the object
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.5; // Example width in inches

    // Object detection variables
    private double cX = 0; // Centroid X-coordinate
    private double cY = 0; // Centroid Y-coordinate
    private double width = 0; // Detected object width

    // Color detection thresholds (you can adjust these values for different colors)
    private Scalar lowerRed = new Scalar(0, 120, 70);
    private Scalar upperRed = new Scalar(10, 255, 255);
    private Scalar lowerBlue = new Scalar(100, 150, 0);
    private Scalar upperBlue = new Scalar(140, 255, 255);
    private Scalar lowerYellow = new Scalar(40, 150, 150);
    private Scalar upperYellow = new Scalar(30, 255, 255);

    private OpenCvCamera camera;
    public ObjectDetectionPipeline objectDetectionPipeline;

    // Constructor
    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        objectDetectionPipeline = new ObjectDetectionPipeline();
        startObjectDetectionPipeline(hardwareMap); // Start the camera pipeline
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    // Initialize and start the camera
    private void startObjectDetectionPipeline(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(objectDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Camera opened successfully, now start streaming
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT); // Fixed: OpenCvCameraRotation enum used here
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera error if needed
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    // OpenCV pipeline for object detection
    public class ObjectDetectionPipeline extends OpenCvPipeline {

        private Mat hsvImage = new Mat();
        private Mat blurredImage = new Mat();
        private Mat thresholdRed = new Mat();
        private Mat thresholdBlue = new Mat();
        private Mat thresholdYellow = new Mat();
        private Mat combinedThreshold = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Convert the frame to HSV color space for better color filtering
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            // Apply color thresholds for red, blue, and yellow colors
            Core.inRange(hsvImage, lowerRed, upperRed, thresholdRed);
            Core.inRange(hsvImage, lowerBlue, upperBlue, thresholdBlue);
            Core.inRange(hsvImage, lowerYellow, upperYellow, thresholdYellow);

            // Combine the thresholded images using bitwise OR
            Core.bitwise_or(thresholdRed, thresholdBlue, combinedThreshold);
            Core.bitwise_or(combinedThreshold, thresholdYellow, combinedThreshold);

            // Blur the image to reduce noise
            Imgproc.GaussianBlur(combinedThreshold, blurredImage, new Size(5, 5), 0);

            // Find contours (potential objects)
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blurredImage, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Iterate over each detected contour
            for (MatOfPoint contour : contours) {
                // Calculate the bounding box of the contour
                Rect boundingRect = Imgproc.boundingRect(contour);
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                // Filter by size and aspect ratio (to identify rectangular objects)
                if (boundingRect.width > 50 && boundingRect.height > 50 && aspectRatio > 0.5 && aspectRatio < 2.0) {
                    // Find the centroid of the detected object
                    Moments moments = Imgproc.moments(contour);
                    cX = moments.get_m10() / moments.get_m00();
                    cY = moments.get_m01() / moments.get_m00();
                    width = boundingRect.width;

                    // Draw the contour and the bounding box for visualization
                    Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                    Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(input, "X: " + (int) cX + " Y: " + (int) cY, new Point(cX, cY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);
                }
            }

            // Return the processed frame
            return input;
        }

        // Calculate the distance to the object based on the width of the detected object
        public double getDistance() {
            if (width == 0) return Double.MAX_VALUE; // Avoid division by zero
            return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * CAMERA_WIDTH) / width;
        }

        // Get the centroid X and Y coordinates of the detected object
        public double getCentroidX() {
            return cX;
        }

        public double getCentroidY() {
            return cY;
        }

        // Get the width of the detected object
        public double getWidth() {
            return width;
        }
    }

    // Cleanup method to stop the camera and release resources
    public void stopObjectDetection() {
        if (camera != null) {
            camera.closeCameraDevice();
        }
    }

    // Optional: Telemetry updates for debugging and visualization
    public void updateTelemetry() {
        telemetry.addData("Centroid X", objectDetectionPipeline.getCentroidX());
        telemetry.addData("Centroid Y", objectDetectionPipeline.getCentroidY());
        telemetry.addData("Object Width", objectDetectionPipeline.getWidth());
        telemetry.addData("Distance", objectDetectionPipeline.getDistance());
        telemetry.update();
    }
}
