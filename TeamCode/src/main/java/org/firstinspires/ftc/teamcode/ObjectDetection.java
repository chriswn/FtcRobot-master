package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetection {

    public static final int CAMERA_WIDTH = 1920;
    public static final int CAMERA_HEIGHT = 1080;
    public static final double OBJECT_WIDTH_IN_REAL_WORLD_UNITS = 3.5; // inches

    private OpenCvCamera camera;
    private ObjectDetectionPipeline pipeline;
    private Telemetry telemetry;

    // Color thresholds (adjust based on your robot’s needs)
    private Scalar lowerRed = new Scalar(0, 120, 70);
    private Scalar upperRed = new Scalar(10, 255, 255);
    private Scalar lowerBlue = new Scalar(100, 150, 0);
    private Scalar upperBlue = new Scalar(140, 255, 255);
    private Scalar lowerYellow = new Scalar(40, 150, 150);
    private Scalar upperYellow = new Scalar(30, 255, 255);

    // Color thresholds (can be modified at runtime through telemetry)
    public ObjectDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.pipeline = new ObjectDetectionPipeline();
        startCamera(hardwareMap);
    }

    private void startCamera(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);

        // Fixing AsyncCameraOpenListener issue
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Set the live view container ID (for displaying camera feed)
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    // Method to access the pipeline data
    public ObjectDetectionPipeline getPipeline() {
        return pipeline;
    }

    // Method to access the camera
    public OpenCvCamera getCamera() {
        return camera;
    }

    // The OpenCvPipeline that processes the frame from the camera
    public class ObjectDetectionPipeline extends OpenCvPipeline {

        private double centroidX = 0, centroidY = 0, width = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvImage = new Mat();
            Mat blurredImage = new Mat();
            Mat thresholdRed = new Mat();
            Mat thresholdBlue = new Mat();
            Mat thresholdYellow = new Mat();
            Mat combinedThreshold = new Mat();

            // Convert the input frame from RGB to HSV color space
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            // Threshold the image to detect red, blue, and yellow colors
            Core.inRange(hsvImage, lowerRed, upperRed, thresholdRed);
            Core.inRange(hsvImage, lowerBlue, upperBlue, thresholdBlue);
            Core.inRange(hsvImage, lowerYellow, upperYellow, thresholdYellow);

            // Combine the thresholds for all colors
            Core.bitwise_or(thresholdRed, thresholdBlue, combinedThreshold);
            Core.bitwise_or(combinedThreshold, thresholdYellow, combinedThreshold);

            // Apply a Gaussian blur to smooth the thresholded image
            Imgproc.GaussianBlur(combinedThreshold, blurredImage, new Size(5, 5), 0);

            // Find contours in the blurred image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(blurredImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Process each contour
            for (MatOfPoint contour : contours) {
                Rect boundingRect = Imgproc.boundingRect(contour);

                // Only process contours larger than 50x50 pixels
                if (boundingRect.width > 50 && boundingRect.height > 50) {
                    Moments moments = Imgproc.moments(contour);
                    centroidX = moments.get_m10() / moments.get_m00();
                    centroidY = moments.get_m01() / moments.get_m00();
                    width = boundingRect.width;

                    // Draw the bounding box and centroid on the image
                    Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                    Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2);
                    Imgproc.putText(input, "X: " + (int) centroidX + " Y: " + (int) centroidY, new Point(centroidX, centroidY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);
                }
            }

            // Optional: Send data to telemetry for debugging
            telemetry.addData("Centroid X", centroidX);
            telemetry.addData("Centroid Y", centroidY);
            telemetry.addData("Object Width", width);
            telemetry.update();

            return input;
        }

        // Getters for centroid and object width
        public double getCentroidX() {
            return centroidX;
        }

        public double getCentroidY() {
            return centroidY;
        }

        public double getWidth() {
            return width;
        }

        // Calculate the distance to the object based on its width in the image
        public double getDistance() {
            if (width == 0) {
                return 0; // Avoid division by zero
            }
            return (OBJECT_WIDTH_IN_REAL_WORLD_UNITS * CAMERA_WIDTH) / width;
        }
    }
}
