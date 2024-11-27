package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;
import java.util.ArrayList;

public class ObjectDetectionPipeline extends OpenCvPipeline {

    // Matrices for image processing
    private Mat hsvImage = new Mat();
    private Mat blurredImage = new Mat();
    private Mat thresholdRed = new Mat();
    private Mat thresholdBlue = new Mat();
    private Mat thresholdYellow = new Mat();
    private Mat combinedThreshold = new Mat();

    // Color detection ranges
    private Scalar lowerRed = RedColorDetection.lowerRed;
    private Scalar upperRed = RedColorDetection.upperRed;
    private Scalar lowerBlue = BlueColorDetection.lowerBlue;
    private Scalar upperBlue = BlueColorDetection.upperBlue;
    private Scalar lowerYellow = YellowColorDetection.lowerYellow;
    private Scalar upperYellow = YellowColorDetection.upperYellow;

    // Detected object's properties
    public double cX = 0; // Centroid X-coordinate
    public double cY = 0; // Centroid Y-coordinate
    public double width = 0; // Detected object width

    private String detectedColor = "None"; // To store detected color

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
                if (moments.get_m00() != 0) {  // Avoid divide-by-zero error
                    cX = moments.get_m10() / moments.get_m00();
                    cY = moments.get_m01() / moments.get_m00();
                }
                width = boundingRect.width;

                // Draw the contour and the bounding box for visualization
                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(255, 0, 0), 2);
                Imgproc.putText(input, "X: " + (int) cX + " Y: " + (int) cY, new Point(cX, cY), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 255), 2);

                // Detect which color was detected based on the thresholds
                if (Core.countNonZero(thresholdRed) > 0) {
                    detectedColor = "Red";
                } else if (Core.countNonZero(thresholdBlue) > 0) {
                    detectedColor = "Blue";
                } else if (Core.countNonZero(thresholdYellow) > 0) {
                    detectedColor = "Yellow";
                }
            }
        }

        // Return the processed frame with drawn contours
        return input;
    }

    // Calculate the distance to the object based on the width of the detected object
    public double getDistance() {
        if (width == 0) return Double.MAX_VALUE; // Avoid division by zero
        // Distance formula based on the detected object's width and the camera parameters
        return (ObjectDetection.OBJECT_WIDTH_IN_REAL_WORLD_UNITS * ObjectDetection.CAMERA_WIDTH) / width;
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

    // Getter for detectedColor
    public String getDetectedColor() {
        return detectedColor;
    }
}
