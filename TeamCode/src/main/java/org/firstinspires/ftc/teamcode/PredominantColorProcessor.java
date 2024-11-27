package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.HashMap;
import java.util.Map;

public class PredominantColorProcessor {

    // Enum for swatches (colors)
    public enum Swatch {
        RED, BLUE, YELLOW, BLACK, WHITE
    }

    // A map to store color ranges (HSV)
    private Map<Swatch, Scalar[]> colorRanges;

    private Mat frame;
    private Mat hsvFrame;

    // Constructor that initializes color ranges
    public PredominantColorProcessor() {
        colorRanges = new HashMap<>();

        // Define color ranges in HSV space
        colorRanges.put(Swatch.RED, new Scalar[]{new Scalar(0, 120, 70), new Scalar(10, 255, 255)}); // Example range for Red
        colorRanges.put(Swatch.BLUE, new Scalar[]{new Scalar(100, 150, 50), new Scalar(140, 255, 255)}); // Example range for Blue
        colorRanges.put(Swatch.YELLOW, new Scalar[]{new Scalar(20, 100, 100), new Scalar(30, 255, 255)}); // Example range for Yellow
        colorRanges.put(Swatch.BLACK, new Scalar[]{new Scalar(0, 0, 0), new Scalar(180, 255, 50)}); // Example range for Black
        colorRanges.put(Swatch.WHITE, new Scalar[]{new Scalar(0, 0, 200), new Scalar(180, 50, 255)}); // Example range for White
    }

    // Method to process the image and detect the predominant color in a given ROI
    public Result processFrame(Mat inputFrame) {
        this.frame = inputFrame;
        hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Map<Swatch, Integer> colorCount = new HashMap<>();

        // Loop over all swatches and count the pixels that fall within the color range
        for (Swatch swatch : Swatch.values()) {
            Scalar[] range = colorRanges.get(swatch);
            Mat mask = new Mat();
            Core.inRange(hsvFrame, range[0], range[1], mask);

            // Count non-zero pixels (pixels that are part of the detected color)
            int count = Core.countNonZero(mask);
            colorCount.put(swatch, count);
        }

        // Find the swatch with the highest count
        Swatch dominantColor = getDominantColor(colorCount);

        return new Result(dominantColor, colorCount.get(dominantColor));
    }

    // Helper method to determine the dominant color based on pixel count
    private Swatch getDominantColor(Map<Swatch, Integer> colorCount) {
        int maxCount = 0;
        Swatch dominantColor = Swatch.RED;

        for (Map.Entry<Swatch, Integer> entry : colorCount.entrySet()) {
            if (entry.getValue() > maxCount) {
                maxCount = entry.getValue();
                dominantColor = entry.getKey();
            }
        }
        return dominantColor;
    }

    // Nested class to return the analysis result
    public static class Result {
        private Swatch swatch;
        private int confidence;

        public Result(Swatch swatch, int confidence) {
            this.swatch = swatch;
            this.confidence = confidence;
        }

        public Swatch getSwatch() {
            return swatch;
        }

        public int getConfidence() {
            return confidence;
        }
    }
}
