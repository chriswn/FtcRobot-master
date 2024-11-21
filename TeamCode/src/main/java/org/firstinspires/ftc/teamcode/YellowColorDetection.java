package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;
@Autonomous(name="YellowColorDetection")

public class YellowColorDetection {

    public static Scalar lowerYellow = new Scalar(20, 100, 100);  // Lower bound for yellow color
    public static Scalar upperYellow = new Scalar(40, 255, 255);  // Upper bound for yellow color
}
