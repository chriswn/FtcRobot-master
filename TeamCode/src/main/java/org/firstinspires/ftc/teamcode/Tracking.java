package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tracking {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Telemetry telemetry;

    // Constructor to initialize motors and telemetry
    public Tracking(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motors to be run using brake mode (or you can use float depending on your setup)
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Method to move towards the object based on its distance
    public void moveTowardsObject(double distance) {
        double speed = calculateSpeedBasedOnDistance(distance);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        // Debugging output
        telemetry.addData("Moving Towards Object", "Distance: %.2f", distance);
        telemetry.update();
    }

    // Method to rotate the robot when no object is detected
    public void searchForObject() {
        // Example: Rotate the robot to search for an object
        rotateRobot(0.5);  // Rotate at 50% speed
    }

    // Method to rotate the robot
    private void rotateRobot(double speed) {
        leftMotor.setPower(-speed);  // Reverse one motor to rotate
        rightMotor.setPower(speed);  // Forward the other motor to rotate
    }

    // Helper method to calculate speed based on distance
    private double calculateSpeedBasedOnDistance(double distance) {
        // Scale speed from distance, minimum speed is 0.2, max is 1.0
        return Math.max(0.2, Math.min(1.0, distance / 12.0));  // Example scaling factor
    }

    // Method to stop the robot
    public void stopMovement() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
