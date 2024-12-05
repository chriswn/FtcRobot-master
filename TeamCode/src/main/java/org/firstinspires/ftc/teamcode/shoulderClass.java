package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry; 

@Autonomous(name = "shoulderClass", group = "Competition")
public class shoulderClass extends LinearOpMode {
    private DcMotor shoulder;
    private class shoulderClass{
            private DcMotor shoulder = null;
             private static final double TICKS_PER_REVOLUTION = 560.0; // For REV Core Hex Motor
             private static final double MOTOR_POWER = 0.5;           // Motor power level

 public shoulderClass(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");

        shoulder.setDirection(DcMotor.Direction.REVERSE);

        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

             * @param shoulderTicks Number of encoder ticks to move the shoulder.

        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(MOTOR_POWER);

                stopMotors();

         public void moveShoulderToPosition(int ticks) {
              moveArmToPosition(ticks, 0); // Move only the shoulder
              }
     
     public void stopMotors() {
        shoulder.setPower(0);
        }

    } 
    
}
}
