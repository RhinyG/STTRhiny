package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivetrainAlex extends RobotPart{

    AlexDistanceSensorUtil distanceSensor = new AlexDistanceSensorUtil();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;

    public static double maxSpeed = 0.6;

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double forward, double right, double rotate, double slowMode) {
        double leftFrontPower = -forward - right + rotate;
        double rightFrontPower = -forward + right - rotate;
        double leftRearPower = -forward + right + rotate;
        double rightRearPower = -forward - right - rotate;
        double maxPower = 1.0;
        double slowModeCalc;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));

        if(slowMode > 0.5){
            slowModeCalc = (1.0/slowMode)/2;
            leftFront.setPower(slowModeCalc * maxSpeed * (leftFrontPower / maxPower));
            rightFront.setPower(slowModeCalc * maxSpeed * (rightFrontPower / maxPower));
            rightBack.setPower(slowModeCalc *maxSpeed * rightRearPower / maxPower);
            leftBack.setPower(slowModeCalc * maxSpeed * (leftRearPower / maxPower));
        } else {
            leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
            rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
            rightBack.setPower(maxSpeed * rightRearPower / maxPower);
            leftBack.setPower(maxSpeed * (leftRearPower / maxPower));
        }
    }
}