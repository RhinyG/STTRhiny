package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake extends RobotPart{

    Servo leftClaw;
    Servo leftRotate;
    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;
    double leftClawPos;
    double leftRotatePos;
    int upperLimit = 3500;

    public void init(HardwareMap map) {
        leftClaw = map.get(Servo.class, "leftClaw");
        leftRotate = map.get(Servo.class, "leftRotate");

        slideLeft = map.get(DcMotorEx.class, "arm1");
//        slideRight = map.get(DcMotorEx.class, "arm1");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slideLeft);
//        motors.put("slideRight", slideRight);
        resetEncoders();
    }

    /**
     *
     * @param mode 1 = intakePos, 2 = movePos, 3 = outtakePos
     */
    public void updateLeftRotate(int mode) {
        if (mode == 1) {
            leftClawPos = 1.0;
        } else if (mode == 2) {
            leftClawPos = 0.2; //0.5
        } else if (mode == 3) {
            leftClawPos = 0.5;
        }

        leftClaw.setPosition(leftClawPos);
    }

    /**
     *
     * @param mode 1 = release, 2 = grab
     */
    public void updateLeftClaw(int mode) {
        if (mode == 1) {
            leftRotatePos = 0.5;
        } else if (mode == 2) {
            leftRotatePos= 0.25;
        }
        leftRotate.setPosition(leftRotatePos);
    }

    public void moveSlidesManually(double power) {
        if (slideLeft.getCurrentPosition() < upperLimit) {
            slideLeft.setPower(power);
            slideRight.setPower(power);
        }
    }

    /**
     * Reza's code
     * Don't insert targetPosition larger than upperLimit, there is no control and wires will snap.
     * @param targetPosition desired slide height
     * @param telemetry anders NPE
     */
    public double slidesGoToHeight(int targetPosition, Telemetry telemetry) {
        double margin = 100;
        double currentPos = slideLeft.getCurrentPosition();
        double distance = Math.abs(currentPos - targetPosition);
        if (currentPos < targetPosition) {
            if (distance > margin) {
                slideLeft.setPower(1);
                slideRight.setPower(1);
            } else {
                slideLeft.setPower(1 * (distance/margin) * 0.4);
                slideRight.setPower(1 * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPos > targetPosition) {
            if (distance > margin) {
                slideLeft.setPower(-1);
                slideRight.setPower(-1);
            } else {
                slideLeft.setPower(-1 * (distance/margin) * 0.4);
                slideRight.setPower(-1 * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (targetPosition == 0 && currentPos <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }
    /**
     * Sequence so the outtake claw can pick up a pixel in and place it on the backboard at the desired height,
     * without the human player needing to press a million buttons.
     * @param height encoder height
     * @param telemetry idk why anders error
     */
    public void outtakeSequence(int height, Telemetry telemetry) {
        updateLeftClaw(1);
        updateLeftRotate(2);
        slidesGoToHeight(0, telemetry);
        updateLeftRotate(1);
        updateLeftClaw(2);
        slidesGoToHeight(height, telemetry);
        updateLeftRotate(3);
        updateLeftClaw(1);
        updateLeftRotate(2);
    }
}