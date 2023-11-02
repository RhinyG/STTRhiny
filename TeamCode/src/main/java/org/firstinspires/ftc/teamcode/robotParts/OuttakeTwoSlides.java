package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeTwoSlides extends RobotPart{

    Servo leftClaw;
    public Servo leftRotate;
    Servo rightClaw;
    public Servo rightRotate;
    public DcMotorEx slides;
    int upperLimit = 2150; //2400 but can shoot up to 130 more than limit
    int lowerLimit = 0;

    public enum ArmHeight {
        INTAKE(0),
        BOTTOM(470),
        FIRSTLINE(1370),
        SECONDLINE(2100);

        private int position;
        public int getPosition() {
            return this.position;
        }
        ArmHeight(int position) {
            this.position = position;
        }
    }
    public enum ClawPositions {
        RELEASE(1),
        GRAB(2);

        private int position;

        public int getPosition() {
            return this.position;
        }

        ClawPositions(int position) {
            this.position = position;
        }
    }
    public enum RotatePositions {
        INTAKEPOS(1),
        MOVEPOS(2),
        OUTTAKEPOS(3);

        private int position;
        public int getPosition() {
            return this.position;
        }
        RotatePositions(int position) {
            this.position = position;
        }
    }

    /**
     * does something
     * @param map
     */
    public void init(HardwareMap map) {
        leftClaw = map.get(Servo.class, "leftClaw");
        leftRotate = map.get(Servo.class, "leftRotate");
        rightClaw = map.get(Servo.class,"rightClaw");
        rightRotate = map.get(Servo.class,"rightRotate");

        slides = map.get(DcMotorEx.class, "slideLeft");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slides);
        resetEncoders();
    }

    /**
     *
     * @param position
     */
    public void updateRotate(RotatePositions position) {
        double leftRotatePos;
        double rightRotatePos = 0;
        if (position == RotatePositions.INTAKEPOS) {
            rightRotatePos = 0.35;
        } else if (position == RotatePositions.MOVEPOS) {
            rightRotatePos = 0.0; //0.5
        } else if (position == RotatePositions.OUTTAKEPOS) {
            rightRotatePos = 0.8;
        }
        leftRotatePos = 1 - rightRotatePos;
        leftClaw.setPosition(leftRotatePos);
        rightClaw.setPosition(rightRotatePos);
    }

    /**
     *
     * @param position
     */
    public void updateLeftClaw(ClawPositions position) {
        double leftClawPos = 0;
        if (position == ClawPositions.RELEASE) {
            leftClawPos = 0.4;
        } else if (position == ClawPositions.GRAB) {
            leftClawPos = 0.0;
        }
        leftRotate.setPosition(leftClawPos);
    }

    /**
     * Same as above
     * @param position
     */
    public void updateRightClaw(ClawPositions position) {
        double rightClawPos = 0;
        if (position == ClawPositions.RELEASE) {
            rightClawPos = 0.7;
        } else if (position == ClawPositions.GRAB) {
            rightClawPos = 0.8;
        }
        rightRotate.setPosition(rightClawPos);
    }

    /**
     * From Reza
     *
     * @param position
     * @param telemetry
     * @return
     */
    public double goToHeight(int position, Telemetry telemetry) {
        double margin = 100;
        double currentPosLeft = slides.getCurrentPosition();
        double distance = Math.abs(currentPosLeft - position);
        if (currentPosLeft < position) {
            if (distance > margin) {
                slides.setPower(1);
            } else {
                slides.setPower(1 * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distance > margin) {
                slides.setPower(-1);
            } else {
                slides.setPower(-1 * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosLeft <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }

    /**
     * From Reza
     * @param btns if True, buttonmode is on (and arm will go to predetermined position). If false, it's on manual.
     * @param power power for manual mode
     * @param heightLeft predetermined height for buttonmode
     * @param telemetry necessary otherwise NPE
     */

    public void update(boolean btns, double power, ArmHeight heightLeft, Telemetry telemetry) {
        double distanceLeft = 0;
        if (btns) {
            distanceLeft = goToHeight(heightLeft.getPosition(), telemetry);
            telemetry.addData("arm", slides.getCurrentPosition());
            telemetry.addData("arm goal", heightLeft.getPosition());
            telemetry.addLine(String.valueOf(heightLeft));
            telemetry.addData("arm power", slides.getPower());
        } else {
            int position = slides.getCurrentPosition();

            if (position <= lowerLimit && power <= 0) {
                setPower(0);
            }
            else if (position >= upperLimit && power >= 0) {
                setPower(0);
            } else {
                slides.setPower(power);
            }
            telemetry.addData("arm", position);
            telemetry.addData("arm power", slides.getPower());
            telemetry.addData("distance to goal", distanceLeft);
        }
    }
//    public void sequenceAttempt(Telemetry telemetry){
//        updateLeftClaw(ClawPositions.RELEASE);
//        updateLeftRotate(RotatePositions.MOVEPOS);
//        goToHeight(0,telemetry);
//        updateLeftRotate(RotatePositions.INTAKEPOS);
//        updateLeftClaw(ClawPositions.GRAB);
//        updateLeftRotate(RotatePositions.MOVEPOS);
//        goToHeight(1300,telemetry);
//        updateLeftRotate(RotatePositions.OUTTAKEPOS);
//        updateLeftClaw(ClawPositions.RELEASE);
//    }
}