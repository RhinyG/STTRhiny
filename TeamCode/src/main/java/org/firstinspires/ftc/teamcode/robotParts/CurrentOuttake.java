package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CurrentOuttake extends RobotPart{

    public Servo claw;
    public Servo leftRotate;
    public Servo rightRotate;
    public DcMotorEx slides;
    int upperLimit = 2250; //2400 but can shoot up to 130 more than limit
    int lowerLimit = 0;
    int sequenceStep = 1;

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
        RELEASE(0.45),
        GRAB(0.1);

        private double position;

        public double getPosition() {
            return this.position;
        }

        ClawPositions(double position) {
            this.position = position;
        }
    }
    public enum RotatePositions {
        INTAKEPOS(0.36),
        MOVEPOS(0.26),
        OUTTAKEPOS(0.7);

        private double position;
        public double getPosition() {
            return this.position;
        }
        RotatePositions(double position) {
            this.position = position;
        }
    }

    /**
     * does something
     * @param map
     */
    public void init(HardwareMap map) {
        claw = map.get(Servo.class, "claw");
        leftRotate = map.get(Servo.class, "leftRotate");
        rightRotate = map.get(Servo.class,"rightRotate");

        claw.setPosition(ClawPositions.RELEASE.getPosition());
        updateRotate(RotatePositions.MOVEPOS);

        slides = map.get(DcMotorEx.class, "slides");

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
        double rightRotatePos = position.getPosition();
        leftRotatePos = 1 - rightRotatePos;
        leftRotate.setPosition(leftRotatePos);
        rightRotate.setPosition(rightRotatePos);
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
     * @param height predetermined height for buttonmode
     * @param telemetry necessary otherwise NPE
     */

    public void updateSlide(boolean btns, double power, ArmHeight height, Telemetry telemetry) {
        double distance = 0;
        if (btns) {
            distance = goToHeight(height.getPosition(), telemetry);
            telemetry.addData("arm", slides.getCurrentPosition());
            telemetry.addData("arm goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
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
            telemetry.addData("distance to goal", distance);
        }
    }

    public void sequenceThree(){
        switch(sequenceStep){
            case 1:
                updateRotate(RotatePositions.MOVEPOS);
                sequenceStep++;
            case 2:
                claw.setPosition(ClawPositions.RELEASE.getPosition());
                sequenceStep++;
        }
    }

//    public void sequenceTwo(boolean start){
//        boolean newSequence = sequenceStart;
//        if(start != sequenceStart){
//            newSequence = start;
//        }
//        if(sequenceStart){
//
//        }
//    }
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