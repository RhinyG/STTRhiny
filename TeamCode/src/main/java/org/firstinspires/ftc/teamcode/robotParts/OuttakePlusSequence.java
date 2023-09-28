package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakePlusSequence extends RobotPart{

    Servo leftClaw;
    Servo leftRotate;
    Servo rightClaw;
    Servo rightRotate;
    public DcMotorEx slideLeft;
    public DcMotorEx slideRight;
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

    public void init(HardwareMap map) {
        leftClaw = map.get(Servo.class, "leftClaw");
        leftRotate = map.get(Servo.class, "leftRotate");

        slideLeft = map.get(DcMotorEx.class, "slideLeft");
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRight = map.get(DcMotorEx.class, "slideRight");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slideLeft);
        motors.put("slideRight", slideRight);
        resetEncoders();
    }

    /**
     *
     * @param position
     */
    public void updateLeftRotate(RotatePositions position) {
        double leftClawPos = 0;
        if (position == RotatePositions.INTAKEPOS) {
            leftClawPos = 0.5;
        } else if (position == RotatePositions.MOVEPOS) {
            leftClawPos = 0.2; //0.5
        } else if (position == RotatePositions.OUTTAKEPOS) {
            leftClawPos = 1.0;
        }
        leftClaw.setPosition(leftClawPos);
    }

    /**
     * Same as above
     * @param position
     */
    public void updateRightRotate(RotatePositions position) {
        double rightClawPos = 0;
        if (position == RotatePositions.INTAKEPOS) {
            rightClawPos = 0.5;
        } else if (position == RotatePositions.MOVEPOS) {
            rightClawPos = 0.2;
        } else if (position == RotatePositions.OUTTAKEPOS) {
            rightClawPos = 1.0;
        }
        rightClaw.setPosition(rightClawPos);
    }

    /**
     *
     * @param position
     */
    public void updateLeftClaw(ClawPositions position) {
        double leftRotatePos = 0;
        if (position == ClawPositions.RELEASE) {
            leftRotatePos = 0.25;
        } else if (position == ClawPositions.GRAB) {
            leftRotatePos = 0.5;
        }
        leftRotate.setPosition(leftRotatePos);
    }

    /**
     * Same as above
     * @param position
     */
    public void updateRightClaw(ClawPositions position) {
        double rightRotatePos = 0;
        if (position == ClawPositions.RELEASE) {
            rightRotatePos = 0.25;
        } else if (position == ClawPositions.GRAB) {
            rightRotatePos = 0.5;
        }
        rightRotate.setPosition(rightRotatePos);
    }

    /**
     * From Reza
     * When we have two equal motors it might be better to move everything under one thing so no currentPosLeft & currentPosRight, just currentPos
     * @param position
     * @param telemetry
     * @return
     */
    public double goToHeight(int position, Telemetry telemetry) {
        double margin = 100;
        double currentPosLeft = slideLeft.getCurrentPosition();
        double currentPosRight = slideRight.getCurrentPosition();
        double distanceLeft = Math.abs(currentPosLeft - position);
        double distanceRight = Math.abs(currentPosRight - position);
        if (currentPosLeft < position) {
            if (distanceLeft > margin) {
                slideLeft.setPower(1);
            } else {
                slideLeft.setPower(1 * (distanceLeft/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distanceLeft > margin) {
                slideLeft.setPower(-1);
            } else {
                slideLeft.setPower(-1 * (distanceLeft/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosLeft <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        //so delete this then
        if (currentPosRight < position) {
            if (distanceRight > margin) {
                slideRight.setPower(1);
            } else {
                slideRight.setPower(1 * (distanceRight/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosRight > position) {
            if (distanceRight > margin) {
                slideRight.setPower(-1);
            } else {
                slideRight.setPower(-1 * (distanceRight/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosRight <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distanceLeft;
    }

    /**
     * From Reza
     * @param btns if True, buttonmode is on (and arm will go to predetermined position). If false, it's on manual.
     * @param power power for manual mode
     * @param heightLeft predetermined height for buttonmode
     * @param heightRight same but right
     * @param telemetry necessary otherwise NPE
     */

    public void update(boolean btns, double power, ArmHeight heightLeft, ArmHeight heightRight, Telemetry telemetry) {
        double distanceLeft = 0;
        double distanceRight = 0;
        if (btns) {
            distanceLeft = goToHeight(heightLeft.getPosition(), telemetry);
            distanceRight = goToHeight(heightRight.getPosition(), telemetry);
            telemetry.addData("arm", slideLeft.getCurrentPosition());
            telemetry.addData("arm goal", heightLeft.getPosition());
            telemetry.addLine(String.valueOf(heightLeft));
            telemetry.addData("arm power", slideLeft.getPower());
        } else {
            int position = slideLeft.getCurrentPosition();

            if (position <= lowerLimit && power <= 0) {
                setPower(0);
            }
            else if (position >= upperLimit && power >= 0) {
                setPower(0);
            } else {
                slideLeft.setPower(power);
                slideRight.setPower(power);
            }
            telemetry.addData("arm", position);
            telemetry.addData("arm power", slideLeft.getPower());
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