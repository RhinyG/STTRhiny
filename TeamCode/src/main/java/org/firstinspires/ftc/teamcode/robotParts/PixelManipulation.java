package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixelManipulation extends RobotPart{

    private final LinearOpMode myOpMode;
    public Servo claw;
    public Servo leftRotate;
    public Servo rightRotate;
    public DcMotorEx slides;
    int upperLimit = 1150;
    int lowerLimit = 0;
    public enum ArmHeight {
        INTAKE(0),
        FIRSTLINE(460),
        SECONDLINE(700),
        THIRDLINE(1100);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmHeight(int position) {
            this.position = position;
        }
    }

    public enum ClawPositions {
        RELEASE(0.36),
        GRABONE(0.7),
        GRABTWO(0.53);

        private final double position;

        public double getPosition() {
            return this.position;
        }

        ClawPositions(double position) {
            this.position = position;
        }
    }
    public enum RotatePositions {
        INTAKEPOS(0.525),
        MOVEPOS(0.575),
        OUTTAKEPOS(0.165);

        private final double position;
        public double getPosition() {
            return this.position;
        }
        RotatePositions(double position) {
            this.position = position;
        }
    }

    public PixelManipulation(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * Init
     * @param map otherwise NPE
     */
    public void init(HardwareMap map) {
        claw = map.get(Servo.class, "claw");
        leftRotate = map.get(Servo.class, "leftRotate");
        rightRotate = map.get(Servo.class,"rightRotate");

        claw.setPosition(0.45);
//        claw.setPosition(ClawPositions.GRABONE.getPosition());
        updateRotate(RotatePositions.MOVEPOS);

        slides = map.get(DcMotorEx.class, "slides");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slides);
        resetEncoders();
    }

    /**
     * This updates the claws to a (new) position
     * @param position is a value from the enumerator RotatePositions
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
     * @param position is the height it should go to.
     * @param telemetry because otherwise NPE.
     * @return It returns its current distance to target so that can be used again to set a new target.
     */
    public double goToHeight(int position, Telemetry telemetry) {
        double margin = 50.0;
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
     * Merger from Reza's goToHeight and Sander's DriveY to allow goToHeight to work in autonomous.
     * @param height This is the position you want to go to. In ArmHeight, not integers.
     * @param telemetry Necessary otherwise NPE.
     */
    public void autonGoToHeight(ArmHeight height, Telemetry telemetry) {
        double margin = 50.0;
        int position = height.getPosition();
        double currentPos = slides.getCurrentPosition();
        double dPos = Math.abs(currentPos - position);
        while (!(Math.abs(dPos) < margin) && myOpMode.opModeIsActive()) {
            if (currentPos < position) {
                slides.setPower(0.5);
                telemetry.addLine("up");
            } else if (currentPos > position) {
                slides.setPower(-0.5);
                telemetry.addLine("down");
            } else if (position == 0 && currentPos <= 0) {
                slides.setPower(0);
            }
            telemetry.addData("arm height", slides.getCurrentPosition());
            telemetry.addData("arm goal", height.getPosition());
            telemetry.update();
            currentPos = slides.getCurrentPosition();
            dPos = Math.abs(currentPos - position);
        }
        slides.setPower(0);
        myOpMode.sleep(100);
    }
    public void autonGoToHeight(ArmHeight height){autonGoToHeight(height, myOpMode.telemetry);}

    public void SanderArm(int var){
        slides.setTargetPosition(var);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(slides.getCurrentPosition() > var) {
            slides.setPower(-0.3);
        } else {
            slides.setPower(0.3);
        }
        while (myOpMode.opModeIsActive() && slides.isBusy()) {
            myOpMode.idle();
        }
        slides.setPower(0.2);
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
}