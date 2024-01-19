package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PixelManipulation extends RobotPart{

    private final LinearOpMode myOpMode;
    public Servo claw;
    public Servo wrist;
    public Servo elbow;
    public Servo intakeServo;
    public DcMotorEx slides;
    public DcMotorEx intake;
    int upperLimit = 1150;
    int lowerLimit = 0;

    public enum ArmHeight {
        INTAKE(0),
        CHAINPOS(100),
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

    public enum IntakeServoPositions {
        GROUNDPOS(0),
        STACKPOS(0.15);

        private final double position;
        public double getPosition() {
            return this.position;
        }
        IntakeServoPositions(double position) {
            this.position = position;
        }
    }

    public enum ElbowPositions {
        INTAKEPOS(0.745),
        MOVEPOSLOW(0.77),
        MOVEPOSHIGH(0.83),
        CHAINPOS(0.78),
        AUTONSTART(0.52),
        OUTTAKEPOS(0.56);


        private final double position;
        public double getPosition() {
            return this.position;
        }
        ElbowPositions(double position) {
            this.position = position;
        }
    }

    public enum WristPositions {
        INTAKEPOS(0.63),//0.39 is servo to slides, 0.63 is servo away
        ONE(0.35),
        TWO(0.52),
        THREE(0.72),
        FOUR(0.91),
        FIVE(0.208),
        SIX(0.97);

        private final double position;
        public double getPosition() {
            return this.position;
        }
        WristPositions(double position) {
            this.position = position;
        }
    }

    public enum ClawPositions {
        RELEASE(0.45),
        GRABONE(0.34),
        AUTONSTART(0.29),
        GRABTWO(0.365);

        private final double position;

        public double getPosition() {
            return this.position;
        }

        ClawPositions(double position) {
            this.position = position;
        }
    }

    public PixelManipulation(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * Init
     * @param map otherwise NPE
     */
    public void init(HardwareMap map, Telemetry telemetry) {
        elbow = map.get(Servo.class,"elbow");
        wrist = map.get(Servo.class, "wrist");
        claw = map.get(Servo.class, "claw");
        intakeServo = map.get(Servo.class, "intakeServo");

        claw.setPosition(ClawPositions.GRABONE.getPosition());
//        updateElbow(ElbowPositions.MOVEPOS);
        updateIntakeServo(IntakeServoPositions.GROUNDPOS);
        wrist.setPosition(WristPositions.INTAKEPOS.getPosition());

        slides = map.get(DcMotorEx.class, "slides");
        intake = map.get(DcMotorEx.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slides);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();
    }

    /**
     * This updates the claws to a (new) position
     */
    public void updateWrist(double x, double y, Telemetry telemetry) {
        double theta;
        double r = Math.sqrt(x * x + y * y);
        WristPositions outtakePos;

        if (x >= 0 && y >= 0) {
            theta = Math.atan(y / x);
        } else if (x<0) {
            theta = Math.atan(y / x) + Math.PI;
        } else {
            theta = Math.atan(y / x) + 2 * Math.PI;
        }

        if(r > 0.5){
            if (theta < 2.0/6.0*Math.PI) {
                outtakePos = WristPositions.ONE;
            } else if (theta < 4.0/6.0*Math.PI) {
                outtakePos = WristPositions.TWO;
            } else if (theta < 6.0/6.0*Math.PI) {
                outtakePos = WristPositions.THREE;
            } else if (theta < 8.0/6.0*Math.PI) {
                outtakePos = WristPositions.FOUR;
            } else if (theta < 10.0/6.0*Math.PI) {
                outtakePos = WristPositions.FIVE;
            } else {
                outtakePos = WristPositions.SIX;
            }
            telemetry.addData("wristpos",outtakePos);
//            wrist.setPosition(outtakePos.getPosition());

        } else if (myOpMode.gamepad2.left_stick_button) {
            outtakePos = WristPositions.INTAKEPOS;
//            wrist.setPosition(outtakePos.getPosition());
            telemetry.addData("Going for neutral",outtakePos);
        }
    }

    public void updateElbow(ElbowPositions position) {
        elbow.setPosition(position.getPosition());
    }

    public void updateIntakeServo (IntakeServoPositions position) {
        intakeServo.setPosition(position.getPosition());
    }

    /**
     * From Reza
     *
     * @param position is the height it should go to.
     * @param telemetry because otherwise NPE.
     * @return It returns its current distance to target so that can be used again to set a new target.
     */
    public double goToHeight(int position, double power, Telemetry telemetry) {
        double margin = 50.0;
        double currentPosLeft = slides.getCurrentPosition();
        double distance = Math.abs(currentPosLeft - position);
        if (currentPosLeft < position) {
            if (distance > margin) {
                slides.setPower(power);
            } else {
                slides.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distance > margin) {
                slides.setPower(-power);
            } else {
                slides.setPower(-power * (distance/margin) * 0.4);
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
    public void autonGoToHeight(ArmHeight height, Telemetry telemetry) {autonGoToHeight(height, 0.7, myOpMode.telemetry);};
    public void autonGoToHeight(ArmHeight height, double power, Telemetry telemetry) {
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
                slides.setPower(0.05);
            }
            telemetry.addData("arm height", slides.getCurrentPosition());
            telemetry.addData("arm goal", height.getPosition());
            telemetry.update();
            currentPos = slides.getCurrentPosition();
            dPos = Math.abs(currentPos - position);
        }
        slides.setPower(0.05);
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

    public void dropYellowPixel(){
        autonGoToHeight(PixelManipulation.ArmHeight.FIRSTLINE);
        myOpMode.sleep(100);
        claw.setPosition(PixelManipulation.ClawPositions.RELEASE.getPosition());
        myOpMode.sleep(100);
        claw.setPosition(PixelManipulation.ClawPositions.GRABONE.getPosition());
        myOpMode.sleep(100);
        updateElbow(PixelManipulation.ElbowPositions.MOVEPOSHIGH);
        myOpMode.sleep(100);
        autonGoToHeight(PixelManipulation.ArmHeight.INTAKE);
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
            distance = goToHeight(height.getPosition(), 1, telemetry);
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
    public void updateIntake(double power) {
        intake.setPower(power);
    }
}