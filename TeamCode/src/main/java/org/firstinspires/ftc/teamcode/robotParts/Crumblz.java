package org.firstinspires.ftc.teamcode.robotParts;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//TODO: what does extends do? Can you use that instead of a constructor?
public class Crumblz extends RobotPart {
//TODO: explain variables
    private final LinearOpMode myOpMode;
    Telemetry telemetry;
    HardwareMap map;
    public Servo clawLeft,clawRight,elbow;
    public DcMotorEx armExtend1, armExtend2,armRotate;
    public int state;
    double
            p = 0.008,
            i = 0,
            d = 0,
            f = 0.08;
    //TODO: see if other PIDs like liftPID work better.
    PIDController controller = new PIDController(p, i, d);
    int rotateGoal;
//TODO: explain enumerators
//TODO: enumerators in separate files
    public enum ArmRotatePos {
        intakeGround(0),
        outtakeBack(3050);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmRotatePos(int position) {
            this.position = position;
        }
    }

    public enum ArmExtendPos {
        ZERO(0);
        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmExtendPos(int position) {this.position = position;}
    }

    public enum ArmLimits {
        slideLower(0),
        slideUpper(900),
        rotateLower(0),
        rotateUpper(3050);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmLimits(int position) {
            this.position = position;
        }
    }

    public enum ElbowPositions {
        intakePos(0.085),//0.15
        stackFivePos(0.095),
        outtakeFrontSidePos(0.075), //.1
        outtakeBackSidePos(0.76),
        foldPos(0.8);

        private final double position;
        public double getPosition() {
            return this.position;
        }
        ElbowPositions(double position) {
            this.position = position;
        }
    }

    public enum ClawPositions {
        openLeft(0.69),
        releaseLeft(0.50),
        grabLeft(0.33),
        openRight(0.38),
        releaseRight(0.52),
        grabRight(0.7);

        private final double position;
        public double getPosition() {
            return this.position;
        }
        ClawPositions(double position) {
            this.position = position;
        }
    }

    /**
     * This is the constructor.
     * @param opmode is opmode from LinearOpMode file
     */
    public Crumblz(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = opmode.telemetry;
        map = opmode.hardwareMap;
    }

    /**
     * This methods initialises the pixel manipulation mechanisms and sets all the directions and modes to their correct settings.
     */
    public void init() {
        elbow = myOpMode.hardwareMap.get(Servo.class,"elbow");
        clawLeft = myOpMode.hardwareMap.get(Servo.class, "clawLeft");
        clawRight = myOpMode.hardwareMap.get(Servo.class, "clawRight");

        elbow.setPosition(ElbowPositions.foldPos.getPosition());
        clawLeft.setPosition(ClawPositions.grabLeft.getPosition());
        clawRight.setPosition(ClawPositions.grabRight.getPosition());

        armExtend1 = myOpMode.hardwareMap.get(DcMotorEx.class, "armExtend");
        armExtend2 = myOpMode.hardwareMap.get(DcMotorEx.class, "armExtendo");
        armRotate = myOpMode.hardwareMap.get(DcMotorEx.class, "armRotate");

        armExtend1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend1.setDirection(DcMotorSimple.Direction.FORWARD);
        armExtend2.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        // from ArmReza code seems do interact with RobotPart.java I don't know really
        motors.put("armExtend", armExtend1);

        resetEncoders();
    }

   //TODO: documentation
    public void updateClaw(ClawPositions positionLeft, ClawPositions positionRight) {
        clawLeft.setPosition(positionLeft.getPosition());
        clawRight.setPosition(positionRight.getPosition());
    }
    //TODO: documentation
    public void updateElbow() {
        double position;
        if (armRotate.getCurrentPosition() < 300) {
            position = ElbowPositions.intakePos.getPosition();
        } else if (armRotate.getCurrentPosition() < 2200) {
            position = ElbowPositions.outtakeFrontSidePos.getPosition();
        } else {
            position = ElbowPositions.outtakeBackSidePos.getPosition();
        }
        elbow.setPosition(position);
    }

    //TODO: documentation
    public void slidesGoToHeight(int position, double power) {
        double margin = 50.0;
        double currentPosLeft = armExtend1.getCurrentPosition();
        double distance = Math.abs(currentPosLeft - position);
        if (currentPosLeft < position) {
            if (distance > margin) {
                armExtend1.setPower(power);
                armExtend2.setPower(power);
            } else {
                armExtend1.setPower(power * (distance/margin) * 0.4);
                armExtend2.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distance > margin) {
                armExtend1.setPower(-power);
                armExtend2.setPower(-power);
            } else {
                armExtend1.setPower(-power * (distance/margin) * 0.4);
                armExtend2.setPower(-power * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosLeft <= 0) {
            armExtend1.setPower(0);
            armExtend2.setPower(0);
        } else {
            armExtend1.setPower(0.01);
            armExtend2.setPower(0.01);
        }
        telemetry.addData("distance to goal", distance);
    }

    //TODO: documentation
    public void updateSlide(boolean buttonMode, double power, Crumblz.ArmExtendPos height) {
        if (buttonMode) {
            slidesGoToHeight(height.getPosition(), 1.0);
            telemetry.addData("slide", armExtend1.getCurrentPosition());
            telemetry.addData("slide goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("arm power", armExtend1.getPower());
        } else {
            int position = armExtend1.getCurrentPosition();
            boolean limitReached;

            limitReached = (position <= ArmLimits.slideLower.getPosition() && power <= 0) || (position >= ArmLimits.slideUpper.getPosition() && power >= 0);

            if (!limitReached) {
                armExtend1.setPower(power);
                armExtend2.setPower(power);
            } else {

                armExtend1.setPower(0);
                armExtend2.setPower(0);
            }
            telemetry.addData("slide pos", position);
            telemetry.addData("slide power", armExtend1.getPower());
        }
    }

    //TODO: documentation
    public double rotateToPos(int position, double power) {
        double margin = 60.0;
        double currentPosLeft = armRotate.getCurrentPosition();
        double distance = Math.abs(currentPosLeft - position);
        if (currentPosLeft < position) {
            if (distance > margin) {
                armRotate.setPower(power);
            } else {
                armRotate.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distance > margin) {
                armRotate.setPower(-power);
            } else {
                armRotate.setPower(-power * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosLeft <= 0) {
            setAllPowers(0);
        } else {
            setAllPowers(0.01);
        }
        return distance;
    }

    //TODO: documentation
    public void updateRotate(boolean buttonMode, double power, Crumblz.ArmRotatePos height, int holdSlides) {
        double distance = 0;
        double slideHeight = armExtend1.getCurrentPosition();
        if (buttonMode) {
            distance = rotateToPos(height.getPosition(), 1);
            if(distance > 100){
                slidesGoToHeight(holdSlides,0.7);
            }
            telemetry.addData("rotate", armRotate.getCurrentPosition());
            telemetry.addData("rotate goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("rotate power", armRotate.getPower());
        } else {
            int position = armRotate.getCurrentPosition();
            boolean limitReached;

            limitReached = (position <= ArmLimits.rotateLower.getPosition() && power <= 0) || (position >= ArmLimits.rotateUpper.getPosition() && power >= 0);

            if (!limitReached) {
                if (slideHeight > 700) {
                    armRotate.setPower(0.5 * power);
                }else {
                    armRotate.setPower(power);
                }
            } else {
                armRotate.setPower(0);
            }
            telemetry.addData("rotate", position);
            telemetry.addData("rotate power", armRotate.getPower());
            telemetry.addData("distance to goal", distance);
        }
    }

    //TODO: documentation
    public void secretRotate(double power) {
        int position = armRotate.getCurrentPosition();
        armRotate.setPower(power);

        telemetry.addData("rotate", position);
        telemetry.addData("rotate power", armRotate.getPower());
    }

    //TODO: documentation
    public void secretSlide(double power) {
        int position = armExtend1.getCurrentPosition();
        armExtend1.setPower(power);
        armExtend2.setPower(power);
        telemetry.addData("slide", position);
        telemetry.addData("slide power", armExtend1.getPower());
    }

    //TODO: documentation
    public void autonArm(int var){
        armRotate.setTargetPosition(var);
        armRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(armRotate.getCurrentPosition() > var) {
            armRotate.setPower(-0.8);
        } else {
            armRotate.setPower(0.8);
        }
        while (myOpMode.opModeIsActive() && armRotate.isBusy()) {
            myOpMode.idle();
        }
        armRotate.setPower(0);
    }

    //TODO: documentation
    public void autonSlide(int var){
        armExtend1.setTargetPosition(var);
        armExtend2.setTargetPosition(var);
        armExtend1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(armExtend1.getCurrentPosition() > var) {
            armExtend1.setPower(-0.5);
            armExtend2.setPower(-0.5);
        } else {
            armExtend1.setPower(0.5);
            armExtend2.setPower(0.5);
        }
        while (myOpMode.opModeIsActive() && (armExtend1.isBusy() || armExtend2.isBusy())) {
            myOpMode.idle();
        }
    }

    //TODO: documentation
    public void FSMArm(int var, double speed){
        if (Math.abs(armRotate.getCurrentPosition() - var) > 100) {
            if (armRotate.getCurrentPosition() > var) {
                armRotate.setPower(-speed);
            } else {
                armRotate.setPower(speed);
            }
        } else if (Math.abs(armExtend1.getCurrentPosition() - var) > 50) {
            if (armRotate.getCurrentPosition() > var) {
                armRotate.setPower(-0.01);
            } else {
                armRotate.setPower(0.01);
            }
        }
    }

    //TODO: documentation
    //TODO: PID
    public void FSMSlide(int var, double speed){
        if (Math.abs(armExtend1.getCurrentPosition() - var) > 175) {
            if (armExtend1.getCurrentPosition() > var) {
                armExtend1.setPower(-speed);
                armExtend2.setPower(-speed);
            } else {
                armExtend1.setPower(speed);
                armExtend2.setPower(speed);
            }
        } else if (Math.abs(armExtend1.getCurrentPosition() - var) > 10) {
            if (armExtend1.getCurrentPosition() > var) {
                armExtend1.setPower(-0.3);
                armExtend2.setPower(-0.3);
            } else {
                armExtend1.setPower(0.3);
                armExtend2.setPower(0.3);
            }
        }
        else {
            if (armRotate.getCurrentPosition() > 800) {
                armExtend1.setPower(0.01);
                armExtend2.setPower(0.01);
            } else {
                armExtend1.setPower(0);
                armExtend2.setPower(0.01);
            }
        }
    }
    public void rotateArm(){
        controller.setPID(p,i,d);
        int armPos = armRotate.getCurrentPosition();
        double pid = controller.calculate(armPos,rotateGoal);
        if (armPos > 1500) {
            f = 0;
        } else {
            f = 0.08;
        }
        double ticks_in_degree = 5281.1 / 180.0;
        double ff = Math.cos(Math.toRadians(rotateGoal / ticks_in_degree)) * f;

        double power = pid + ff;
        armRotate.setPower(power);

        telemetry.addData("pos", armPos);
    }
}