package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Crumblz extends RobotPart {

    private final LinearOpMode myOpMode;
    public Servo clawLeft;
    public Servo clawRight;
    public Servo wrist;
    public Servo elbow;
    public DcMotorEx armExtend;
    public DcMotorEx armRotate;

    public enum ArmRotatePos {
        INTAKEGROUND(4000),
        OUTTAKEFRONT(2300),
        OUTTAKEBACK(800);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmRotatePos(int position) {
            this.position = position;
        }
    }

    public enum ArmExtendPos {
        ZERO(0),
        ONETHIRD(260),
        TWOTHIRDS(520),
        FULL(780);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmExtendPos(int position) {this.position = position;}
    }

    public enum ArmLimits {
        SLIDELOWER(0),
        SLIDEUPPER(950),
        ROTATELOWER(0),
        ROTATEUPPER(3900);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmLimits(int position) {
            this.position = position;
        }
    }

    public enum ElbowPositions {
        INTAKEPOS(0.13),//0.15
        OUTTAKEFRONTSIDEPOS(0.13),
        FOLDPOS(0);


        private final double position;
        public double getPosition() {
            return this.position;
        }
        ElbowPositions(double position) {
            this.position = position;
        }
    }

    public enum WristPositions {
        INTAKEPOS(0.63),//0.39 is servo to armExtend, 0.63 is servo away
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
        RELEASELEFT(0.69),
        GRABLEFT(0.555),
        RELEASERIGHT(0.38),
        GRABRIGHT(0.51);

        private final double position;

        public double getPosition() {
            return this.position;
        }

        ClawPositions(double position) {
            this.position = position;
        }
    }

    public Crumblz(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * Init
     * @param map otherwise NPE
     */
    public void init(HardwareMap map) {
        elbow = map.get(Servo.class,"elbow");
        wrist = map.get(Servo.class, "wrist");
        clawLeft = map.get(Servo.class, "clawLeft");
        clawRight = map.get(Servo.class, "clawRight");

        wrist.setPosition(WristPositions.INTAKEPOS.getPosition());

        armExtend = map.get(DcMotorEx.class, "armExtend");
        armRotate = map.get(DcMotorEx.class, "armRotate");

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotate.setDirection(DcMotorSimple.Direction.REVERSE);

        // from ArmReza code seems do interact with RobotPart.java I don't know really
        motors.put("slideLeft", armExtend);
        resetEncoders();
    }

    /**
     * This updates the claws to a (new) position
     */
    public void updateClaw(ClawPositions positionLeft, ClawPositions positionRight) {
        clawLeft.setPosition(positionLeft.getPosition());
        clawRight.setPosition(positionRight.getPosition());
    }

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
            } else if (theta < /*6.0/6.0**/Math.PI) {
                outtakePos = WristPositions.THREE;
            } else if (theta < 8.0/6.0*Math.PI) {
                outtakePos = WristPositions.FOUR;
            } else if (theta < 10.0/6.0*Math.PI) {
                outtakePos = WristPositions.FIVE;
            } else {
                outtakePos = WristPositions.SIX;
            }
            telemetry.addData("wristPos",outtakePos);

        } else if (myOpMode.gamepad2.left_stick_button) {
            outtakePos = WristPositions.INTAKEPOS;
            telemetry.addData("Going for neutral",outtakePos);
        }
    }

    //0.64 rotate 48, 0.707 rotate 790
    //servo pos = (0.707-0.64)/(790-48) * armRotate.getCurrentPosition + b
    // b = 0.707 - (0.707-0.64)/(790-48) * 790 = 0.635
    //servo pos = (0.707-0.64)/(790-48) * armRotate.getCurrentPosition + 0.635
    public void updateElbow() {
        double position;
        if (armRotate.getCurrentPosition() > 3800){
            position = ElbowPositions.INTAKEPOS.getPosition();
        } else if (armRotate.getCurrentPosition() < 150){
            position = ElbowPositions.FOLDPOS.getPosition();
        } else if (armRotate.getCurrentPosition() < 2000) {
            position = (0.707-0.64)/(790-48) * armRotate.getCurrentPosition() + 0.635;
        } else {
            position = ElbowPositions.OUTTAKEFRONTSIDEPOS.getPosition();
        }
        elbow.setPosition(position);
    }

    public double slidesGoToHeight(int position, double power, Telemetry telemetry) {
        double margin = 50.0;
        double currentPosLeft = armExtend.getCurrentPosition();
        double distance = Math.abs(currentPosLeft - position);
        if (currentPosLeft < position) {
            if (distance > margin) {
                armExtend.setPower(power);
            } else {
                armExtend.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPosLeft > position) {
            if (distance > margin) {
                armExtend.setPower(-power);
            } else {
                armExtend.setPower(-power * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPosLeft <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }

    public void updateSlide(boolean buttonMode, double power, Crumblz.ArmExtendPos height, Telemetry telemetry) {
        double distance = 0;
        if (buttonMode) {
            distance = slidesGoToHeight(height.getPosition(), 0.7, telemetry);
            telemetry.addData("slide", armExtend.getCurrentPosition());
            telemetry.addData("slide goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("arm power", armExtend.getPower());
        } else {
            int position = armExtend.getCurrentPosition();
            boolean limitReached;

            if ((position <= ArmLimits.SLIDELOWER.getPosition() && power <= 0)|| (position >= ArmLimits.SLIDEUPPER.getPosition() && power >= 0)) {
                limitReached = true;
            } else {
                limitReached = false;
            }

            if (!limitReached) {
                armExtend.setPower(power);
            } else {
                armExtend.setPower(0);
            }
            telemetry.addData("slide pos", position);
            telemetry.addData("slide power", armExtend.getPower());
            telemetry.addData("distance to goal", distance);
        }
    }
    public double rotateToPos(int position, double power, Telemetry telemetry) {
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
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }

    public void updateRotate(boolean buttonMode, double power, Crumblz.ArmRotatePos height, int holdSlides, Telemetry telemetry) {
        double distance = 0;
        if (buttonMode) {
            distance = rotateToPos(height.getPosition(), 1, telemetry);
            if(distance > 100){
                slidesGoToHeight(holdSlides,0.7,telemetry);
            }
            telemetry.addData("rotate", armRotate.getCurrentPosition());
            telemetry.addData("rotate goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("rotate power", armRotate.getPower());
        } else {
            int position = armRotate.getCurrentPosition();
            boolean limitReached;

            if ((position <= ArmLimits.ROTATELOWER.getPosition() && power <= 0)|| (position >= ArmLimits.ROTATEUPPER.getPosition() && power >= 0)) {
                limitReached = true;
            } else {
                limitReached = false;
            }

            if (!limitReached) {
                armRotate.setPower(power);
            } else {
                armRotate.setPower(0);
            }
            telemetry.addData("rotate", position);
            telemetry.addData("rotate power", armRotate.getPower());
            telemetry.addData("distance to goal", distance);
        }
    }

    public void secretRotate(double power, Telemetry telemetry) {
        int position = armRotate.getCurrentPosition();
        armRotate.setPower(power);

        telemetry.addData("rotate", position);
        telemetry.addData("rotate power", armRotate.getPower());
    }
    public void secretSlide(double power, Telemetry telemetry) {
        int position = armExtend.getCurrentPosition();
        armExtend.setPower(power);
        armExtend.setPower(0);
        telemetry.addData("slide", position);
        telemetry.addData("slide power", armExtend.getPower());
    }
}