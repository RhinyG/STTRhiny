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
    public int state = 0;
    public enum ArmRotatePos {
        INTAKEGROUND(0),
        OUTTAKEFRONT(1300),
        OUTTAKEBACK(3300);
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
        FULL(780);
        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmExtendPos(int position) {this.position = position;}
    }
    public enum ArmLimits {
        SLIDELOWER(0),
        SLIDEUPPER(900),
        ROTATELOWER(0),
        ROTATEUPPER(3050);
        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmLimits(int position) {
            this.position = position;
        }
    }
    public enum ElbowPositions {
        INTAKEPOS(0.1),//0.15
        OUTTAKEFRONTSIDEPOS(0.1),
        OUTTAKEBACKSIDEPOS(0.8),
        FOLDPOS(0.8);
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
        GRABLEFT(0.47),
        RELEASERIGHT(0.38),
        GRABRIGHT(0.58);
        private final double position;
        public double getPosition() {
            return this.position;
        }
        ClawPositions(double position) {
            this.position = position;
        }
    }
    public Crumblz(LinearOpMode opmode) {
        myOpMode = opmode;
        telemetry = myOpMode.telemetry;
        map = myOpMode.hardwareMap;
    }
    Telemetry telemetry;
    HardwareMap map;
    public void init() {
        elbow = map.get(Servo.class,"elbow");
        wrist = map.get(Servo.class, "wrist");
        clawLeft = map.get(Servo.class, "clawLeft");
        clawRight = map.get(Servo.class, "clawRight");

        wrist.setPosition(WristPositions.INTAKEPOS.getPosition());
        elbow.setPosition(ElbowPositions.FOLDPOS.getPosition());

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
    public void updateWrist(double x, double y) {
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
        if (armRotate.getCurrentPosition() < 300) {
            position = ElbowPositions.INTAKEPOS.getPosition();
        } else if (armRotate.getCurrentPosition() < 1600) {
            position = ElbowPositions.OUTTAKEFRONTSIDEPOS.getPosition();
        } else {
            position = ElbowPositions.OUTTAKEBACKSIDEPOS.getPosition();
        }
        elbow.setPosition(position);
    }
    public double slidesGoToHeight(int position, double power) {
        double margin = 50.0;
        double currentPos = armExtend.getCurrentPosition();
        double distance = Math.abs(currentPos - position);
        if (currentPos < position) {
            if (distance > margin) {
                armExtend.setPower(power);
            } else {
                armExtend.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPos > position) {
            if (distance > margin) {
                armExtend.setPower(-power);
            } else {
                armExtend.setPower(-power * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPos <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }
    public void updateSlide(boolean buttonMode, double power, Crumblz.ArmExtendPos height) {
        double distance = 0;
        if (buttonMode) {
            distance = slidesGoToHeight(height.getPosition(), 0.7);
            telemetry.addData("slide", armExtend.getCurrentPosition());
            telemetry.addData("slide goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("arm power", armExtend.getPower());
        } else {
            int position = armExtend.getCurrentPosition();
            boolean limitReached;

            limitReached = (position <= ArmLimits.SLIDELOWER.getPosition() && power <= 0) || (position >= ArmLimits.SLIDEUPPER.getPosition() && power >= 0);

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
    public double rotateToPos(int position, double power) {
        double margin = 60.0;
        double currentPos = armRotate.getCurrentPosition();
        double distance = Math.abs(currentPos - position);
        if (currentPos < position) {
            if (distance > margin) {
                armRotate.setPower(power);
            } else {
                armRotate.setPower(power * (distance/margin) * 0.4);
            }
            telemetry.addLine("up");
        } else if (currentPos > position) {
            if (distance > margin) {
                armRotate.setPower(-power);
            } else {
                armRotate.setPower(-power * (distance/margin) * 0.4);
            }
            telemetry.addLine("down");
        } else if (position == 0 && currentPos <= 0) {
            setPower(0);
        } else {
            setPower(0.01);
        }
        return distance;
    }
    public void updateRotate(boolean buttonMode, double power, Crumblz.ArmRotatePos height, int holdSlides) {
        double distance = 0;
        double slideHeight = armExtend.getCurrentPosition();
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

            limitReached = (position <= ArmLimits.ROTATELOWER.getPosition() && power <= 0) || (position >= ArmLimits.ROTATEUPPER.getPosition() && power >= 0);

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
    public void secretRotate(double power) {
        int position = armRotate.getCurrentPosition();
        armRotate.setPower(power);

        telemetry.addData("rotate", position);
        telemetry.addData("rotate power", armRotate.getPower());
    }
    public void secretSlide(double power) {
        int position = armExtend.getCurrentPosition();
        armExtend.setPower(power);
        armExtend.setPower(0);
        telemetry.addData("slide", position);
        telemetry.addData("slide power", armExtend.getPower());
    }
}