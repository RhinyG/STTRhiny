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
    public Servo claw;
    public Servo wrist;
    public Servo elbow;
    public DcMotorEx armExtend;
    public DcMotorEx armRotate;

    public enum ArmRotatePos {
        INTAKEGROUND(-1150),
        INTAKESTACKTHREE(-1075),
        INTAKESTACKFIVE(-1000),
        OUTTAKE(500);

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
        ONETHIRD(300),
        TWOTHIRDS(600),
        FULL(1000);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmExtendPos(int position) {this.position = position;}
    }

    public enum ArmLimits {
        SLIDELOWER(0),
        SLIDEUPPER(1000),
        ROTATELOWER(0),
        ROTATEUPPER(2850);

        private final int position;
        public int getPosition() {
            return this.position;
        }
        ArmLimits(int position) {
            this.position = position;
        }
    }

    public enum ElbowPositions {
        INTAKEPOS(0.7475),
        MOVEPOSLOW(0.765),
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
        RELEASE(0.46),
        GRAB(0.365);

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
        claw = map.get(Servo.class, "claw");

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
    public void updateClaw(ClawPositions position) {
        claw.setPosition(position.getPosition());
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

    public void updateElbow(ElbowPositions position) {
        elbow.setPosition(position.getPosition());
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
            distance = slidesGoToHeight(height.getPosition(), 1, telemetry);
            telemetry.addData("slide", armExtend.getCurrentPosition());
            telemetry.addData("slide goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("arm power", armExtend.getPower());
        } else {
            int position = armExtend.getCurrentPosition();

            if (position <= ArmLimits.SLIDELOWER.getPosition() && power <= 0) {
                setPower(0);
            }
            else if (position >= ArmLimits.SLIDEUPPER.getPosition() && power >= 0) {
                setPower(0);
            } else {
                armExtend.setPower(power);
            }
            telemetry.addData("slide", position);
            telemetry.addData("slide power", armExtend.getPower());
            telemetry.addData("distance to goal", distance);
        }
    }
    public double rotateToPos(int position, double power, Telemetry telemetry) {
        double margin = 50.0;
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

    public void updateRotate(boolean buttonMode, double power, Crumblz.ArmRotatePos height, Telemetry telemetry) {
        double distance = 0;
        if (buttonMode) {
            distance = rotateToPos(height.getPosition(), 1, telemetry);
            telemetry.addData("rotate", armRotate.getCurrentPosition());
            telemetry.addData("rotate goal", height.getPosition());
            telemetry.addLine(String.valueOf(height));
            telemetry.addData("rotate power", armRotate.getPower());
        } else {
            int position = armRotate.getCurrentPosition();

            if (position <= ArmLimits.ROTATELOWER.getPosition() && power <= 0) {
                setPower(0);
            }
            else if (position >= ArmLimits.ROTATEUPPER.getPosition() && power >= 0) {
                setPower(0);
            } else {
                armRotate.setPower(power);
            }
            telemetry.addData("rotate", position);
            telemetry.addData("rotate power", armRotate.getPower());
            telemetry.addData("distance to goal", distance);
        }
    }
}