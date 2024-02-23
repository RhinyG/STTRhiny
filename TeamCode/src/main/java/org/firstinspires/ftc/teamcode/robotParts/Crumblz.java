package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Crumblz extends RobotPart {
//TODO: explain variables
    private final LinearOpMode myOpMode;
    public Servo clawLeft;
    public Servo clawRight;
    public Servo elbow;
    public DcMotorEx armExtend;
    public DcMotorEx armRotate;
    public int state;
//TODO: explain enumerators
    public enum ArmRotatePos {
        INTAKEGROUND(0),
        OUTTAKEFRONT(1300),
        OUTTAKEBACK(3050);

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
        INTAKEPOS(0.075),//0.15
        STACKFIVEPOS(0.095),
        OUTTAKEFRONTSIDEPOS(0.075), //.1
        OUTTAKEBACKSIDEPOS(0.76),
        FOLDPOS(0.8);


        private final double position;
        public double getPosition() {
            return this.position;
        }
        ElbowPositions(double position) {
            this.position = position;
        }
    }

    public enum ClawPositions {
        OPENLEFT(0.69),
        RELEASELEFT(0.50),
        GRABLEFT(0.32),
        OPENRIGHT(0.38),
        RELEASERIGHT(0.52),
        GRABRIGHT(0.7);

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
    }

    /**
     * This methods initialises the pixel manipulation mechanisms and sets all the directions and modes to their correct settings.
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     *            This can alternatively be done with myOpMode.hardwareMap.get but that's longer so we don't.
     *            It can also probably be done via the constructor but I haven't managed to do that yet.
     */
    public void init(HardwareMap map) {
        elbow = map.get(Servo.class,"elbow");
        clawLeft = map.get(Servo.class, "clawLeft");
        clawRight = map.get(Servo.class, "clawRight");

        elbow.setPosition(ElbowPositions.FOLDPOS.getPosition());
        clawLeft.setPosition(ClawPositions.GRABLEFT.getPosition());
        clawRight.setPosition(ClawPositions.GRABRIGHT.getPosition());

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

   //TODO: documentation
    public void updateClaw(ClawPositions positionLeft, ClawPositions positionRight) {
        clawLeft.setPosition(positionLeft.getPosition());
        clawRight.setPosition(positionRight.getPosition());
    }
    //TODO: documentation
    public void updateElbow(boolean fold) {
        double position;
        if (fold) {
            position = ElbowPositions.FOLDPOS.getPosition();
        } else {
            if (armRotate.getCurrentPosition() < 300) {
                position = ElbowPositions.INTAKEPOS.getPosition();
            } else if (armRotate.getCurrentPosition() < 2200) {
                position = ElbowPositions.OUTTAKEFRONTSIDEPOS.getPosition();
            } else {
                position = ElbowPositions.OUTTAKEBACKSIDEPOS.getPosition();
            }
        }
        elbow.setPosition(position);
    }

    //TODO: documentation
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

    //TODO: documentation
    public void updateSlide(boolean buttonMode, double power, Crumblz.ArmExtendPos height, Telemetry telemetry) {
        double distance = 0;
        if (buttonMode) {
            distance = slidesGoToHeight(height.getPosition(), 1.0, telemetry);
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

    //TODO: documentation
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

    //TODO: documentation
    public void updateRotate(boolean buttonMode, double power, Crumblz.ArmRotatePos height, int holdSlides, Telemetry telemetry) {
        double distance = 0;
        double slideHeight = armExtend.getCurrentPosition();
        if (buttonMode) {
            distance = rotateToPos(height.getPosition(), 1, telemetry);
            if(distance > 100){
                slidesGoToHeight(holdSlides,0.7, telemetry);
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
        int position = armExtend.getCurrentPosition();
        armExtend.setPower(power);
        armExtend.setPower(0);
        telemetry.addData("slide", position);
        telemetry.addData("slide power", armExtend.getPower());
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
        armExtend.setTargetPosition(var);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(armExtend.getCurrentPosition() > var) {
            armExtend.setPower(-0.5);
        } else {
            armExtend.setPower(0.5);
        }
        while (myOpMode.opModeIsActive() && armExtend.isBusy()) {
            myOpMode.idle();
        }
        armExtend.setPower(0.3);
    }

    //TODO: documentation
    public void FSMArm(int var, double speed){
        if (Math.abs(armRotate.getCurrentPosition() - var) > 100) {
            if (armRotate.getCurrentPosition() > var) {
                armRotate.setPower(-speed);
            } else {
                armRotate.setPower(speed);
            }
        } else if (Math.abs(armExtend.getCurrentPosition() - var) > 50) {
            if (armRotate.getCurrentPosition() > var) {
                armRotate.setPower(-0.01);
            } else {
                armRotate.setPower(0.01);
            }
        }
    }

    //TODO: documentation
    public void FSMSlide(int var, double speed){
        if (Math.abs(armExtend.getCurrentPosition() - var) > 175) {
            if (armExtend.getCurrentPosition() > var) {
                armExtend.setPower(-speed);
            } else {
                armExtend.setPower(speed);
            }
        } else if (Math.abs(armExtend.getCurrentPosition() - var) > 10) {
            if (armExtend.getCurrentPosition() > var) {
                armExtend.setPower(-0.3);
            } else {
                armExtend.setPower(0.3);
            }
        }
        else {
            //TODO: find a proper way to stop this shit
            if (armRotate.getCurrentPosition() > 800) {
                armExtend.setPower(0.01);
            } else {
                armExtend.setPower(0);
            }
        }
    }
    //TODO: FSM PIDF-controller
}