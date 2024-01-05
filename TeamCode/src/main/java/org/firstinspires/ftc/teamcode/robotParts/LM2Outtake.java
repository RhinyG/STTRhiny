package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class LM2Outtake extends RobotPart {

    private final LinearOpMode myOpMode;

    public DcMotorEx slides;

    public Servo droneServo;
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
    public LM2Outtake(LinearOpMode opmode) {myOpMode = opmode;}
    public void init(HardwareMap map) {
        slides = map.get(DcMotorEx.class, "slides");
        droneServo = map.get(Servo.class, "droneServo");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // from ArmReza code seems do interact with RobotPart.java i don't know really
        motors.put("slideLeft", slides);
        resetEncoders();
    }
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
    public void autonGoToHeight(CurrentOuttake.ArmHeight height, Telemetry telemetry) {
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
    public void updateSlide(boolean btns, double power, CurrentOuttake.ArmHeight height, Telemetry telemetry) {
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

    public void updateDrone(double position) {
        double servoPosition = position;
        droneServo.setPosition(servoPosition);
    }

    public void autonGoToHeight(CurrentOuttake.ArmHeight height){autonGoToHeight(height, myOpMode.telemetry);}
}
