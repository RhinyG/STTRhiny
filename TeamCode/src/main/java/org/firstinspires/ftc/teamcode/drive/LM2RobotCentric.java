package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ArmHeight.THIRDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ClawPositions.GRABONE;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ClawPositions.GRABTWO;
import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ClawPositions.RELEASE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "LM2 RobotCentric")
public class LM2RobotCentric extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    PixelManipulation outtake = new PixelManipulation(this);
    Servo plane;
    Servo hook;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap, telemetry);
        plane = hardwareMap.servo.get("plane");
        hook = hardwareMap.servo.get("hook");

        PixelManipulation.ArmHeight height = INTAKE;
        PixelManipulation.ClawPositions clawPosition = RELEASE;
        PixelManipulation.ElbowPositions elbowPosition = PixelManipulation.ElbowPositions.MOVEPOS;
        PixelManipulation.IntakeServoPositions intakeServoPostion = PixelManipulation.IntakeServoPositions.GROUNDPOS;

        boolean buttonMode = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double intakeMotorSuck = gamepad1.right_trigger;
            double intakeMotorSpit = gamepad1.left_trigger;

            double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;

            boolean slidesGrab = gamepad2.x;
            boolean slidesLow = gamepad2.a;
            boolean slidesMid = gamepad2.b;
            boolean slidesHigh = gamepad2.y;

            boolean intakeServoLow = gamepad1.left_bumper;
            boolean intakeServoHigh = gamepad1.right_bumper;

            boolean elbowIntakePos = gamepad2.dpad_left;
            boolean elbowMovePos = gamepad2.dpad_down;
            boolean elbowOuttakePos = gamepad2.dpad_right;

            double wristX = gamepad2.left_stick_x;
            double wristY = -gamepad2.left_stick_y;
//TODO idk of deze dpad up nuttig is voor teleop plus igor wil dpad up voor de chain movement
//            boolean clawGrabOne = gamepad2.dpad_up;
            boolean clawMoveChain = gamepad2.dpad_up;
            boolean clawGrabTwo = gamepad2.right_bumper;
            boolean clawRelease = gamepad2.left_bumper;

            boolean planeLaunch = gamepad1.x;
            boolean planeReset = gamepad1.y;

            boolean hookFold = gamepad1.a;
            boolean hookRelease = gamepad1.b;

            if (intakeMotorSuck > 0.1) {
                outtake.updateIntake(0.80);
            } else if (intakeMotorSpit > 0.1) {
                outtake.updateIntake(-0.5);
            } else {
                outtake.updateIntake(0.0);
            }

            if (slidesGrab) {
                buttonMode = true;
                height = INTAKE;
            } else if (slidesLow) {
                buttonMode = true;
                height = FIRSTLINE;
            } else if (slidesMid) {
                buttonMode = true;
                height = SECONDLINE;
            } else if (slidesHigh) {
                buttonMode = true;
                height = THIRDLINE;
            }

            if (Math.abs(slidePower) > 0.1) {
                buttonMode = false;
            }

            outtake.updateSlide(buttonMode, slidePower, height, telemetry);

            if (intakeServoLow) {
                intakeServoPostion = PixelManipulation.IntakeServoPositions.GROUNDPOS;
            } else if (intakeServoHigh) {
                intakeServoPostion = PixelManipulation.IntakeServoPositions.STACKPOS;
            }

            outtake.updateIntakeServo(intakeServoPostion);

            if (elbowIntakePos) {
                elbowPosition = PixelManipulation.ElbowPositions.INTAKEPOS;
            } else if (elbowMovePos) {
                elbowPosition = PixelManipulation.ElbowPositions.MOVEPOS;
            } else if (elbowOuttakePos) {
                elbowPosition = PixelManipulation.ElbowPositions.OUTTAKEPOS;
            }

            outtake.updateElbow(elbowPosition);

            outtake.updateWrist(wristX,wristY, telemetry);

            if (clawRelease) {
                clawPosition = RELEASE;
            } else if (clawGrabTwo) {
                clawPosition = GRABTWO;
            }

            if (clawMoveChain) {
                elbowPosition = PixelManipulation.ElbowPositions.CHAINPOS;
                sleep(90);
                outtake.autonGoToHeight(FIRSTLINE, 0.4, telemetry);
                elbowPosition = PixelManipulation.ElbowPositions.MOVEPOS;
            }

            outtake.claw.setPosition(clawPosition.getPosition());

            if(planeLaunch){
                plane.setPosition(0.55);
            } else if (planeReset) {
                plane.setPosition(0);
            }

            if(hookFold) {
                hook.setPosition(0.0);
            }else if (hookRelease) {
                hook.setPosition(1.0);
            }

            drivetrain.RobotCentric();

            telemetry.addData("Slide Position", outtake.slides.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("WristPos", outtake.wrist.getPosition());
            telemetry.addData("ElbowPos", outtake.elbow.getPosition());
            telemetry.update();
        }
    }
}