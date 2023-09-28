package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
import org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence;

import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.BOTTOM;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.INTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.MOVEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.OUTTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ClawPositions.RELEASE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ClawPositions.GRAB;

@TeleOp
public class SlideSequenceTest extends LinearOpMode {
    DrivetrainAlex drivetrain = new DrivetrainAlex();
    OuttakePlusSequence outtake = new OuttakePlusSequence();

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        OuttakePlusSequence.ArmHeight height1 = INTAKE;
        OuttakePlusSequence.ArmHeight height2= INTAKE;
        OuttakePlusSequence.ClawPositions clawPosition1 = GRAB;
        OuttakePlusSequence.ClawPositions clawPosition2 = GRAB;
        OuttakePlusSequence.RotatePositions rotatePosition1 = INTAKEPOS;
        OuttakePlusSequence.RotatePositions rotatePosition2 = INTAKEPOS;

        boolean buttonMode1 = false;
        boolean buttonMode2 = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
            double rotate = -gamepad1.left_stick_x;
            double slidePower1 = gamepad1.right_trigger - gamepad1.left_trigger;
            double slidePower2 = gamepad2.right_trigger - gamepad2.left_trigger;


            boolean release1 = gamepad1.right_bumper;
            boolean grab1 = gamepad1.left_bumper;
            boolean intakePos1 = gamepad1.dpad_down;
            boolean movePos1 = gamepad1.dpad_left;
            boolean outtakePos1 = gamepad1.dpad_right;
            boolean release2 = gamepad2.right_bumper;
            boolean grab2 = gamepad2.left_bumper;
            boolean intakePos2 = gamepad2.dpad_down;
            boolean movePos2 = gamepad2.dpad_left;
            boolean outtakePos2 = gamepad2.dpad_right;

            boolean intakeBtn1 = gamepad1.x;
            boolean low1 = gamepad1.a;
            boolean mid1 = gamepad1.b;
            boolean high1 = gamepad1.y;
            boolean intakeBtn2 = gamepad2.x;
            boolean low2 = gamepad2.a;
            boolean mid2 = gamepad2.b;
            boolean high2 = gamepad2.y;


            DrivetrainAlex.maxSpeed = 1;

            if (intakeBtn1) {
                buttonMode1 = true;
                height1 = INTAKE;
            } else if (low1) {
                buttonMode1 = true;
                height1 = BOTTOM;
            } else if (mid1) {
                buttonMode1 = true;
                height1 = FIRSTLINE;
            } else if (high1) {
                buttonMode1 = true;
                height1 = SECONDLINE;
            }

            if (intakeBtn2) {
                buttonMode2 = true;
                height2 = INTAKE;
            } else if (low2) {
                buttonMode2 = true;
                height2 = BOTTOM;
            } else if (mid2) {
                buttonMode2 = true;
                height2 = FIRSTLINE;
            } else if (high2) {
                buttonMode2 = true;
                height2 = SECONDLINE;
            }

            if (Math.abs(slidePower1) > 0.1) {
                buttonMode1 = false;
            }

            if (Math.abs(slidePower2) > 0.1) {
                buttonMode2 = false;
            }

            if(release1){
                clawPosition1 = RELEASE;
            } else if (grab1) {
                clawPosition1 = GRAB;
            }

            if(release2){
                clawPosition2 = RELEASE;
            } else if (grab2) {
                clawPosition2 = GRAB;
            }

            if(intakePos1){
                rotatePosition1 = INTAKEPOS;
            } else if (movePos1) {
                rotatePosition1 = MOVEPOS;
            } else if (outtakePos1) {
                rotatePosition1 = OUTTAKEPOS;
            }

            if(intakePos2){
                rotatePosition2 = INTAKEPOS;
            } else if (movePos2) {
                rotatePosition2 = MOVEPOS;
            } else if (outtakePos2) {
                rotatePosition2 = OUTTAKEPOS;
            }

            drivetrain.drive(y, x, rotate);
            outtake.updateLeft(buttonMode1, slidePower1, height1, telemetry);
            outtake.updateRight(buttonMode2,slidePower2,height2,telemetry);
            outtake.updateLeftClaw(clawPosition1);
            outtake.updateRightClaw(clawPosition2);
            outtake.updateLeftRotate(rotatePosition1);
            outtake.updateRightRotate(rotatePosition2);
            telemetry.addData("Slide Position", outtake.slideLeft.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower1);
            telemetry.update();
        }
    }
}