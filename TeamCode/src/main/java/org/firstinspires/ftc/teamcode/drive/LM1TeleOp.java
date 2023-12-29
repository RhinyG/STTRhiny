package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.THIRDLINE;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.INTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.MOVEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.OUTTAKEPOS;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ClawPositions.RELEASE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ClawPositions.GRABONE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ClawPositions.GRABTWO;

@TeleOp(name = "IGORGEBRUIKDEZE")
public class LM1TeleOp extends LinearOpMode {
    DrivetrainAlex drivetrain = new DrivetrainAlex();
    CurrentOuttake outtake = new CurrentOuttake(this);

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo plane = hardwareMap.servo.get("plane");

        CurrentOuttake.ArmHeight height = INTAKE;
        CurrentOuttake.ClawPositions clawPosition = GRABONE;
        CurrentOuttake.RotatePositions rotatePosition = MOVEPOS;

        boolean buttonMode = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
            double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;

            boolean intakePos = gamepad2.dpad_left;
            boolean movePos = gamepad2.dpad_down;
            boolean outtakePos = gamepad2.dpad_right;

            boolean releaseTwo = gamepad2.left_bumper;
            boolean grab = gamepad2.right_bumper;
            boolean releaseOne = gamepad2.dpad_up;

            double slowMode = gamepad1.right_trigger;

            boolean intakeBtn = gamepad2.x;
            boolean low = gamepad2.a;
            boolean mid = gamepad2.b;
            boolean high = gamepad2.y;

            boolean planeLaunch = gamepad1.x;
            boolean planeReset = gamepad1.y;

            double intakePower = 0;

            if(gamepad1.left_bumper){
                intakePower = 0.2;
            } else if (gamepad1.right_bumper) {
                intakePower = -0.7;
            }

            DrivetrainAlex.maxSpeed = 1;

            if (intakeBtn) {
                buttonMode = true;
                height = INTAKE;
            } else if (low) {
                buttonMode = true;
                height = FIRSTLINE;
            } else if (mid) {
                buttonMode = true;
                height = SECONDLINE;
            } else if (high) {
                buttonMode = true;
                height = THIRDLINE;
            }

            if (Math.abs(slidePower) > 0.1) {
                buttonMode = false;
            }

            if(releaseTwo){
                clawPosition = RELEASE;
            } else if (releaseOne) {
                clawPosition = GRABONE;
            } else if (grab) {
                clawPosition = GRABTWO;
            }

            if(intakePos){
                rotatePosition = INTAKEPOS;
            } else if (movePos) {
                rotatePosition = MOVEPOS;
            } else if (outtakePos) {
                rotatePosition = OUTTAKEPOS;
            }

            if(planeLaunch){
                plane.setPosition(0.55);
            } else if (planeReset) {
                plane.setPosition(0);
            }

            intake.setPower(intakePower);
            drivetrain.drive(y, x, rotate, slowMode);
            outtake.updateSlide(buttonMode, slidePower, height, telemetry);
            outtake.claw.setPosition(clawPosition.getPosition());
            outtake.updateRotate(rotatePosition);
            telemetry.addData("Slide Position", outtake.slides.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("LeftPos", outtake.leftRotate.getPosition());
            telemetry.addData("RightPos", outtake.rightRotate.getPosition());
            telemetry.update();
        }
    }
}