package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
import org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides;

import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ArmHeight.BOTTOM;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.RotatePositions.INTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.RotatePositions.MOVEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.RotatePositions.OUTTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ClawPositions.RELEASE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakeTwoSlides.ClawPositions.GRAB;

@TeleOp(name = "IGORGEBRUIKDEZE")
public class CurrentTeleOp extends LinearOpMode {
    DrivetrainAlex drivetrain = new DrivetrainAlex();
    OuttakeTwoSlides outtake = new OuttakeTwoSlides();

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor hook = hardwareMap.dcMotor.get("hook");

        OuttakeTwoSlides.ArmHeight height = INTAKE;
        OuttakeTwoSlides.ClawPositions clawPosition1 = GRAB;
        OuttakeTwoSlides.ClawPositions clawPosition2 = GRAB;
        OuttakeTwoSlides.RotatePositions rotatePosition1 = INTAKEPOS;

        boolean buttonMode = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
            double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;

            boolean release1 = gamepad2.right_bumper;
            boolean grab1 = gamepad2.left_bumper;
            boolean intakePos = gamepad2.dpad_left;
            boolean movePos = gamepad2.dpad_down;
            boolean outtakePos = gamepad2.dpad_right;
            boolean release2 = gamepad2.right_bumper;
            boolean grab2 = gamepad2.left_bumper;

            boolean intakeBtn = gamepad1.x;
            boolean low = gamepad1.a;
            boolean mid = gamepad1.b;
            boolean high = gamepad1.y;

            double hookPower = gamepad1.left_trigger - gamepad1.right_trigger;
            double intakePower = 0;

            if(gamepad1.left_bumper){
                intakePower = 1;
            } else if (gamepad1.right_bumper) {
                intakePower = -1;
            }

            DrivetrainAlex.maxSpeed = 1;

            if (intakeBtn) {
                buttonMode = true;
                height = INTAKE;
            } else if (low) {
                buttonMode = true;
                height = BOTTOM;
            } else if (mid) {
                buttonMode = true;
                height = FIRSTLINE;
            } else if (high) {
                buttonMode = true;
                height = SECONDLINE;
            }

            if (Math.abs(slidePower) > 0.1) {
                buttonMode = false;
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

            if(intakePos){
                rotatePosition1 = INTAKEPOS;
            } else if (movePos) {
                rotatePosition1 = MOVEPOS;
            } else if (outtakePos) {
                rotatePosition1 = OUTTAKEPOS;
            }

            intake.setPower(intakePower);
            hook.setPower(hookPower);
            drivetrain.drive(y, x, rotate);
            outtake.update(buttonMode, slidePower, height, telemetry);
            outtake.updateLeftClaw(clawPosition1);
            outtake.updateRightClaw(clawPosition2);
            outtake.updateRotate(rotatePosition1);
            telemetry.addData("Slide Position", outtake.slides.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("LeftPos", outtake.leftRotate.getPosition());
            telemetry.addData("RightPos", outtake.rightRotate.getPosition());
            telemetry.update();
        }
    }
}