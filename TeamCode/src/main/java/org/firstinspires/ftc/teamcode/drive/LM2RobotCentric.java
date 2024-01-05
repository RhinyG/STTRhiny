package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.THIRDLINE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "LM2 RobotCentric")
public class LM2TeleOp extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    CurrentOuttake outtake = new CurrentOuttake(this);
    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);

        CurrentOuttake.ArmHeight height = INTAKE;

        boolean buttonMode = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double slidePower = gamepad2.right_trigger - gamepad2.left_trigger;

            boolean intakeBtn = gamepad2.x;
            boolean low = gamepad2.a;
            boolean mid = gamepad2.b;
            boolean high = gamepad2.y;

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

            drivetrain.RobotCentric();
            outtake.updateSlide(buttonMode, slidePower, height, telemetry);
            telemetry.addData("Slide Position", outtake.slides.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.addData("LeftPos", outtake.leftRotate.getPosition());
            telemetry.addData("RightPos", outtake.rightRotate.getPosition());
            telemetry.update();
        }
    }
}