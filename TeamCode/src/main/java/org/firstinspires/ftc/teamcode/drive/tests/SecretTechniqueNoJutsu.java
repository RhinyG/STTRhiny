package org.firstinspires.ftc.teamcode.drive.tests;

import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.FULL;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.ZERO;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.INTAKEGROUND;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.OUTTAKEBACK;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABLEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABRIGHT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASELEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASERIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "Secret Technique No Jutsu")
public class SecretTechniqueNoJutsu extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);
        waitForStart();

        Crumblz.ClawPositions leftPos = RELEASELEFT;
        Crumblz.ClawPositions rightPos = RELEASERIGHT;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean RotateIntakeButton = gamepad1.a;
            boolean RotateOuttakeButton = gamepad1.b;
            double armRotatePower;
            boolean armRotatePositive = gamepad1.right_bumper;
            boolean armRotateNegative = gamepad1.left_bumper;

            boolean ExtendFullyButton = gamepad1.x;
            boolean ExtendNoneButton = gamepad1.y;

            boolean leftOpen = gamepad2.dpad_left;
            boolean leftClose = gamepad2.dpad_up;
            boolean rightOpen = gamepad2.dpad_right;
            boolean rightClose = gamepad2.dpad_down;

            if(armRotateNegative) {
                armRotatePower = -1;
            }
            else if(armRotatePositive) {
                armRotatePower = 1;
            }
            else {
                armRotatePower = 0;
            }

            double armExtendPower = gamepad1.right_trigger - gamepad1.left_trigger;


            if (leftOpen) {
                leftPos = RELEASELEFT;
            }
            if (leftClose) {
                leftPos = GRABLEFT;
            }
            if (rightOpen) {
                rightPos = RELEASERIGHT;
            }
            if (rightClose) {
                rightPos = GRABRIGHT;
            }

            drivetrain.RobotCentric(1);
            if(gamepad2.a){
                arm.elbow.setPosition(0.8);
            } else {
                arm.updateElbow();
            }
            if(gamepad2.right_stick_button){
                arm.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(gamepad2.left_stick_button){
                arm.armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            arm.updateClaw(leftPos,rightPos);
            arm.secretRotate(armRotatePower,telemetry);
            arm.secretSlide(armExtendPower, telemetry);
            telemetry.update();
        }
    }
}