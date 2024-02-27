package org.firstinspires.ftc.teamcode.drive.tests;

import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.grabLeft;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.grabRight;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.openLeft;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.openRight;

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
        arm.init();
        waitForStart();

        Crumblz.ClawPositions leftPos = openLeft;
        Crumblz.ClawPositions rightPos = openRight;

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
                leftPos = openLeft;
            }
            if (leftClose) {
                leftPos = grabLeft;
            }
            if (rightOpen) {
                rightPos = openRight;
            }
            if (rightClose) {
                rightPos = grabRight;
            }

            drivetrain.RobotCentric(1, false);
            if(gamepad2.a){
                arm.elbow.setPosition(0.8);
            } else {
                arm.updateElbow();
            }
            if(gamepad2.right_stick_button){
                arm.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(gamepad2.left_stick_button){
                arm.armExtend1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            arm.updateClaw(leftPos,rightPos);
            arm.secretRotate(armRotatePower);
            arm.secretSlide(armExtendPower);
            telemetry.update();
        }
    }
}