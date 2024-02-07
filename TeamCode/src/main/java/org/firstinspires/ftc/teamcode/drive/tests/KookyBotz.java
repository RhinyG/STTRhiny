package org.firstinspires.ftc.teamcode.drive.tests;

import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.FULL;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.ZERO;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.INTAKEGROUND;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.OUTTAKEBACK;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.OUTTAKEFRONT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABLEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABRIGHT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASELEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASERIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;

@TeleOp(name = "KOOKYBOTZ")
public class KookyBotz extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    boolean extendButtonMode = false;
    boolean rotateButtonMode = false;
    int holdSlides = 0;
    boolean holdSlideButton = false;

    Crumblz.ArmExtendPos extendPos = ZERO;
    Crumblz.ArmRotatePos rotatePos = INTAKEGROUND;
    Crumblz.ClawPositions leftPos = RELEASELEFT;
    Crumblz.ClawPositions rightPos = RELEASERIGHT;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean RotateIntakeButton = gamepad2.a;
            boolean RotateOuttakeButton = gamepad2.b;
            double armRotatePower = -gamepad2.left_stick_x;

            boolean ExtendFullyButton = gamepad1.x;
            boolean ExtendNoneButton = gamepad1.y;

            boolean leftOpen = gamepad2.dpad_left;
            boolean leftClose = gamepad2.dpad_up;
            boolean rightOpen = gamepad2.dpad_right;
            boolean rightClose = gamepad2.dpad_down;

            Crumblz.ClawPositions currentLeft = leftPos;
            Crumblz.ClawPositions currentRight = rightPos;

            if (RotateIntakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend.getCurrentPosition();
                rotatePos = INTAKEGROUND;
            } else if (RotateOuttakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend.getCurrentPosition();
                rotatePos = OUTTAKEBACK;
            }

            if (Math.abs(armRotatePower) > 0.1) {
                rotateButtonMode = false;
            }

            double IgorArmExtendPower = 0.7*(gamepad1.right_trigger - gamepad1.left_trigger);
            double DeanArmExtendPower = 0.7*(gamepad2.right_trigger - gamepad2.left_trigger);
            double armExtendPower;

            if (Math.abs(IgorArmExtendPower) > 0.1 && Math.abs(DeanArmExtendPower) > 0.1) {
                if (arm.armRotate.getCurrentPosition() > 3100){
                    armExtendPower = DeanArmExtendPower;
                } else {
                    armExtendPower = IgorArmExtendPower;
                }
            } else if (Math.abs(IgorArmExtendPower) > 0.1) {
                armExtendPower = IgorArmExtendPower;
            } else {
                armExtendPower = DeanArmExtendPower;
            }

            if (ExtendFullyButton) {
                extendButtonMode = true;
                extendPos = FULL;
            } else if (ExtendNoneButton) {
                extendButtonMode = true;
                extendPos = ZERO;
            }

            if (Math.abs(IgorArmExtendPower) > 0.1 || Math.abs(DeanArmExtendPower) > 0.1) {
                extendButtonMode = false;
            }

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

            arm.updateElbow();
            arm.updateClaw(leftPos,rightPos);
            arm.updateSlide(extendButtonMode,armExtendPower,extendPos,telemetry);
            arm.updateRotate(rotateButtonMode, armRotatePower, rotatePos, holdSlides, telemetry);
            if (arm.armRotate.getCurrentPosition() < 2000){
                drivetrain.RobotCentric(-1);
            } else {
                drivetrain.RobotCentric(1);
            }
            telemetry.update();
        }
    }
}