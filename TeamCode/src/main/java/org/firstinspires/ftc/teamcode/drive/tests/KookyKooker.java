package org.firstinspires.ftc.teamcode.drive.tests;

import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.FULL;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.ZERO;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.INTAKEGROUND;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.OUTTAKEBACK;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABLEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.GRABRIGHT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.OPENLEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.OPENRIGHT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASELEFT;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.RELEASERIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "LET THE KOOKYKOOKER KOOK")
public class KookyKooker extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    Servo plane;

    boolean extendButtonMode = false;
    boolean rotateButtonMode = false;
    int holdSlides = 0;
    boolean holdSlideButton = false;

    Crumblz.ArmExtendPos extendPos = ZERO;
    Crumblz.ArmRotatePos rotatePos = INTAKEGROUND;

    Crumblz.ClawPositions leftPos = OPENLEFT;
    Crumblz.ClawPositions rightPos = OPENRIGHT;
    double leftTime = 0;
    double rightTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);

        plane = hardwareMap.servo.get("plane");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double armRotatePower = gamepad1.right_trigger-gamepad1.left_trigger;

            double armExtendPower = 0.7*(-gamepad1.right_stick_y);

            boolean leftToggle = gamepad1.right_bumper;
            boolean rightToggle = gamepad1.left_bumper;

            boolean RotateIntakeButton = gamepad1.a;
            boolean RotateOuttakeButton = gamepad1.b;

            boolean planeLaunch = gamepad1.dpad_up;
            boolean planeReset = gamepad1.dpad_down;

            if(planeLaunch){
                plane.setPosition(0.55);
            } else if (planeReset) {
                plane.setPosition(0);
            }

            if (RotateIntakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend.getCurrentPosition();
                rotatePos = INTAKEGROUND;
            } else if (RotateOuttakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend.getCurrentPosition();
                rotatePos = OUTTAKEBACK;
            }

            if ((leftTime + 500) < System.currentTimeMillis()){
                if (leftToggle && leftPos == OPENLEFT) {
                    leftPos = GRABLEFT;
                    leftTime = System.currentTimeMillis();
                } else if (leftToggle && (leftPos == GRABLEFT || leftPos == RELEASERIGHT)) {
                    leftPos = OPENLEFT;
                    leftTime = System.currentTimeMillis();
                }
            }
            if(gamepad1.dpad_right){
                leftPos = RELEASELEFT;
            }
            if ((rightTime + 500) < System.currentTimeMillis()) {
                if (rightToggle && rightPos == OPENRIGHT) {
                    rightPos = GRABRIGHT;
                    rightTime = System.currentTimeMillis();
                } else if (rightToggle && (rightPos == GRABRIGHT || rightPos == RELEASERIGHT)) {
                    rightPos = OPENRIGHT;
                    rightTime = System.currentTimeMillis();
                }
            }
            if(gamepad1.dpad_left){
                rightPos = RELEASERIGHT;
            }
            if(gamepad1.y){
                arm.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            arm.updateElbow();
            arm.updateClaw(leftPos,rightPos);
            arm.updateSlide(extendButtonMode,armExtendPower,extendPos, telemetry);
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