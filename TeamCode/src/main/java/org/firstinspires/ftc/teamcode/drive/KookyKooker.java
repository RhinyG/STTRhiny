package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmExtendPos.ZERO;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.intakeGround;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ArmRotatePos.outtakeBack;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.grabLeft;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.grabRight;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.openLeft;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.openRight;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.releaseLeft;
import static org.firstinspires.ftc.teamcode.robotParts.Crumblz.ClawPositions.releaseRight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
@TeleOp(name = "LET THE KOOKYKOOKER KOOK")
public class KookyKooker extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    Servo plane, hookServo;
    DcMotor hook;

    boolean extendButtonMode = false,rotateButtonMode = false, foldToggle = false;
    int holdSlides = 0;
    Crumblz.ArmExtendPos extendPos = ZERO;
    Crumblz.ArmRotatePos rotatePos = intakeGround;
    Crumblz.ClawPositions leftPos = openLeft, rightPos = openRight;
    double leftTime = 0, rightTime = 0, foldTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam Left"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
        drivetrain.init(hardwareMap);
        arm.init();

        plane = hardwareMap.servo.get("plane");

        hook = hardwareMap.dcMotor.get("hook");
        hookServo = hardwareMap.servo.get("hookServo");
        hookServo.setPosition(0);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double armRotatePower = gamepad1.right_trigger-gamepad1.left_trigger;

            double armExtendPower = -gamepad1.right_stick_y;

            boolean leftToggle = gamepad1.right_bumper;
            boolean rightToggle = gamepad1.left_bumper;

            boolean RotateIntakeButton = gamepad1.a;
            boolean RotateOuttakeButton = gamepad1.b;

            boolean planeLaunch = gamepad2.dpad_left;
            boolean planeReset = gamepad2.dpad_right;

            double hookPower = gamepad2.right_trigger - gamepad2.left_trigger;

            hook.setPower(hookPower);

            if(planeLaunch){
                plane.setPosition(0.55);
            } else if (planeReset) {
                plane.setPosition(0);
            }

            if (RotateIntakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend1.getCurrentPosition();
                rotatePos = intakeGround;
            } else if (RotateOuttakeButton) {
                rotateButtonMode = true;
                holdSlides = arm.armExtend1.getCurrentPosition();
                rotatePos = outtakeBack;
            }
            if(Math.abs(armRotatePower) > 0.1) {rotateButtonMode = false;}

            if ((leftTime + 500) < System.currentTimeMillis()){
                if (leftToggle && leftPos == openLeft) {
                    leftPos = grabLeft;
                    leftTime = System.currentTimeMillis();
                } else if (leftToggle && (leftPos == grabLeft || leftPos == releaseRight)) {
                    leftPos = openLeft;
                    leftTime = System.currentTimeMillis();
                }
            }
            if(gamepad1.dpad_right){
                leftPos = releaseLeft;
            }
            if ((rightTime + 500) < System.currentTimeMillis()) {
                if (rightToggle && rightPos == openRight) {
                    rightPos = grabRight;
                    rightTime = System.currentTimeMillis();
                } else if (rightToggle && (rightPos == grabRight || rightPos == releaseRight)) {
                    rightPos = openRight;
                    rightTime = System.currentTimeMillis();
                }
            }
            if(gamepad1.dpad_left){
                rightPos = releaseRight;
            }
            if(gamepad1.y){
                arm.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.dpad_up) {
                hookServo.setPosition(0.35);
            } else if (gamepad2.dpad_down) {
                hookServo.setPosition(0);
            }

            if ((foldTime + 500) < System.currentTimeMillis() && gamepad2.right_bumper){
                foldTime = System.currentTimeMillis();
                foldToggle = !foldToggle;
            }

            if (!foldToggle) {
                arm.updateElbow();
            } else {
                arm.elbow.setPosition(Crumblz.ElbowPositions.foldPos.getPosition());
            }
            arm.updateClaw(leftPos,rightPos);
            arm.updateSlide(extendButtonMode,armExtendPower,extendPos);
            arm.updateRotate(rotateButtonMode, armRotatePower, rotatePos, holdSlides);
            //TODO: slowmode based on voltage sensor
            if (arm.armRotate.getCurrentPosition() < 2000){
                drivetrain.RobotCentric(-1, arm.armExtend1.getCurrentPosition() >= 500);
            } else {
                drivetrain.RobotCentric(1, arm.armExtend1.getCurrentPosition() >= 500);
            }
            telemetry.addData("rotatePower",armRotatePower);
            telemetry.addData("leftFront",drivetrain.FrontL.getPower());
            telemetry.addData("rightFront",drivetrain.FrontR.getPower());
            telemetry.addData("leftBack",drivetrain.BackL.getPower());
            telemetry.addData("rightBack",drivetrain.BackR.getPower());
            telemetry.update();
        }
    }
}