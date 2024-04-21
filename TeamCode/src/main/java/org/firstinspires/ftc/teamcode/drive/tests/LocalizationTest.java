package org.firstinspires.ftc.teamcode.drive.tests;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
@TeleOp(name = "Localization Test")
public class LocalizationTest extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    double[] localization;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivetrain.RobotCentric(1, false);
            localization = drivetrain.arcLocalization();
            telemetry.addData("leftFront",drivetrain.FrontL.getPower());
            telemetry.addData("rightFront",drivetrain.FrontR.getPower());
            telemetry.addData("leftBack",drivetrain.BackL.getPower());
            telemetry.addData("rightBack",drivetrain.BackR.getPower());
            telemetry.addData("x",localization[0]);
            telemetry.addData("y",localization[1]);
            telemetry.addData("heading",localization[2]);
            telemetry.update();
        }
    }
}