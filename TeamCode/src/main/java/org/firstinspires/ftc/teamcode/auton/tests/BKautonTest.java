package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
@Disabled
@Autonomous(name = "BK auton")
public class BKautonTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);
    Crumblz arm = new Crumblz(this);

    public void runOpMode() {
        drive.init(hardwareMap);
        arm.init(hardwareMap);
        drive.calibrateEncoders();
        drive.resetYaw();
//        camera.findScoringPosition(false);
        int finalPossy = 0;
//        while (!isStarted()){
//            if (gamepad1.dpad_left) {
//                finalPos = 0;
//            } else if (gamepad1.dpad_up) {
//                finalPos = 1;
//            } else if (gamepad1.dpad_right) {
//                finalPos = 2;
//            }
//            telemetry.addData("pos",finalPos);
//            telemetry.update();
//        }
        waitForStart();
        int finalPos = camera.pos;
        if (opModeIsActive()) {
//            telemetry.addData("localPos", camera.pos);
            if (finalPossy == 0) {
                drive.linearDrive(77,-59,0.7,telemetry,3000);
                arm.autonArm(2800);
                arm.elbow.setPosition(Crumblz.ElbowPositions.outtakeBackSidePos.getPosition());
                arm.autonSlide(550);
                drive.linearDrive(10,-25,0.4,telemetry,2500);
                arm.clawLeft.setPosition(Crumblz.ClawPositions.releaseLeft.getPosition());
                sleep(100);
                drive.linearDrive(-10,35,0.4,telemetry,2500);
                arm.autonSlide(0);
                arm.autonArm(0);
                arm.elbow.setPosition(Crumblz.ElbowPositions.intakePos.getPosition());
                arm.autonSlide(900);
                arm.clawRight.setPosition(Crumblz.ClawPositions.releaseRight.getPosition());
                arm.autonSlide(0);
                drive.linearDrive(-70,-30,0.5,telemetry,4000);
            } else if (finalPossy == 1) {

            } else if (finalPossy == 2){

            }
            sleep(30000);
        }
    }
}