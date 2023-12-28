package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.THIRDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.FIRSTLINE;

@Autonomous(name = "53?")
public class fiftyThreePoints extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    CurrentOuttake outtake = new CurrentOuttake(this);

    public void runOpMode() {
        methods.init(hardwareMap);
        outtake.init(hardwareMap);
        methods.calibrateEncoders();
        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();

        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            if (finalPos == 0) {
                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(-90);
                methods.driveY(-30 + 0.5 * methods.robotLength_cm);
                methods.driveY(90 - 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(90);
                sleep(500);
                outtake.autonGoToHeight(FIRSTLINE);
                sleep(500);
                methods.driveX(-10);
                outtake.updateRotate(CurrentOuttake.RotatePositions.OUTTAKEPOS);
                methods.driveY(-15, 0.2,telemetry);
                outtake.claw.setPosition(CurrentOuttake.ClawPositions.RELEASE.getPosition());
            } else if (finalPos == 1) {
                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
                methods.driveY(-112 + methods.robotLength_cm);
                methods.driveY(53);
                methods.driveX(-60);
                methods.rotateToHeading(90);
                methods.driveY(-30);
            } else if (finalPos == 2){
                methods.driveX(-0.5 * methods.robotWidth_cm);
                methods.driveY(-82 + 0.5 * methods.robotLength_cm);
                methods.driveY(50);
                methods.driveX(-60);
                methods.rotateToHeading(90);
                methods.driveY(-10);
            }
            sleep(30000);
            outtake.autonGoToHeight(FIRSTLINE, telemetry);
            telemetry.addData("DOnE","DONE");
            telemetry.update();
//            int finalPos = camera.pos;
//            telemetry.addData("localPos", camera.pos);
//            if (finalPos == 0) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(-90);
//                methods.driveY(-40 + 0.5 * methods.robotLength_cm);
//                methods.driveY(40 - 0.5 * methods.robotLength_cm);
//                methods.driveX(-60 + 0.5 * methods.robotWidth_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-80 + 0.5 * methods.robotLength_cm);
//            } else if (finalPos == 1) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-112 + methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-60);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-30);
//            } else if (finalPos == 2){
//                methods.driveX(-0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-30);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-25);
//            }
            sleep(30000);
        }
    }
}

