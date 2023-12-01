package org.firstinspires.ftc.teamcode.auton.TwentyFivePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    CurrentOuttake slides = new CurrentOuttake();

    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap);
        methods.calibrateEncoders();
        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();

        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(-90);
                methods.driveY(-30 + 0.5 * methods.robotLength_cm);
                methods.driveY(90 - 0.5 * methods.robotLength_cm);
                methods.driveX(60 + 0.5 * methods.robotWidth_cm);
                methods.rotateToHeading(90);
            } else if (finalPos == 1) {
                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
                methods.driveY(-112 + methods.robotLength_cm);
                methods.driveY(60);
                methods.driveX(-60);
                methods.rotateToHeading(90);
            } else if (finalPos == 2){
                methods.driveX(-0.5 * methods.robotWidth_cm);
                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
                methods.driveY(60);
                methods.driveX(-60);
                methods.rotateToHeading(90);
            }
            sleep(30000);
        }
    }
}

