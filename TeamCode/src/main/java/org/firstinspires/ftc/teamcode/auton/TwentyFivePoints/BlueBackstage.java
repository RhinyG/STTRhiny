package org.firstinspires.ftc.teamcode.auton.TwentyFivePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVTrussIsRight;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

@Autonomous(name = "BlueBackstage")
public class BlueBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsRight camera = new OpenCVTrussIsRight(this);
    CurrentOuttake slides = new CurrentOuttake();

    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap);
        methods.calibrateEncoders();

        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();
        // kijk of je assen kloppen de XYZ van imu, anders doet ie niks
        waitForStart();
        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveX(-2 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
                methods.driveY(15);
                //purple is placed
                methods.driveX(70);
                methods.rotateToHeading(-90);
                methods.driveX(-40);

            } else if (finalPos == 1) {
                methods.driveX(-25.5 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-112 + methods.robotLength_cm);
                methods.driveY(15);
                //purple is placed
                methods.driveX(60 - 0.5 * methods.robotWidth_cm);
                methods.rotateToHeading(-90);
                methods.driveY(-75 + 0.5 * methods.robotLength_cm);
                methods.driveX(35);
            } else if (finalPos == 2){
                methods.driveX(-20 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-100 + 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(95);
                methods.driveY(-10 + 0.5 * methods.robotLength_cm);
                methods.driveY(120 - 0.5 * methods.robotLength_cm);
                //purple is placed
                methods.driveX(-60);
                methods.rotateToHeading(-90);
                methods.driveY(-30);
            }
            sleep(30000);
        }
    }
}
