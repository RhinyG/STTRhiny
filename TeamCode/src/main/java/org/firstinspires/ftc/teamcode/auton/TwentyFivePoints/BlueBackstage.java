package org.firstinspires.ftc.teamcode.auton.TwentyFivePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsRight;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;

@Autonomous(name = "BlueBackstage")
public class BlueBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsRight camera = new OpenCVTrussIsRight(this);
    PixelManipulation slides = new PixelManipulation(this);

    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap, telemetry);
        methods.calibrateEncoders();

        camera.findScoringPosition();
        // kijk of je assen kloppen de XYZ van imu, anders doet ie niks
        waitForStart();
        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveX(5 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
                methods.driveY(15);
                //purple is placed
                methods.driveX(65);
                methods.rotateToHeading(-90);
                methods.driveX(40);

            } else if (finalPos == 1) {
                methods.driveX(-25.5 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-114 + methods.robotLength_cm);
                methods.driveY(15);
                //purple is placed
                methods.driveX(60 - 0.5 * methods.robotWidth_cm);
                methods.rotateToHeading(-90);
                methods.driveY(-70 + 0.5 * methods.robotLength_cm);
                methods.driveX(45);
                methods.driveY(-10);
            } else if (finalPos == 2){
                methods.driveX(-30 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-100 + 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(95);
                methods.driveY(-20 + 0.5 * methods.robotLength_cm);
                methods.driveY(110 - 0.5 * methods.robotLength_cm);
                //purple is placed
                methods.driveX(-60);
                methods.rotateToHeading(-90);
                sleep(1000);
                methods.driveY(-30);
            }
            sleep(30000);
        }
    }
}
