package org.firstinspires.ftc.teamcode.auton.TwentyFivePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsRight;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;

@Autonomous(name = "RedWing")
public class RedWing extends LinearOpMode {
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
                methods.driveX(3 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-83 + 0.5 * methods.robotLength_cm);
                methods.driveY(15);
                methods.driveX(-28);
                methods.driveY(-82);
                methods.rotateToHeading(90);
                methods.driveY(-140);
                methods.driveX(-20);
                methods.driveY(-100);
            } else if (finalPos == 1) {
                methods.driveX(-25.5 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-112 + methods.robotLength_cm);
                methods.driveY(15);
                methods.driveX(15 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-76);
                methods.rotateToHeading(90);
                methods.driveY(-160 - 0.5 * methods.robotLength_cm);
                methods.driveX(-20);
                methods.driveY(-100);
            } else if (finalPos == 2){
                methods.driveX(-23.5 + 0.5 * methods.robotWidth_cm);
                methods.driveY(-100 + 0.5 * methods.robotLength_cm);
                methods.rotateToHeading(95);
                methods.driveY(-42 + 0.5 * methods.robotLength_cm);
                methods.driveY(42 - 0.5 * methods.robotLength_cm);
                methods.driveX(55);
                methods.driveY(-160);
                methods.driveY(-100);
            }
            sleep(30000);
        }
    }
}
