package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

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
        waitForStart();
        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveY(-16);
                methods.driveX(30);
                methods.driveY(-90);
                methods.driveY(30);
                methods.rotateToHeading(90);
                methods.driveY(-115);
//                slides.dropYellowPixel();
                methods.driveX(60);
                methods.driveY(-30);
            } else if (finalPos == 1) {
                methods.driveY(-130);
                methods.driveY(20);
                methods.rotateToHeading(90);
                methods.driveY(-155);
                methods.driveX(-5);
//                slides.dropYellowPixel();
                methods.driveY(10);
                methods.driveX(80);
                methods.driveY(-30);
            } else if (finalPos == 2){
                methods.driveY(-90);
                methods.rotateToHeading(-90);
                methods.driveY(-25);
                methods.driveY(40);
                methods.rotateToHeading(90);
                methods.driveY(-140);
                methods.driveX(-15);
//                slides.dropYellowPixel();
                methods.driveY(10);
                methods.driveX(90);
            }
            sleep(30000);
        }
    }
}
