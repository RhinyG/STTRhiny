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
        methods.resetYaw();
        camera.findScoringPosition();
        slides.claw.setPosition(PixelManipulation.ClawPositions.AUTONSTART.getPosition());
        slides.updateElbow(PixelManipulation.ElbowPositions.AUTONSTART);
        slides.wrist.setPosition(0.68);
        waitForStart();
        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveX(20);
                methods.driveY(-90);
                methods.driveY(30);
                methods.rotateToHeading(90);
                methods.driveY(-125, 0.2, telemetry,7000);
                methods.driveX(30);
                methods.driveY(-4, 0.2, telemetry, 3000);
                slides.dropYellowPixel();
                methods.driveX(60);
                methods.driveY(-30);
            } else if (finalPos == 1) {
                methods.driveY(-116);
                methods.driveY(30);
                methods.rotateToHeading(80);
                methods.driveY(-145, 0.2, telemetry, 7000);
                methods.driveX(-20);
                slides.dropYellowPixel();
                methods.driveY(20);
                methods.driveX(80);
                methods.driveY(-30);
            } else if (finalPos == 2){
                methods.driveY(-90);
                methods.rotateToHeading(-90);
                methods.driveX(30);
                methods.driveY(-25);
                methods.driveY(40);
                methods.driveX(-35);
                methods.rotateToHeading(90);
                methods.driveY(-135, 0.2, telemetry,7000);
                slides.dropYellowPixel();
                methods.driveY(20);
                methods.driveX(90);
                methods.driveY(-30);
            }
            sleep(30000);
        }
    }
}
