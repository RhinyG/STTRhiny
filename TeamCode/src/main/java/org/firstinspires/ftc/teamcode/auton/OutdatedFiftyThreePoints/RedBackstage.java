package org.firstinspires.ftc.teamcode.auton.OutdatedFiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Outdated.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;

@Disabled
@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    PixelManipulation slides = new PixelManipulation(this);
    public void runOpMode() {
        methods.init();
        slides.init(hardwareMap, telemetry);
        methods.calibrateEncoders();
        camera.findScoringPosition();
        slides.claw.setPosition(PixelManipulation.ClawPositions.AUTONSTART.getPosition());
        slides.updateElbow(PixelManipulation.ElbowPositions.AUTONSTART);
        slides.wrist.setPosition(0.68);
        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.linearDriveY(-90);
                methods.linearRotateToHeading(90);
                methods.linearDriveX(-20);
                methods.linearDriveY(-25);
                methods.linearDriveY(40);
                methods.linearDriveX(35);
                methods.linearRotateToHeading(-90);
                methods.linearDriveY(-140, 0.2, 7000);
                slides.dropYellowPixel();
                methods.linearDriveY(20);
                methods.linearDriveX(-90);
            } else if (finalPos == 1) {
                methods.linearDriveY(-110);
                methods.linearDriveY(30);
                methods.linearRotateToHeading(-90);
                methods.linearDriveY(-155, 0.2, 6000);
                methods.linearDriveY(-10, 0.2, 2500);
                slides.dropYellowPixel();
                methods.linearDriveY(20);
                methods.linearDriveX(-80);
                methods.linearDriveY(-30);
            } else if (finalPos == 2){
                methods.linearDriveX(-12);
                methods.linearDriveY(-90);
                methods.linearDriveY(30);
                methods.linearRotateToHeading(-90);
                methods.linearDriveY(-115, 0.2, 7000);
                methods.linearDriveX(10);
                methods.linearDriveY(-4, 0.2, 2000);
                slides.dropYellowPixel();
                methods.linearDriveX(-60);
                methods.linearDriveY(-30);
            }
            slides.autonGoToHeight(PixelManipulation.ArmHeight.INTAKE, 0, telemetry);
            sleep(30000);
        }
    }
}

