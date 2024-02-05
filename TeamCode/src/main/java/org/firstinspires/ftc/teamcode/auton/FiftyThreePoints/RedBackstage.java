package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;

@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    PixelManipulation slides = new PixelManipulation(this);
    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap, telemetry);
        methods.resetEncoders();
        camera.findScoringPosition();
        slides.claw.setPosition(PixelManipulation.ClawPositions.AUTONSTART.getPosition());
        slides.updateElbow(PixelManipulation.ElbowPositions.AUTONSTART);
        slides.wrist.setPosition(0.68);
        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            telemetry.addData("localPos", camera.pos);
            if (finalPos == 0) {
                methods.driveY(-90);
                methods.rotateToHeading(90);
                methods.driveX(-20);
                methods.driveY(-25);
                methods.driveY(40);
                methods.driveX(35);
                methods.rotateToHeading(-90);
                methods.driveY(-140, 0.2, telemetry, 7000);
                slides.dropYellowPixel();
                methods.driveY(20);
                methods.driveX(-90);
            } else if (finalPos == 1) {
                methods.driveY(-110);
                methods.driveY(30);
                methods.rotateToHeading(-90);
                methods.driveY(-155, 0.2, telemetry, 6000);
                methods.driveY(-10, 0.2, telemetry, 2500);
                slides.dropYellowPixel();
                methods.driveY(20);
                methods.driveX(-80);
                methods.driveY(-30);
            } else if (finalPos == 2){
                methods.driveX(-12);
                methods.driveY(-90);
                methods.driveY(30);
                methods.rotateToHeading(-90);
                methods.driveY(-115, 0.2, telemetry, 7000);
                methods.driveX(10);
                methods.driveY(-4, 0.2, telemetry, 2000);
                slides.dropYellowPixel();
                methods.driveX(-60);
                methods.driveY(-30);
            }
            slides.autonGoToHeight(PixelManipulation.ArmHeight.INTAKE, 0, telemetry);
            sleep(30000);
        }
    }
}

