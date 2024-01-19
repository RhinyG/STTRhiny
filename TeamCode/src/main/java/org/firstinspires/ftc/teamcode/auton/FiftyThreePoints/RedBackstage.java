package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;

@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    PixelManipulation slides = new PixelManipulation(this);

    public void runOpMode() {
        methods.init(hardwareMap);
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
                methods.driveY(-110);
                methods.rotateToHeading(90);
                methods.driveY(-45);
                methods.driveY(40);
                methods.rotateToHeading(-90);
                methods.driveY(-152);
                methods.driveX(20);
//                slides.dropYellowPixel();
                methods.driveY(-10, 1, telemetry);
                methods.driveY(10);
                methods.driveX(-90);
            } else if (finalPos == 1) {
                methods.driveX(-10);
                methods.driveY(-115);
                methods.driveY(20);
                methods.rotateToHeading(-90);
                methods.driveY(-125);
                methods.driveX(5);
//                slides.dropYellowPixel();
                methods.driveX(-80);
            } else if (finalPos == 2){
                methods.driveX(15);
                methods.driveY(-90);
                methods.rotateToHeading(-90);
                methods.driveY(-110);
//                slides.dropYellowPixel();
                methods.driveX(-70);
                methods.driveY(-20);
            }
            slides.autonGoToHeight(PixelManipulation.ArmHeight.INTAKE, 0, telemetry);
            sleep(30000);
        }
    }
}

