package org.firstinspires.ftc.teamcode.auton.OutdatedFiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Outdated.OpenCVTrussIsRight;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
@Disabled
@Autonomous(name = "BlueBackstage")
public class BlueBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsRight camera = new OpenCVTrussIsRight(this);
    PixelManipulation slides = new PixelManipulation(this);

    public void runOpMode() {
        methods.initRobot();
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
                methods.linearDriveX(20);
                methods.linearDriveY(-90);
                methods.linearDriveY(30);
                methods.linearRotateToHeading(90);
                methods.linearDriveY(-125, 0.2,7000);
                methods.linearDriveX(30);
                methods.linearDriveY(-4, 0.2, 3000);
                slides.dropYellowPixel();
                methods.linearDriveX(60);
                methods.linearDriveY(-30);
            } else if (finalPos == 1) {
                methods.linearDriveY(-116);
                methods.linearDriveY(30);
                methods.linearRotateToHeading(80);
                methods.linearDriveY(-145, 0.2, 7000);
                methods.linearDriveX(-20);
                slides.dropYellowPixel();
                methods.linearDriveY(20);
                methods.linearDriveX(80);
                methods.linearDriveY(-30);
            } else if (finalPos == 2){
                methods.linearDriveY(-90);
                methods.linearRotateToHeading(-90);
                methods.linearDriveX(30);
                methods.linearDriveY(-25);
                methods.linearDriveY(40);
                methods.linearDriveX(-35);
                methods.linearRotateToHeading(90);
                methods.linearDriveY(-135, 0.2,7000);
                slides.dropYellowPixel();
                methods.linearDriveY(20);
                methods.linearDriveX(90);
                methods.linearDriveY(-30);
            }
            sleep(30000);
        }
    }
}
