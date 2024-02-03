package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "Dean OpenCV Test", group = "Test")
public class DeanAtHomeTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        camera.findScoringPosition(true);
        waitForStart();
        if (opModeIsActive()){
            drive.driveY(50);
            drive.driveX(50);
        }
    }
}