package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()){
            drive.driveDean(0,100,0.3,telemetry,8000);
            drive.driveDean(100,0,0.3,telemetry,8000);
            drive.driveDean(0,-100,0.3,telemetry,8000);
            drive.driveDean(-100,0,0.3,telemetry,8000);
            sleep(30000);
        }
    }
}