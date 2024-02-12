package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
@Disabled
@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()){
            drive.driveDean(50,-50);
            sleep(30000);
        }
    }
}