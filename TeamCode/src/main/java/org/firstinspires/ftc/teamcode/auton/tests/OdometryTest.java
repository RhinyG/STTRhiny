package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Disabled
@Autonomous(name = "OdomTest", group = "Tests")
public class OdometryTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);

    @Override
    public void runOpMode() {
        drive.initRobot();
        waitForStart();
        if (opModeIsActive()){
            drive.linearDrive(50,-50);
            sleep(30000);
        }
    }
}