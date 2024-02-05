package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        drive.resetEncoders();
        waitForStart();
        if (opModeIsActive()){
            drive.driveSander(0,50);
        }
    }
}