package org.firstinspires.ftc.teamcode.auton.tests;
//TODO: import acmerobotics to access FTC dashboard
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robotParts.Outdated.SanderDrive;

import java.util.List;

//@Config
@Autonomous(name = "SanderTest", group = "Test")
public class SanderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        SanderDrive drive = new SanderDrive(this);
        drive.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            drive.drive(0,50, 0.3, 30, 20000);
            sleep(30000);
        }
    }
}