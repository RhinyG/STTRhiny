package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.CoaxialDrivetrain;
@Config
@TeleOp(group = "test")
public class coaxial extends LinearOpMode {
    CoaxialDrivetrain drive = new CoaxialDrivetrain(this);
    @Override
    public void runOpMode() throws InterruptedException {
        drive.initRobot();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.fullDrivetrainSimple();
        }
    }
}