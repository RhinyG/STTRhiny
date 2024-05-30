package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robotParts.movement.Localization.localization;

import java.util.List;

@Config
@TeleOp(name = "Mecanum drive Test", group = "Tests")
public class MecanumDriveTest extends LinearOpMode {
    double[] position;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Vector2d vector = new Vector2d(gamepad1.left_stick_x,-gamepad1.left_stick_y);
            drive.drive(vector,0);
            telemetry.update();
        }
    }
}