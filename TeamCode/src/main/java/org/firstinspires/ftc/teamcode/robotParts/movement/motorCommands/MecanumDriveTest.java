package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Config
@TeleOp(name = "Mecanum drive Test", group = "Tests")
public class MecanumDriveTest extends LinearOpMode {
    Vector2d one, two, three = new Vector2d();
    public static double a, b, c, d = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drive = new MecanumDrivetrain(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init(hardwareMap,false);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.unoptimizedDrive(drive.toPolar(gamepad1.left_stick_x,-gamepad1.left_stick_y),gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}