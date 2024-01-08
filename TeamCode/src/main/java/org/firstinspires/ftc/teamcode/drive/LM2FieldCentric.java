package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "FieldCentric Test")
public class LM2FieldCentric extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                drivetrain.FieldCentric(0.5,"Red", telemetry);
            } else {
                drivetrain.FieldCentric(1.0,"Red", telemetry);
            }
            telemetry.update();
        }
    }
}