package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;

@TeleOp(name = "FieldCentric Test")
public class FieldCentric extends LinearOpMode {
    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                drivetrain.FieldCentric(0.5, hardwareMap, telemetry);
            } else {
                drivetrain.FieldCentric(1.0,hardwareMap, telemetry);
            }
            telemetry.update();
        }
    }
}