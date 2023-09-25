//comment 4
package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
import org.firstinspires.ftc.teamcode.robotParts.Outtake;

@TeleOp
public class SlideTest extends LinearOpMode {
    DrivetrainAlex drivetrain = new DrivetrainAlex();
    Outtake outtake = new Outtake();


    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        DcMotor hook = hardwareMap.dcMotor.get("slide");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x; // y direction is reversed
            double y = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            boolean open = gamepad1.a;
            boolean closed = gamepad1.b;
            boolean up = gamepad1.x;
            boolean down = gamepad1.y;

            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

            DrivetrainAlex.maxSpeed = 1;

            if(gamepad1.left_bumper) {
                hook.setPower(1);
            } else if (gamepad1.right_bumper) {
                hook.setPower(-1);
            }
            else {
                hook.setPower(0);
            }

            intake.setPower(intakePower);

            drivetrain.drive(y, x, rotate);
            outtake.updateLeftClaw(open, closed);
            outtake.updateLeftRotate(up,down);
        }
    }
}