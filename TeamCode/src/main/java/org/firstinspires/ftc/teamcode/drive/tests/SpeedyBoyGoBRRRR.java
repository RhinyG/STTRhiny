package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(group = "Tests")
public class SpeedyBoyGoBRRRR extends LinearOpMode {
    DcMotorEx left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class,"left_front");
        right = hardwareMap.get(DcMotorEx.class,"right_front");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            left.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x);
            right.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x);
        }
    }
}