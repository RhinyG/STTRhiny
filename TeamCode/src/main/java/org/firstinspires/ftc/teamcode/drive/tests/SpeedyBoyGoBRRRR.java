package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(group = "test")
public class SpeedyBoyGoBRRRR extends LinearOpMode {
    DcMotorEx left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class,"left_front");
        right = hardwareMap.get(DcMotorEx.class,"right_front");
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            left.setPower(gamepad1.left_trigger);
            right.setPower(gamepad1.right_trigger);
        }
    }
}