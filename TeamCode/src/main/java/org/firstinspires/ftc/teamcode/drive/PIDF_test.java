package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PIDF_test extends OpMode {
    private PIDController controller;

    public static double p = 0.008, i = 0, d = 0;
    public static double f = 0.08;
    public static int target = 2900;
    private final double ticks_in_degree = 5281.1 / 180.0;

    private DcMotorEx arm;
    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        arm = hardwareMap.get(DcMotorEx.class, "armRotate");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            target = 1500;
        } else if (gamepad1.b) {
            target = 2000;
        } else if (gamepad1.x) {
            target = 2500;
        } else if (gamepad1.y) {
            target = 3000;
        }
        controller.setPID(p,i,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos,target);
        if (armPos > 1500) {
            f = 0;
        } else {
            f = 0.08;
        }
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        arm.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target",target);
        telemetry.update();
    }
}
