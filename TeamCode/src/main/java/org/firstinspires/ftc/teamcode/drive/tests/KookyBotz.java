package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;

@TeleOp(name = "KOOKYBOTZ")
public class KookyBotz extends LinearOpMode {

    MecanumDrivetrain drivetrain = new MecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    boolean slideButtonMode = false;
    boolean rotateButtonMode = false;

    Crumblz.ArmExtendPos extendPos = Crumblz.ArmExtendPos.ZERO;
    Crumblz.ArmRotatePos rotatePos = Crumblz.ArmRotatePos.INTAKEGROUND;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        arm.init(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double armSlidePower = gamepad1.right_trigger - gamepad1.left_trigger;

            double armRotatePower;
            boolean armRotatePositive = gamepad1.right_bumper;
            boolean armRotateNegative = gamepad1.left_bumper;

            if(armRotateNegative) {
                armRotatePower = -1;
                rotateButtonMode = false;
            }
            else if(armRotatePositive) {
                armRotatePower = 1;
                rotateButtonMode = false;
            }
            else {
                armRotatePower = 0;
                rotateButtonMode = false;
            }

            drivetrain.RobotCentric();
//            arm.updateSlide(slideButtonMode,armSlidePower,extendPos,telemetry);
            arm.updateRotate(rotateButtonMode,armRotatePower,rotatePos,telemetry);
            telemetry.update();
        }
    }
}