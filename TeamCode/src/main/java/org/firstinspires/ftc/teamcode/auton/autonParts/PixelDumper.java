package org.firstinspires.ftc.teamcode.auton.autonParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class PixelDumper {
    LinearOpMode myOpMode;
    DcMotor arm;
    Servo dumper;

    double closedDumperPosition = 0.5;
    double dumperDumperPosition = 0.77;

    int closedArmPosition = 0;
    int dumperArmPosition = -145;

    public PixelDumper(LinearOpMode opMode) {myOpMode = opMode;}

    public void init() {
        arm = myOpMode.hardwareMap.get(DcMotor.class, "YellowPixel_Arm");
        dumper = myOpMode.hardwareMap.get(Servo.class, "YellowPixel_Dumper");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        dumper.setPosition(closedDumperPosition);
    }
    public void goToClosedPosition() {
        dumper.setPosition(closedDumperPosition);
        arm.setTargetPosition(closedArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(-0.5);
        while (myOpMode.opModeIsActive() && arm.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            myOpMode.idle();
        }
        arm.setPower(0.0);

        dumper.setPosition(closedDumperPosition);
    }
    public void goToScoringPosition() {
        arm.setTargetPosition(dumperArmPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        while (myOpMode.opModeIsActive() && arm.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            myOpMode.idle();
        }
        myOpMode.sleep(200);
        dumper.setPosition(dumperDumperPosition);
        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        arm.setPower(0.0);
    }
}
