package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
import org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence;

import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.BOTTOM;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.INTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.MOVEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.RotatePositions.OUTTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ClawPositions.RELEASE;
import static org.firstinspires.ftc.teamcode.robotParts.OuttakePlusSequence.ClawPositions.GRAB;

@TeleOp
public class SlideSequenceTest extends LinearOpMode {
    DrivetrainAlex drivetrain = new DrivetrainAlex();
    OuttakePlusSequence outtake = new OuttakePlusSequence();
    int leftClawVar; // 1 = release, 2 = rotate
    int leftRotateVar; //1 = intakePos, 2 = movePos, 3 = outtakePos

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap);
        outtake.init(hardwareMap);
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        OuttakePlusSequence.ArmHeight height = INTAKE;
        OuttakePlusSequence.ClawPositions clawPosition = GRAB;
        OuttakePlusSequence.RotatePositions rotatePosition = INTAKEPOS;

        boolean buttonMode = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
            double rotate = -gamepad1.left_stick_x;
            double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;

            boolean release = gamepad1.right_bumper;
            boolean grab = gamepad1.left_bumper;
            boolean intakePos = gamepad1.dpad_down;
            boolean movePos = gamepad1.dpad_left;
            boolean outtakePos = gamepad1.dpad_right;
            boolean sequence = gamepad1.dpad_up;

            boolean intakeBtn = gamepad1.x;
            boolean low = gamepad1.a;
            boolean mid = gamepad1.b;
            boolean high = gamepad1.y;

            DrivetrainAlex.maxSpeed = 1;

            if (intakeBtn) {
                buttonMode = true;
                height = INTAKE;
            } else if (low) {
                buttonMode = true;
                height = BOTTOM;
            } else if (mid) {
                buttonMode = true;
                height = FIRSTLINE;
            } else if (high) {
                buttonMode = true;
                height = SECONDLINE;
            }

            if (Math.abs(slidePower) > 0.1) {
                buttonMode = false;
            }

            else {
                intake.setPower(0);
            }

            if(release){
                clawPosition = RELEASE;
            } else if (grab) {
                clawPosition = GRAB;
            }

            if(intakePos){
                rotatePosition = INTAKEPOS;
            } else if (movePos) {
                rotatePosition = MOVEPOS;
            } else if (outtakePos) {
                rotatePosition = OUTTAKEPOS;
            }

            drivetrain.drive(y, x, rotate);
            outtake.update(buttonMode, slidePower, height, telemetry);
            outtake.updateLeftClaw(clawPosition);
            outtake.updateLeftRotate(rotatePosition);
            telemetry.addData("Slide Position", outtake.slideLeft.getCurrentPosition());
            telemetry.addData("Slide Power", slidePower);
            telemetry.update();
        }
    }
}