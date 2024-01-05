//// Can move slides, outtake servo's, intake and drive but not buttons.
//package org.firstinspires.ftc.teamcode.drive;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.DrivetrainAlex;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.OutdatedOuttake;
//@Disabled
//@TeleOp
//public class OutdatedDriveCode extends LinearOpMode {
//    DrivetrainAlex drivetrain = new DrivetrainAlex();
//    OutdatedOuttake outtake = new OutdatedOuttake();
//    int leftClawVar; // 1 = release, 2 = rotate
//    int leftRotateVar; //1 = intakePos, 2 = movePos, 3 = outtakePos
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drivetrain.init(hardwareMap);
//        outtake.init(hardwareMap);
//        DcMotor intake = hardwareMap.dcMotor.get("intake");
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            double y = gamepad1.left_stick_y;
//            double x = gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
//            double rotate = gamepad1.left_stick_x;
//            double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
//
//            boolean release = gamepad1.dpad_left;
//            boolean grab = gamepad1.dpad_right;
//            boolean intakePos = gamepad1.x;
//            boolean movePos = gamepad1.y;
//            boolean outtakePos = gamepad1.dpad_down;
//            boolean sequenceLow = gamepad2.a;
//
//            DrivetrainAlex.maxSpeed = 1;
//
//            if(gamepad1.left_bumper) {
//                intake.setPower(1);
//            } else if (gamepad1.right_bumper) {
//                intake.setPower(-1);
//            }
//            else {
//                intake.setPower(0);
//            }
//
//            if(release){
//                leftClawVar = 1;
//            } else if (grab) {
//                leftClawVar = 2;
//            }
//
//            if(intakePos){
//                leftRotateVar = 1;
//            } else if (movePos) {
//                leftRotateVar = 2;
//            } else if (outtakePos) {
//                leftRotateVar = 3;
//            }
//
//            if(sequenceLow){
//                outtake.outtakeSequence(1500, telemetry);
//            }
//
//            outtake.moveSlidesManually(slidePower);
//            drivetrain.drive(y, x, rotate);
//            outtake.updateLeftClaw(leftClawVar);
//            outtake.updateLeftRotate(leftRotateVar);
//            telemetry.addData("Slide Position", outtake.slideLeft.getCurrentPosition());
//            telemetry.addData("Slide Power", slidePower);
//            telemetry.update();
//        }
//    }
//}