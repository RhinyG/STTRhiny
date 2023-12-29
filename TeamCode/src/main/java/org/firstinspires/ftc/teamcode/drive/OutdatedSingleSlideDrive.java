//package org.firstinspires.ftc.teamcode.drive;
//
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ArmHeight.BOTTOM;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ArmHeight.FIRSTLINE;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ArmHeight.INTAKE;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ArmHeight.SECONDLINE;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ClawPositions.GRAB;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.ClawPositions.RELEASE;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.RotatePositions.INTAKEPOS;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.RotatePositions.MOVEPOS;
//import static org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide.RotatePositions.OUTTAKEPOS;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.robotParts.DrivetrainAlex;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.OuttakeSingleSlide;
//
//@Disabled
//@TeleOp
//public class OutdatedSingleSlideDrive extends LinearOpMode {
//    DrivetrainAlex drivetrain = new DrivetrainAlex();
//    OuttakeSingleSlide outtake = new OuttakeSingleSlide();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        drivetrain.init(hardwareMap);
//        outtake.init(hardwareMap);
//        DcMotor intake = hardwareMap.dcMotor.get("intake");
//
//        OuttakeSingleSlide.ArmHeight height = INTAKE;
//        OuttakeSingleSlide.ClawPositions clawPosition1 = GRAB;
//        OuttakeSingleSlide.ClawPositions clawPosition2 = GRAB;
//        OuttakeSingleSlide.RotatePositions rotatePosition = INTAKEPOS;
//
//        boolean buttonMode = false;
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            double y = gamepad1.left_stick_y;
//            double x = -gamepad1.right_stick_x; //Drivetrain rotate, not rotate Servo
//            double rotate = -gamepad1.left_stick_x;
//            double slidePower = gamepad1.right_trigger - gamepad1.left_trigger;
//
//
//            boolean release1 = gamepad1.right_bumper;
//            boolean grab1 = gamepad1.left_bumper;
//            boolean release2 = gamepad2.right_bumper;
//            boolean grab2 = gamepad2.left_bumper;
//            boolean intakePos = gamepad1.dpad_left;
//            boolean movePos = gamepad1.dpad_down;
//            boolean outtakePos = gamepad1.dpad_right;
//
//            boolean intakeBtn1 = gamepad1.x;
//            boolean low1 = gamepad1.a;
//            boolean mid1 = gamepad1.b;
//            boolean high1 = gamepad1.y;
//
//            DrivetrainAlex.maxSpeed = 1;
//
//            if (intakeBtn1) {
//                buttonMode = true;
//                height = INTAKE;
//            } else if (low1) {
//                buttonMode = true;
//                height = BOTTOM;
//            } else if (mid1) {
//                buttonMode = true;
//                height = FIRSTLINE;
//            } else if (high1) {
//                buttonMode = true;
//                height = SECONDLINE;
//            }
//
//            if (Math.abs(slidePower) > 0.1) {
//                buttonMode = false;
//            }
//
//            if(release1){
//                clawPosition1 = RELEASE;
//            } else if (grab1) {
//                clawPosition1 = GRAB;
//            }
//
//            if(release2){
//                clawPosition2 = RELEASE;
//            } else if (grab2) {
//                clawPosition2 = GRAB;
//            }
//
//            if(intakePos){
//                rotatePosition = INTAKEPOS;
//            } else if (movePos) {
//                rotatePosition = MOVEPOS;
//            } else if (outtakePos) {
//                rotatePosition = OUTTAKEPOS;
//            }
//
//            drivetrain.drive(y, x, rotate);
//            outtake.updateSlides(buttonMode, slidePower, height, telemetry);
//            outtake.updateLeftClaw(clawPosition1);
//            outtake.updateRightClaw(clawPosition2);
//            outtake.updateRotate(rotatePosition);
//            telemetry.addData("Slide Position", outtake.slideLeft.getCurrentPosition());
//            telemetry.addData("Slide Power", slidePower);
//            telemetry.addData("LeftPos", outtake.leftRotate.getPosition());
//            telemetry.addData("RightPos", outtake.rightRotate.getPosition());
//            telemetry.update();
//        }
//    }
//}