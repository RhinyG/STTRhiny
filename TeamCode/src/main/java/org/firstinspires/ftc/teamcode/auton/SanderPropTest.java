//package org.firstinspires.ftc.teamcode.auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVRandomizationTwo;
//
//@Autonomous
//public class SanderPropTest extends LinearOpMode {
//    OpenCVRandomizationTwo random = new OpenCVRandomizationTwo(this);
//    @Override
//    public void runOpMode() {
//        random.findScoringPosition();
//        telemetry.addData("Position", random.pos);
//        telemetry.addData("Left", random.leftavgfin);
//        telemetry.addData("Right", random.rightavgfin);
//        telemetry.addData("Color difference", random.color_difference);
//        telemetry.update();
//        waitForStart();
//        while (opModeIsActive()) {
//            telemetry.addLine("Blijf in init, want daar kun je de camera stream zien");
//            telemetry.update();
//        }
//
//    }
//}