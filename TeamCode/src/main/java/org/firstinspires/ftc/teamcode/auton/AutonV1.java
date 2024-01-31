//package org.firstinspires.ftc.teamcode.auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsLeft;
//import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
//
//@Autonomous(name = "AutonV1")
//public class AutonV1 extends LinearOpMode {
//    newAutonMethods methods = new newAutonMethods(this);
//    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
//    PixelManipulation slides = new PixelManipulation(this);
//
//    public void runOpMode() {
//        methods.init(hardwareMap);
//        slides.init(hardwareMap, telemetry);
//        methods.calibrateEncoders();
//
//        camera.findScoringPosition();
//        // kijk of je assen kloppen de XYZ van imu, anders doet ie niks
//        waitForStart();
//        if (opModeIsActive()) {
//            int finalPos = camera.pos;
//            telemetry.addData("localPos", camera.pos);
//            methods.rotateToHeading(8,0.3, telemetry);
//            methods.driveX(5, 0.3, telemetry);
//            if (finalPos == 0) {
//                methods.driveX(15, 0.3, telemetry);
//                methods.driveY(-20, 0.3, telemetry);
//                methods.driveY(-10, 0.3, telemetry);
////                telemetry.addData("Alex de stemmen", "youris moeder");
////                sleep(30000);
//            } else if (finalPos == 1) {
//                methods.driveY(-90,0.3, telemetry);
//                methods.driveY(-70,0.3, telemetry);
//                telemetry.addData("Alex de stemmen", "deans moeder");
//                sleep(30000);
//            } else if (finalPos == 2){
//                methods.driveY(-60, 0.3, telemetry);
//                methods.rotateToHeading(90, 0.2, telemetry);
//                methods.driveY(-30, 0.3, telemetry);
////                telemetry.addData("Alex de stemmen", "igors moeder");
////                sleep(30000);
//            }
//            // dit is check
////            methods.rotateToHeading(180,0.3, telemetry);
////            methods.rotateToHeading(180,0.3, telemetry);
//            methods.driveY(-20,0.3, telemetry);
//            if (finalPos == 1 || finalPos == 2) {}
//            //rest of shit
//            sleep(30000);
//        }
//    }
//}
