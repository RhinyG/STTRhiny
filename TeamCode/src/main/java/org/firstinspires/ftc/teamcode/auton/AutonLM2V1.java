//package org.firstinspires.ftc.teamcode.auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robotParts.OpenCVTrussIsLeft;
//import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
//
//@Autonomous(name = "RedBackstage")
//public class AutonLM2V1 extends LinearOpMode {
//    newAutonMethods methods = new newAutonMethods(this);
//    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
//    PixelManipulation slides = new PixelManipulation(this);
//
//    public void runOpMode() {
//        methods.init(hardwareMap);
//        slides.init(hardwareMap, telemetry);
//        methods.calibrateEncoders();
//        camera.findScoringPosition();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            int finalPos = camera.pos;
//            telemetry.addData("localPos", camera.pos);
//            if (finalPos == 0) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(-90);
//                methods.driveY(-28 + 0.5 * methods.robotLength_cm);
//                methods.driveY(90 - 0.5 * methods.robotLength_cm);
//                slides.autonGoToHeight(PixelManipulation.ArmHeight.FIRSTLINE);
//                slides.updateElbow();
//                methods.driveX(50 + 0.5 * methods.robotWidth_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(10);
//            } else if (finalPos == 1) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-112 + methods.robotLength_cm);
//                methods.driveY(53);
//                methods.driveX(-60);
//                methods.rotateToHeading(90);
//                methods.driveY(-30);
//            } else if (finalPos == 2){
//                methods.driveX(3 -0.5 * methods.robotWidth_cm);
//                methods.driveY(-82 + 0.5 * methods.robotLength_cm);
//                methods.driveY(50);
//                methods.driveX(-60);
//                methods.rotateToHeading(90);
//                methods.driveY(-10);
//            }
//            sleep(30000);
//        }
//    }
//}
//
