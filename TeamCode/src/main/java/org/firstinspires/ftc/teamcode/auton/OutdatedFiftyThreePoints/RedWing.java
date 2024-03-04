//package org.firstinspires.ftc.teamcode.auton.OutdatedFiftyThreePoints;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.OpenCVTrussIsRight;
//import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
//
//@Disabled
//@Autonomous(name = "RedWing")
//public class RedWing extends LinearOpMode {
//    newAutonMethods methods = new newAutonMethods(this);
//    OpenCVTrussIsRight camera = new OpenCVTrussIsRight(this);
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
//            if (finalPos == 0) {
//                methods.linearDriveY(-16);
//                methods.linearDriveX(40);
//                methods.linearDriveY(-90);
//                methods.linearDriveY(30);
//                methods.linearDriveX(-50);
//                methods.linearDriveY(-115);
//                methods.linearRotateToHeading(-90);
//                methods.linearDriveY(260);
//                methods.linearDriveX(-40);
//                methods.linearDriveY(-20);
//                slides.dropYellowPixel();
//                methods.linearDriveY(-20);
//                methods.linearDriveX(-110);
//                methods.linearDriveY(-30);
//            } else if (finalPos == 1) {
//                methods.linearDriveY(-122);
//                methods.linearDriveY(20);
//                methods.linearDriveX(90);
//                methods.linearDriveY(-100);
//                methods.linearRotateToHeading(-90);
//                methods.linearDriveY(260);
//                methods.linearDriveX(-70);
//                methods.linearDriveY(-20);
//                slides.dropYellowPixel();
//                methods.linearDriveY(20);
//                methods.linearDriveX(-80);
//                methods.linearDriveY(-30);
//            } else if (finalPos == 2){
//                methods.linearDriveY(-88.5);
//                methods.linearRotateToHeading(-90);
//                methods.linearDriveX(30);
//                methods.linearDriveX(-25);
//                methods.linearDriveY(40);
//                methods.linearDriveX(60);
//                methods.linearDriveY(-230);
//                methods.linearDriveX(-100);
//                methods.linearDriveY(-20);
//                slides.dropYellowPixel();
//                methods.linearDriveY(20);
//                methods.linearDriveX(-50);
//                methods.linearDriveY(-30);
//            }
//            sleep(30000);
//        }
//    }
//}
