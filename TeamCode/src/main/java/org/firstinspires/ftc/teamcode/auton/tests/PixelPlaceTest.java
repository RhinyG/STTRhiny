//package org.firstinspires.ftc.teamcode.auton.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.OpenCVTrussIsLeft;
//import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
//import org.firstinspires.ftc.teamcode.robotParts.Outdated.PixelManipulation;
//
//@Disabled
//@Autonomous(name = "PixelPlaceTest")
//public class PixelPlaceTest extends LinearOpMode {
//    newAutonMethods methods = new newAutonMethods(this);
//    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
//    PixelManipulation outtake = new PixelManipulation(this);
//
//    public void runOpMode() {
//        methods.init(hardwareMap);
//        outtake.init(hardwareMap, telemetry);
//        methods.calibrateEncoders();
//        camera.findScoringPosition();
//
//        waitForStart();
//    }
//}
