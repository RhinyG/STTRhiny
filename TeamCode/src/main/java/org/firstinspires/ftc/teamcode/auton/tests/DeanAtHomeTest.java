//package org.firstinspires.ftc.teamcode.auton.tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
//import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
//
//@Autonomous(name = "Dean OpenCV Test", group = "Tests")
//public class DeanAtHomeTest extends LinearOpMode {
//    newAutonMethods drive = new newAutonMethods(this);
//    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);
//
//    @Override
//    public void runOpMode() {
//        drive.init(hardwareMap);
//        camera.findScoringPosition(OpenCVTeamPropDetection.robotPositions.RedBackstage, hardwareMap);
//        waitForStart();
//        if (opModeIsActive()){
//            drive.linearDriveY(50);
//            drive.linearDriveX(50);
//        }
//    }
//}