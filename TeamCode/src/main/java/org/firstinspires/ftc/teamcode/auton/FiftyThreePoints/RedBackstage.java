package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVTrussIsLeft;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.INTAKE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.BOTTOM;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.FIRSTLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ArmHeight.SECONDLINE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.INTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.MOVEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.RotatePositions.OUTTAKEPOS;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ClawPositions.RELEASE;
import static org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake.ClawPositions.GRAB;

@Autonomous(name = "53?")
public class RedBackstage extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);
    CurrentOuttake slides = new CurrentOuttake();

    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap);
        methods.calibrateEncoders();
        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();

        waitForStart();

        if (opModeIsActive()) {
            slides.goToHeight(FIRSTLINE.getPosition(), telemetry);
            telemetry.addData("arm", slides.slides.getCurrentPosition());
            telemetry.addData("arm goal", FIRSTLINE.getPosition());
            telemetry.addLine(String.valueOf(FIRSTLINE.getPosition()));
            telemetry.addData("arm power", slides.slides.getPower());//            int finalPos = camera.pos;
//            telemetry.addData("localPos", camera.pos);
//            if (finalPos == 0) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(-90);
//                methods.driveY(-40 + 0.5 * methods.robotLength_cm);
//                methods.driveY(40 - 0.5 * methods.robotLength_cm);
//                methods.driveX(-60 + 0.5 * methods.robotWidth_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-80 + 0.5 * methods.robotLength_cm);
//            } else if (finalPos == 1) {
//                methods.driveX(25.5 - 0.5 * methods.robotWidth_cm);
//                methods.driveY(-112 + methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-60);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-30);
//            } else if (finalPos == 2){
//                methods.driveX(-0.5 * methods.robotWidth_cm);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.driveY(30);
//                methods.driveX(-30);
//                methods.driveY(-90 + 0.5 * methods.robotLength_cm);
//                methods.rotateToHeading(90);
//                methods.driveY(-25);
//            }
            sleep(30000);
        }
    }
}

