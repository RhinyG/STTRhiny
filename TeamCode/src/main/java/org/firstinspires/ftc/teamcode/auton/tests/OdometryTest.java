package org.firstinspires.ftc.teamcode.auton.tests;

import static org.firstinspires.ftc.teamcode.robotParts.PixelManipulation.ArmHeight.FIRSTLINE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.PixelManipulation;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    PixelManipulation slides = new PixelManipulation(this);

    @Override
    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap, telemetry);
        methods.calibrateEncoders();
        methods.resetYaw();

        waitForStart();
        if (opModeIsActive()){
            slides.claw.setPosition(PixelManipulation.ClawPositions.GRABONE.getPosition());
            slides.autonGoToHeight(PixelManipulation.ArmHeight.FIRSTLINE);
            slides.updateElbow(PixelManipulation.ElbowPositions.OUTTAKEPOS);
            sleep(500);
            slides.claw.setPosition(PixelManipulation.ClawPositions.RELEASE.getPosition());
            sleep(300);
            methods.driveY(0);
            slides.claw.setPosition(PixelManipulation.ClawPositions.GRABONE.getPosition());
            slides.updateElbow(PixelManipulation.ElbowPositions.INTAKEPOS);
            sleep(300);
            slides.autonGoToHeight(PixelManipulation.ArmHeight.INTAKE);
            sleep(30000);
        }
    }
}