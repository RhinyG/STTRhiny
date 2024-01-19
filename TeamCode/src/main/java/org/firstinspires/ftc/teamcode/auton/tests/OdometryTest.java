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
            methods.driveX(90);
        }
    }
}