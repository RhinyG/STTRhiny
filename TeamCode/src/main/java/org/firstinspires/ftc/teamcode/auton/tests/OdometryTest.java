package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.Current;

import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;
import org.firstinspires.ftc.teamcode.robotParts.CurrentOuttake;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);
    CurrentOuttake slides = new CurrentOuttake(this);

    @Override
    public void runOpMode() {
        methods.init(hardwareMap);
        slides.init(hardwareMap);
        methods.calibrateEncoders();
        methods.resetIMU(hardwareMap);

        /**
         * check dit nog, zodra confirmation is over gebruik camera ipv distance sensor
         */
        waitForStart();
        if (opModeIsActive()){
            methods.rotateToHeading(90);
            telemetry.update();
            sleep(30000);
        }
    }
}