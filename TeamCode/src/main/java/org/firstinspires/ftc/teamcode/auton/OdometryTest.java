package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;

@Autonomous(name = "OdomTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    newAutonMethods methods = new newAutonMethods(this);

    @Override
    public void runOpMode() {

        methods.init(hardwareMap);
        methods.resetEncoders();

        methods.resetIMU(hardwareMap);

        //initCamera();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        /**
         * check dit nog, zodra confirmation is over gebruik camera ipv distance sensor
         */
//        while (!isStarted() && !isStopRequested())
//        {
//            findTeamProp();
//        }
        methods.driveXY(100,100,0.3);
        sleep(30000);
    }
}