package org.firstinspires.ftc.teamcode.robotParts;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class DifferentialDrivetrain {
    public DcMotorEx FrontB,FrontR,BackB,BackR;
    //TODO: test
    DcMotorEx[] DTMotors;
    String[] DTMotorNames = {"blue_front","red_front","blue_back","red_back"};
    private final LinearOpMode myOpMode;
    HardwareMap map;
    Telemetry telemetry;
    Gamepad gamepad1, gamepad2;
    public IMU imu;
    public static double pb = 0.001, ib, db, pr = 0.001, ir, dr;
    public static int blueHeadingGoal = 0, redHeadingGoal = 0;
    PIDController blueHeading = new PIDController(pb, ib, db), redHeading = new PIDController(pr,ir,dr);
    int TICKS_PER_ROTATION = 8192, dPosR,dPosB, redCurrentHeading;
    double x, y, r, gamepadTheta, pidR, pidB;
    public DifferentialDrivetrain(LinearOpMode opmode) {
        myOpMode = opmode;
        //TODO: find out why this NPE
        map = opmode.hardwareMap;
        telemetry = opmode.telemetry;
        //TODO: test for NPE
        gamepad1 = opmode.gamepad1;
        gamepad2 = opmode.gamepad2;
    }

    /**
     * This methods initialises the swerve drivetrain and the IMU and sets all the directions and modes to their correct settings.
     */
    public void init() {
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        DTMotors = new DcMotorEx[]{FrontB, FrontR, BackB, BackR};
        //TODO: test for NPE
        for (int i = 0; i < DTMotors.length; i++) {
            DTMotors[i] = myOpMode.hardwareMap.get(DcMotorEx.class,DTMotorNames[i]);
            if ((i + 2) % 2 == 0){
                DTMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                DTMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

//        FrontB = myOpMode.hardwareMap.get(DcMotorEx.class, "blue_front");
//        FrontR = myOpMode.hardwareMap.get(DcMotorEx.class, "red_front");
//        BackB = myOpMode.hardwareMap.get(DcMotorEx.class, "blue_back");
//        BackR = myOpMode.hardwareMap.get(DcMotorEx.class, "red_back");
//
//        FrontB.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
//        BackB.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackR.setDirection(DcMotorSimple.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }

    public void swerveSimple(){
        FrontB.setPower(1*gamepad2.left_stick_y);
        FrontR.setPower(-1*gamepad2.left_stick_x);
        BackB.setPower(1*gamepad2.right_stick_y);
        BackR.setPower(-1*gamepad2.right_stick_x);
    }

    public void swerveRobotCentric(){
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = Math.sqrt(x * x + y * y);
        redCurrentHeading = FrontR.getCurrentPosition();

        if (x >= 0 && y >= 0) {
            gamepadTheta = Math.atan(y / x);
        } else if (x<0) {
            gamepadTheta = Math.atan(y / x) + Math.PI;
        } else {
            gamepadTheta = Math.atan(y / x) + 2 * Math.PI;
        }

        if(r > 0.5){
            redHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(2*Math.PI)*TICKS_PER_ROTATION);
        }

        dPosR = redHeadingGoal - redCurrentHeading;

        if (dPosR < -0.5 * TICKS_PER_ROTATION) {
            redHeadingGoal -= TICKS_PER_ROTATION;
        } else if (dPosR > 0.5 * TICKS_PER_ROTATION) {
            redHeadingGoal += TICKS_PER_ROTATION;
        }

        redHeading.setPID(pr,ir,dr);
        pidR = redHeading.calculate(redCurrentHeading,redHeadingGoal);

        FrontR.setPower(pidR);
        BackR.setPower(pidR);
        telemetry.addData("r",r);
        telemetry.addData("x",x);
        telemetry.addData("y",y);
        telemetry.addData("theta",gamepadTheta);
        telemetry.addData("target",redHeadingGoal);
    }
    //TODO:documentation
    public void resetYaw() {
        imu.resetYaw();
    }
}