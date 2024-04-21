package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

//TODO: rename
public abstract class RobotPart extends LinearOpMode {//TODO: extends OpMode, like PIDF_test
    protected Map<String, DcMotorEx> motors = new HashMap<>();
    protected Map<String, Servo> servos = new HashMap<>();
    protected Map<String, CRServo> crServos = new HashMap<>();
    IMU imu;
    DcMotorEx leftOdo,rightOdo,backOdo;
    final double Lx = 0,Ly = 15,
            Rx = 0,Ry = -15,
            Bx = -15,By = 0,
            WHEEL_RADIUS = 48,//mm
            GEAR_RATIO = 1/13.7,
            TICKS_PER_ROTATION = 8192,
            odoMultiplier = (69.5/38.6),
            ticksPerCM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS); //about 690 ticks per centimeter
    double oldX,oldY,oldTheta,currentX,currentY,currentTheta;
    //pos is in ticks, rest in cm or radian.
    double oldLPos, oldRPos, oldBPos, currentLPos, currentRPos, currentBPos, dL, dR, dB, relDX, relDY, rSTR, rFWD, dForward, dTheta, dStrafe;


    public void resetEncoders() {
        for (DcMotorEx motor : motors.values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setAllPowers(double power) {
        for (DcMotorEx motor : motors.values()) {
            motor.setPower(power);
        }
    }

    public void allCrPower(double power) {
        for (CRServo servo : crServos.values()) {
            servo.setPower(power);
        }
    }
    //TODO: documentation
    public void initIMU(HardwareMap map){
        imu = map.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }
    /**
     * ResetYaw is one of the new IMU methods, and it resets the yaw of the robot. When implemented correctly,
     * yaw is the only rotate axis you want to change.
     */
    public void resetYaw() {
        imu.resetYaw();
    }
    //TODO: documentation, EN
    public double[] oldToPolar(double[] cartesian) {
        return oldToPolar(cartesian[0],cartesian[1]);
    }
    //TODO: documentation, EN
    public double[] oldToPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double theta;
        if (x >= 0 && y >= 0) {
            theta = Math.atan(y / x);
        } else if (x<0) {
            theta = Math.atan(y / x) + Math.PI;
        } else {
            theta = Math.atan(y / x) + 2 * Math.PI;
        }
        return new double[]{r,theta};
    }
    //TODO: documentation, EN
    public double[] toPolar(double[] cartesian) {
        return toPolar(cartesian[0],cartesian[1]);
    }
    //TODO: documentation, EN
    public double[] toPolar(double x, double y) {
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y,x);
        return new double[]{r,theta};
    }
    //TODO: documentation, EN
    public double[] toCartesian(double[] polar) {
        return toCartesian(polar[0],polar[1]);
    }
    //TODO: documentation, EN
    public double[] toCartesian(double r, double theta) {
        double x = r * Math.cos(theta);
        double y = r * Math.sin(theta);
        return new double[]{x,y};
    }
    //TODO: documentation, EN
    //The Clueless linear odometry tracking equations
    public double[] linearLocalization() {
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;

        currentLPos = leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * Ly - dL * Ry)/(Ly - Ry); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(Ly - Ry);
        dStrafe = dB - Bx * dTheta;

        relDX = dForward;
        relDY = dStrafe;

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }
    //TODO: documentation, EN
    //btw it's a constant velocity arc localization.
    public double[] arcLocalization() {
        oldTheta = currentTheta;
        oldX = currentX;
        oldY = currentY;
        oldLPos = currentLPos;
        oldRPos = currentRPos;
        oldBPos = currentBPos;

        currentLPos = leftOdo.getCurrentPosition();
        currentRPos = rightOdo.getCurrentPosition();
        currentBPos = backOdo.getCurrentPosition();

        dR = (currentRPos - oldRPos) / ticksPerCM;
        dL = (currentLPos - oldLPos) / ticksPerCM;
        dB = (currentBPos - oldBPos) / ticksPerCM;

        dForward = (dR * Ly - dL * Ry)/(Ly - Ry); //Robot centric variable. If Ly = Ry, this can be simplified to (dR + dL) / 2.
        dTheta = (dR - dL)/(Ly - Ry);
        dStrafe = dB - Bx * dTheta;

        //If dTheta == 0, revert to linearLocalization because then you have a circle with infinite diameter, which it can't (because it divides by zero). Luckily a circle with infinite diameter is just a line.
        if (dTheta == 0) {
            relDX = dForward;
            relDY = dStrafe;
        } else {
            rFWD = dForward / dTheta;
            rSTR = dStrafe / dTheta;

            relDX = rFWD * Math.sin(dTheta) - rSTR * (1 - Math.cos(dTheta));
            relDY = rSTR * Math.sin(dTheta) + rFWD * (1 - Math.cos(dTheta));
        }

        currentTheta = oldTheta + dTheta;
        currentX = oldX + relDX * Math.cos(currentTheta) - relDY * Math.sin(currentTheta);
        currentY = oldY + relDY * Math.cos(currentTheta) - relDX * Math.sin(currentTheta);

        return new double[] {currentX, currentY, currentTheta};
    }
    //TODO: new pathfinding algorithm
    //TODO: getCurrentHeadingRadians
    //TODO: getCurrentHeadingDegrees
    //TODO: calibrateEncoders
    //TODO: stop
    //TODO: checkDirection
}
