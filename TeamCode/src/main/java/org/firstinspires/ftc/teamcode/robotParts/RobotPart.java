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
    //TODO: getCurrentHeadingRadians
    //TODO: getCurrentHeadingDegrees
    //TODO: calibrateEncoders
    //TODO: stop
    //TODO: checkDirection
    //TODO: toPolar
    //TODO: toCardinal
}