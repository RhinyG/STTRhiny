package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class FSMMecanumDrivetrain {
    public int state = 0;
    public int driveState = 0;
    public int rotateState = 0;
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;

    double current_target_heading = 0;
    public IMU imu;
    double WHEEL_RADIUS = 48;//mm
    double ODO_RADIUS = 17.5;//mm?
    double GEAR_RATIO = 1/13.7;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
    double rotateThreshold = 0.5;
    double odoMultiplier = (69.5/38.6);
    double CM_PER_TICK = (odoMultiplier * 2*Math.PI * GEAR_RATIO * WHEEL_RADIUS)/(TICKS_PER_ROTATION); //about 1/690

    double beginTime;
    double TimeElapsed;
    double OdoY_Pos;
    double OdoX_Pos;
    double tickY;
    double tickX;
    double dPosY;
    double dPosX;
    double dPos;
    double Kp = 0.05;
    double FWD;
    double STR;
    double ROT;
    double speed;
    double min_speed = 0.13;
    double remweg;
    double a = 1;
    double b = 1;
    double heading;
    double dHeading;

    public FSMMecanumDrivetrain(LinearOpMode opmode) {myOpMode = opmode;}
    Telemetry telemetry = myOpMode.telemetry;

    public void init(HardwareMap map) {
        imu = map.get(IMU.class, "imu");

        FrontL = map.get(DcMotor.class, "left_front");
        FrontR = map.get(DcMotor.class, "right_front");
        BackL = map.get(DcMotor.class, "left_back");
        BackR = map.get(DcMotor.class, "right_back");

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }
    //TODO: put these first versions in EN, then delete three methods if not necessary
    public void setFSMDrive(double x, double y, double max_speed) {
        calibrateEncoders();
        beginTime = System.currentTimeMillis();
        heading = current_target_heading;
        tickY = (int) (y * OURTICKS_PER_CM);
        tickX = (int) (x * OURTICKS_PER_CM);
        remweg = max_speed * 35000;
        updateDrive();
        state++;
    }
    public void runFSMDrive(double max_speed, double stopTime) {
        if ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {

            if (Math.abs(dPosY) < remweg) {
                a = dPosY / remweg;
            } else {
                a = 1;
            }
            if (Math.abs(dPosX) < remweg) {
                b = dPosX / remweg;
            } else {
                b = 1;
            }

            FWD = a * max_speed;
            speed = min_speed + a * (max_speed - min_speed);
            STR = b * max_speed;
            if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
                FWD = -FWD;
            }
            if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
                STR = -STR;
            }
            ROT = Kp * Math.abs(speed) * dHeading;

            telemetry.addData("tickY", tickY);
            telemetry.addData("PosY", OdoY_Pos / OURTICKS_PER_CM);
            telemetry.addData("dPosY", dPosY);
            telemetry.addData("tickX", tickX);
            telemetry.addData("PosX", OdoX_Pos / OURTICKS_PER_CM);
            telemetry.addData("dPosX", dPosX);
            telemetry.addData("speed", speed);
            telemetry.addData("a", a);
            telemetry.addData("dPos", dPos);
            telemetry.addData("CurrentHeading", getCurrentHeading());
            telemetry.addData("TargetHeading", heading);
            telemetry.addData("STR", STR);
            telemetry.addData("FWD", FWD);
            telemetry.addData("turn", ROT);
            telemetry.addData("time", runtime.milliseconds());
            telemetry.addData("FL power", FrontL.getPower());
            telemetry.addData("FR power", FrontR.getPower());
            telemetry.addData("BL power", BackL.getPower());
            telemetry.addData("BR power", BackR.getPower());

            FrontL.setPower(FWD + STR + ROT);
            FrontR.setPower(FWD - STR - ROT);
            BackL.setPower(FWD - STR + ROT);
            BackR.setPower(FWD + STR - ROT);
            updateDrive();
        } else {
            state++;
        }
    }
    public void endFSMDrive() {
        Stop();
        myOpMode.sleep(100);
        state++;
    }
    //TODO: documentation
    public void FSMDrive(double x, double y, double max_speed, double stopTime) {
        switch (driveState) {
            case 0:
                calibrateEncoders();
                beginTime = System.currentTimeMillis();
                heading = current_target_heading;
                tickY = (int) (y * OURTICKS_PER_CM);
                tickX = (int) (x * OURTICKS_PER_CM);
                remweg = max_speed * 35000;
                updateDrive();
                driveState++;
                break;
            case 1:
                if ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {

                    if (Math.abs(dPosY) < remweg) {
                        a = dPosY / remweg;
                    } else {
                        a = 1;
                    }
                    if (Math.abs(dPosX) < remweg) {
                        b = dPosX / remweg;
                    } else {
                        b = 1;
                    }

                    FWD = a * max_speed;
                    speed = min_speed + a * (max_speed - min_speed);
                    STR = b * max_speed;
                    if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
                        FWD = -FWD;
                    }
                    if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
                        STR = -STR;
                    }
                    ROT = Kp * Math.abs(speed) * dHeading;

                    telemetry.addData("tickY", tickY);
                    telemetry.addData("PosY", OdoY_Pos / OURTICKS_PER_CM);
                    telemetry.addData("dPosY", dPosY);
                    telemetry.addData("tickX", tickX);
                    telemetry.addData("PosX", OdoX_Pos / OURTICKS_PER_CM);
                    telemetry.addData("dPosX", dPosX);
                    telemetry.addData("speed", speed);
                    telemetry.addData("a", a);
                    telemetry.addData("dPos", dPos);
                    telemetry.addData("CurrentHeading", getCurrentHeading());
                    telemetry.addData("TargetHeading", heading);
                    telemetry.addData("STR", STR);
                    telemetry.addData("FWD", FWD);
                    telemetry.addData("turn", ROT);
                    telemetry.addData("time", runtime.milliseconds());
                    telemetry.addData("FL power", FrontL.getPower());
                    telemetry.addData("FR power", FrontR.getPower());
                    telemetry.addData("BL power", BackL.getPower());
                    telemetry.addData("BR power", BackR.getPower());

                    FrontL.setPower(FWD + STR + ROT);
                    FrontR.setPower(FWD - STR - ROT);
                    BackL.setPower(FWD - STR + ROT);
                    BackR.setPower(FWD + STR - ROT);
                    updateDrive();
                } else {
                    driveState++;
                }
                break;
            case 2:
                Stop();
                myOpMode.sleep(100);
                state++;
                break;
        }
    }
    public void updateDrive(){
        dHeading = getCurrentHeading() - heading;
        OdoY_Pos = FrontL.getCurrentPosition();
        OdoX_Pos = FrontR.getCurrentPosition();
        TimeElapsed = System.currentTimeMillis() - beginTime;
        dPosY = tickY - OdoY_Pos;
        dPosX = tickX - OdoX_Pos;
        dPos = Math.abs(dPosX) + Math.abs(dPosY);
    }
    public void FSMRotate(double target_heading, double speed) {
        switch (rotateState) {
            case 0:
                target_heading *= -1;
                double current_heading = getCurrentHeading();
                dHeading = target_heading - current_heading;
                double direction;
                rotateThreshold = 0.5;
                telemetry.addData("curHeading", current_heading);
                telemetry.addData("dHeading",dHeading);
                rotateState++;
                break;
            case 1:
                if (!(Math.abs(dHeading) < rotateThreshold) && myOpMode.opModeIsActive()) {
                    direction = checkDirection(dHeading);

                    if(dHeading < 10 * rotateThreshold) {
                        speed = 0.2;
                    }

                    FrontL.setPower(-speed * direction);
                    FrontR.setPower(speed * direction);
                    BackL.setPower((-speed * direction));
                    BackR.setPower(speed * direction);

                    current_heading = getCurrentHeading();
                    dHeading = target_heading - current_heading;
                    telemetry.addData("curHeading", current_heading);
                    telemetry.addData("dHeading",dHeading);
                } else {
                    rotateState++;
                }
                break;
            case 2:
                calibrateEncoders();
                Stop();
                current_target_heading = target_heading;
                state++;
                break;
        }
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void Stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }

    public double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (orientation.getYaw(AngleUnit.DEGREES));
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void calibrateEncoders() {
        FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}