package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.sql.Time;
import java.util.Arrays;

public class newAutonMethods {
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;

    final public int robotLength_cm = 39;
    final public int robotWidth_cm = 40;
    final public double gravityConstant = 1.35;

    double current_target_heading = 0;
    public IMU imu;
    double WHEEL_RADIUS = 48;//mm
    double ODO_RADIUS = 17.5;//mm?
    double GEAR_RATIO = 1/13.7;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
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
    double speed;
    double min_speed = 0.13;
    double remweg;
    double a = 1;
    double b = 1;
    double heading;
    double dHeading;

    public newAutonMethods(LinearOpMode opmode) {myOpMode = opmode;}

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

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }
    public void driveDean(double x,double y, double max_speed, Telemetry telemetry, double stopTime) {
        calibrateEncoders();
        beginTime = System.currentTimeMillis();
        double turn;
        heading = current_target_heading;
        tickY = (int) (y * OURTICKS_PER_CM);
        tickX = (int) (x * OURTICKS_PER_CM);
        remweg = max_speed * 35000;
        updateDean(x,y);
        while ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {

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
            speed = min_speed + a * (max_speed-min_speed);
            STR = b * max_speed;
            if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
                FWD = -FWD;
            }
            if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
                STR = -STR;
            }
            turn = Kp*Math.abs(speed)*dHeading;

            telemetry.addData("tickY", tickY);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPosY", dPosY);
            telemetry.addData("tickX", tickX);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPosX", dPosX);
            telemetry.addData("speed", speed);
            telemetry.addData("a", a);
            telemetry.addData("dPos", dPos);
            telemetry.addData("CurrentHeading", getCurrentHeading());
            telemetry.addData("TargetHeading", heading);
            telemetry.addData("STR", STR);
            telemetry.addData("FWD", FWD);
            telemetry.addData("turn", turn);
            telemetry.addData("time", runtime.milliseconds());
            telemetry.addData("FL power", FrontL.getPower());
            telemetry.addData("FR power", FrontR.getPower());
            telemetry.addData("BL power", BackL.getPower());
            telemetry.addData("BR power", BackR.getPower());
            telemetry.update();

            FrontL.setPower(FWD + STR + turn);
            FrontR.setPower(FWD - STR - turn);
            BackL.setPower(FWD - STR + turn);
            BackR.setPower(FWD + STR - turn);
            updateDean(x,y);
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void updateDean(double x, double y){
        dHeading = getCurrentHeading() - heading;
        OdoY_Pos = FrontL.getCurrentPosition();
        OdoX_Pos = FrontR.getCurrentPosition();
        TimeElapsed = System.currentTimeMillis() - beginTime;
        dPosY = tickY - OdoY_Pos;
        dPosX = tickX - OdoX_Pos;
        dPos = Math.abs(dPosX) + Math.abs(dPosY);
    }

    public void driveY (double position){
        driveY(position,0.3, myOpMode.telemetry, 10000);
    }
//    positive = forward
    public void driveY(double position, double speed, Telemetry telemetry, double stopTime) {
        calibrateEncoders();
        double beginTime = System.currentTimeMillis();
        double TimeElapsed = System.currentTimeMillis() - beginTime;
        double turn;
        double heading = current_target_heading;
        double OdoY_Pos = FrontL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive() && TimeElapsed < stopTime) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getCurrentHeading() - heading);

            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("CurrentHeading", getCurrentHeading());
            telemetry.addData("TargetHeading", heading);
            telemetry.addData("time", runtime.milliseconds());
            telemetry.addData("FL power", FrontL.getPower());
            telemetry.addData("FR power", FrontR.getPower());
            telemetry.addData("BL power", BackL.getPower());
            telemetry.addData("BR power", BackR.getPower());
            telemetry.update();

            FrontL.setPower(speed + turn);
            BackL.setPower((speed + turn));
            BackR.setPower(speed - turn);
            FrontR.setPower(speed - turn);

            TimeElapsed = System.currentTimeMillis() - beginTime;

            OdoY_Pos = FrontL.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }
    public void driveX(double position){
        driveX(position,0.3, myOpMode.telemetry);
    }
    //positive = right
    public void driveX(double position, double speed, Telemetry telemetry) {
        speed = speed * -1;
        calibrateEncoders();
        double Kp = 0.03;
        double turn= 0;
        double heading = current_target_heading;
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoX_Pos;
        while (!((Math.abs(dPos) < threshold)) && myOpMode.opModeIsActive()) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getCurrentHeading() - heading);

            telemetry.addData("tick", tick);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("FL power", FrontL.getPower());
            telemetry.addData("FR power", FrontR.getPower());
            telemetry.addData("BL power", BackL.getPower());
            telemetry.addData("BR power", BackR.getPower());
            telemetry.update();

            FrontL.setPower(-speed + turn);
            FrontR.setPower(speed - turn);
            BackL.setPower(speed + turn);
            BackR.setPower(-speed - turn);

            OdoX_Pos = FrontR.getCurrentPosition();
            dPos = tick - OdoX_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void rotateToHeading(double target_heading){
        rotateToHeading(target_heading,0.4, myOpMode.telemetry);
    }
    //positive = clockwise
    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        target_heading *= -1;
        double current_heading = getCurrentHeading();
        double dHeading = target_heading - current_heading;
        double direction;
        double margin = 0.5;
        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < margin) && myOpMode.opModeIsActive()) {
            direction = checkDirection(dHeading);

            FrontL.setPower(-speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower((-speed * direction));
            BackR.setPower(speed * direction);

            current_heading = getCurrentHeading();
            dHeading = target_heading - current_heading;
            if(dHeading < 10 * margin) {
                speed = 0.2;
            }
            telemetry.addData("curHeading", current_heading);
            telemetry.addData("dHeading",dHeading);
            telemetry.update();
        }
        calibrateEncoders();
        Stop();
        current_target_heading = target_heading;
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void FieldCentric(double speed, HardwareMap map) {
        double theta = getCurrentHeading()*(Math.PI/180);
        double forward = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
        double strafe = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
        double rotate = myOpMode.gamepad1.right_stick_x;

        FrontL.setPower((-forward + strafe + rotate) * speed);
        FrontR.setPower((-forward - strafe - rotate) * speed);
        BackL.setPower((-forward - strafe + rotate) * speed);
        BackR.setPower((-forward + strafe - rotate) * speed);

        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            resetYaw();
        }
        myOpMode.telemetry.addData("wtf",-forward-strafe+rotate);
        myOpMode.telemetry.addData("Forward",forward);
        myOpMode.telemetry.addData("Strafe",strafe);
        myOpMode.telemetry.addData("Rotate",rotate);
        myOpMode.telemetry.addData("Currentheading",getCurrentHeading());
    }
    public void RobotCentric(double speed) {
        double FWD = myOpMode.gamepad1.left_stick_y;
        double STR = myOpMode.gamepad1.left_stick_x;
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;

        FrontL.setPower((FWD + STR + ROT) * (speed));
        FrontR.setPower((FWD - STR + ROT) * (speed));
        BackL.setPower((FWD - STR - ROT) * (speed));
        BackR.setPower((FWD + STR - ROT) * (speed));
    }

    public void Stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}