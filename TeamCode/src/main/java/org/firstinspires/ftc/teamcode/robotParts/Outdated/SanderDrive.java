package org.firstinspires.ftc.teamcode.robotParts.Outdated;
//for github
//TODO: import acmerobotics to access FTC dashboard
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

//@Config
public class SanderDrive {
    private LinearOpMode myOpMode;

    public DcMotorEx FrontL;
    public DcMotorEx FrontR;
    public DcMotorEx BackL;
    public DcMotorEx BackR;

    double cur_x = 0;
    double cur_y = 0;
    double current_target_heading = 0;
    BHI260IMU imu;
    double WHEEL_RADIUS = 48;
    double GEAR_RATIO = 1.0/13.8;
    double TICKS_PER_ROTATION = 8192;
    double odoMultiplier = (38.6/69.5);
    double CM_PER_TICK = (odoMultiplier * 2*Math.PI * GEAR_RATIO * WHEEL_RADIUS)/(TICKS_PER_ROTATION); //about 1/690
    double threshold = 250;
    double heading_error = 2;
    double min_speed = 0.15;
    double beginTime;
    double timeElapsedSECONDS;
    double standardStopTime = 6;
    double a = 0;

    int odoY_Pos;
    int odoX_Pos;
    double speed;
    double remweg;
    double Kp;
    double turn;
    double heading;
    double theta;
    double k;
    double cur_pos;
    double[] odoDistances;
    int target_x;
    int target_y;
    public int dPos_x;
    public int dPos_y;
    public int dPos;
    double f;
    double s;
    double FWD;
    double STR;
    double dHead;
    double odo_speed;
    double fl;
    double fr;
    double bl;
    double br;
    int X;
    int Y;

    public SanderDrive(LinearOpMode opmode) {myOpMode = opmode;}

    public void init(HardwareMap map) {
        imu = map.get(BHI260IMU.class, "imu");

        FrontL = map.get(DcMotorEx.class, "left_front");
        FrontR = map.get(DcMotorEx.class, "right_front");
        BackL = map.get(DcMotorEx.class, "left_back");
        BackR = map.get(DcMotorEx.class, "right_back");

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.FORWARD);
        resetEncoders();

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }

    public void drive(double x, double y, double max_speed, double stopTime, double b) {
        resetEncoders();
        beginTime = System.currentTimeMillis();
        remweg = max_speed * b;
        Kp = 0.1;
        heading = getTargetHeading(current_target_heading);
        theta = Math.toRadians(90);
        k = 0.1;
        odoDistances = calculateOdoDistance(x-cur_x,y-cur_y,k, Math.toRadians(-heading + 90));
        X = (int) odoDistances[1];
        Y = (int) odoDistances[0];
        target_x = (int) (X / CM_PER_TICK);
        target_y = (int) (Y / CM_PER_TICK);
        double[] arr;
        updateTargets();
        while ((Math.abs(dPos_x) > threshold || Math.abs(dPos_y) > threshold || Math.abs(dHead) > heading_error) &&  myOpMode.opModeIsActive() && timeElapsedSECONDS < stopTime) {
            STR = s;
            FWD = f;
            if ((dPos_x < 0 && STR > 0) || (dPos_x > 0 && STR < 0)) {
                STR = -STR;
            }
            if ((dPos_y < 0 && FWD > 0) || (dPos_y > 0 && FWD < 0)) {
                FWD = -FWD;
            }

            if (dPos > 0.5 * remweg) {
                a = 0.3;
            } else if (dPos < 2 * remweg) {
                a = dPos / (remweg);
            } else {
                a = 1;
            }
            speed = min_speed + a * (max_speed-min_speed);
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());
            fl = (FWD + STR); //FWD: was ++++
            fr = (FWD - STR); //STR: was +--+
            bl = (FWD - STR); //turn: was ++--
            br = (FWD + STR); //worked with 0

            myOpMode.telemetry.addData("undiv fl", fl);
            myOpMode.telemetry.addData("undiv fr", fr);
            myOpMode.telemetry.addData("undiv bl", bl);
            myOpMode.telemetry.addData("undiv br", br);

            arr = new double[]{Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br)};
            Arrays.sort(arr);
            if (Math.abs(arr[arr.length-1]) > speed) {
                double division = speed / Math.abs(arr[arr.length-1]);
                fl *= division;
                fr *= division;
                bl *= division;
                br *= division;
                myOpMode.telemetry.addData("div FWD", FWD * division);
                myOpMode.telemetry.addData("div STR", STR * division);
            }
            arr = null;
            fl += turn; //FWD: - + + -
            fr -= turn; //STR: + + - -
            bl += turn;
            br -= turn;

            telemetry();

            FrontL.setPower(fl);
            FrontR.setPower(fr);
            BackL.setPower(bl);
            BackR.setPower(br);

            updateTargets();
        }
        Stop();
        cur_x = x;
        cur_y = y;
        myOpMode.sleep(100);
    }

    public void updateTargets(){
        timeElapsedSECONDS = (System.currentTimeMillis() - beginTime) / 1000;
        odoY_Pos = FrontL.getCurrentPosition();
        odoX_Pos = FrontR.getCurrentPosition();
        cur_pos = Math.abs(odoX_Pos) + Math.abs(odoY_Pos);
        dPos_x = target_x - odoX_Pos;
        dPos_y = target_y - odoY_Pos;
        f = (dPos_x * Math.sin(theta) + dPos_y * Math.cos(theta));
        s = (dPos_x * Math.cos(theta) - dPos_y * Math.sin(theta));
        dHead = getCurrentHeading_DEGREES() - heading;
        dPos = Math.abs(dPos_x) + Math.abs(dPos_y);
    }

    public void telemetry() {
        myOpMode.telemetry.addData("fl", fl);
        myOpMode.telemetry.addData("fr", fr);
        myOpMode.telemetry.addData("bl", bl);
        myOpMode.telemetry.addData("br", br);
        myOpMode.telemetry.addData("target_x", target_x);
        myOpMode.telemetry.addData("target_y", target_y);
        myOpMode.telemetry.addData("dPos_x", dPos_x);
        myOpMode.telemetry.addData("dPos_y", dPos_y);
        myOpMode.telemetry.addData("FWD", FWD);
        myOpMode.telemetry.addData("STR", STR);
        myOpMode.telemetry.addData("turn", turn);
        myOpMode.telemetry.addData("X", odoDistances[1]);
        myOpMode.telemetry.addData("Y", odoDistances[0]);

        myOpMode.telemetry.addData("dHead", dHead);
        myOpMode.telemetry.addData("speed", speed);
        myOpMode.telemetry.addData("a", a);
        myOpMode.telemetry.addData("remweg", remweg);
        myOpMode.telemetry.addData("dpos", dPos);
        myOpMode.telemetry.addData("cur_pos", cur_pos);
        myOpMode.telemetry.addData("Odo_Speed", odo_speed);
        myOpMode.telemetry.addData("VelocityY", FrontR.getVelocity() * CM_PER_TICK);
        myOpMode.telemetry.addData("VelocityX", -FrontL.getVelocity()* CM_PER_TICK);
        myOpMode.telemetry.addData("odoY_Pos", odoY_Pos);
        myOpMode.telemetry.addData("odoX_Pos", odoX_Pos);
        myOpMode.telemetry.addData("currentHeading",getCurrentHeading_DEGREES());
        myOpMode.telemetry.addData("theta",theta);
        myOpMode.telemetry.addData("heading",heading);

        myOpMode.telemetry.update();
    }


//    public void atagDrive(double x, double y, double speed, double stopTime, double b) {
//        resetEncoders();
//        cur_y = 0;
//        cur_x = 0;
//        current_target_heading = 180;
//        drive(-x, -y, speed, stopTime, b);
//    }

    public void driveY(double position, double speed, double stopTime) {
        beginTime = System.currentTimeMillis();
        timeElapsedSECONDS = (System.currentTimeMillis() - beginTime) / 1000;
        //resetEncoders();
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoY_Pos = -FrontL.getCurrentPosition();
        int tick = (int) (position / CM_PER_TICK);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive() && timeElapsedSECONDS < stopTime) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());

            myOpMode.telemetry.addData("tick", tick);
            myOpMode.telemetry.addData("OdoY", OdoY_Pos);
            myOpMode.telemetry.addData("dPos", dPos);
            myOpMode.telemetry.addData("speed", speed);
            myOpMode.telemetry.update();

            FrontL.setPower(speed + turn);
            FrontR.setPower(-speed - turn);
            BackL.setPower(-speed + turn);
            BackR.setPower(speed - turn);
            timeElapsedSECONDS = (System.currentTimeMillis() - beginTime) / 1000;
            OdoY_Pos = -FrontL.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void driveX(double position, double speed, double stopTime) {
        speed = speed * -1;
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoX_Pos = -FrontR.getCurrentPosition();
        int tick = (int) (position / CM_PER_TICK);
        double dPos = tick - OdoX_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive() && timeElapsedSECONDS < stopTime) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());
            FrontL.setPower(-speed - turn);
            FrontR.setPower(-speed - turn);
            BackL.setPower(-speed + turn);
            BackR.setPower(-speed + turn);
            timeElapsedSECONDS = (System.currentTimeMillis() - beginTime) / 1000;
            OdoX_Pos = -FrontR.getCurrentPosition();
            dPos = tick - OdoX_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void driveY(double position, double speed) {
        driveY(position, speed, standardStopTime);
    }
    public void driveX(double position, double speed) {
        driveX(position, speed, standardStopTime);
    }
//    public void drive(double x, double y) {
//        drive(x, y, 0.7, 6, 20000);
//    }

    public void rotateToHeading(double target_heading, double speed) {
        double current_heading = getTargetHeading(getCurrentHeading_DEGREES());
        double dHeading = target_heading - current_heading;
        double direction;
        myOpMode.telemetry.addData("curHeading", current_heading);
        myOpMode.telemetry.addData("dHeading",dHeading);
        myOpMode.telemetry.update();
        while (!(dHeading> -.5 && dHeading < .5) && myOpMode.opModeIsActive()) {
            direction = -checkDirection(current_heading-target_heading);

            FrontL.setPower(speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower(speed * direction);
            BackR.setPower(speed * direction);

            current_heading = getTargetHeading(getCurrentHeading_DEGREES());
            dHeading = target_heading - current_heading;
            myOpMode.telemetry.addData("curHeading", current_heading);
            myOpMode.telemetry.addData("dHeading",dHeading);
            myOpMode.telemetry.addData("FL power", FrontL.getPower());
            myOpMode.telemetry.addData("FR power", FrontR.getPower());
            myOpMode.telemetry.addData("BL power", BackL.getPower());
            myOpMode.telemetry.addData("BR power", BackR.getPower());
            myOpMode.telemetry.update();
        }
        resetEncoders();
        Stop();
        current_target_heading = target_heading;
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void FieldCentric(double speed) {
        double theta = getCurrentHeading() + (Math.PI / 2);
        double FWD = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
        double STR = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;

        FrontL.setPower((FWD + STR + ROT) * (speed));
        FrontR.setPower((FWD - STR + ROT) * (speed));
        BackL.setPower((FWD - STR - ROT) * (speed));
        BackR.setPower((FWD + STR - ROT) * (speed));

        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            imu.resetYaw();
        }
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

    private double[] calculateOdoDistance(double x, double y, double k, double heading) {
        double[][] eMatrix = {{Math.cos(heading), -Math.sin(heading)}, {Math.sin(heading), Math.cos(heading)}};
        double phi = 0;
        double[] gradientPhi = {x, -y};
        double[] term1 = {
                eMatrix[0][0] * gradientPhi[0] + eMatrix[0][1] * gradientPhi[1],
                eMatrix[1][0] * gradientPhi[0] + eMatrix[1][1] * gradientPhi[1]
        };
        double term2 = k * phi;
        double[] term2Vector = {term2 * gradientPhi[0], term2 * gradientPhi[1]};
        double[] vectorField = {term1[0] - term2Vector[0], term1[1] - term2Vector[1]};
        return vectorField;
    }

    public double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (-orientation.getYaw(AngleUnit.RADIANS));
    }

    public double getCurrentHeading_DEGREES() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (-orientation.getYaw(AngleUnit.DEGREES));
    }

    public double getTargetHeading(double heading) {
        if (heading > 180) {
            heading = 360 - heading;
        }
        return heading;
    }

    public void resetEncoders() {
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