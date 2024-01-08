package org.firstinspires.ftc.teamcode.robotParts.Outdated;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class autonMethods {
    private LinearOpMode myOpMode;

    public ElapsedTime runtime = new ElapsedTime();
    //private PsrAutoUtil robot = new PsrAutoUtil(this);
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    BNO055IMU imu;
    Orientation anglesHead;
    DigitalChannel digitalTouch;


    double currentArm;
    double startArm;
    int prev_odo_leftX;
    int prev_odo_rightX;
    int prev_odo_centerY;
    int rotations = 0;

    public static double cmPerTickY = 0.0028;
    public static double cmPerTickX = 0.0018;

    public static int SleepAfterCmd = 125;//was 300

    public void initMotors(HardwareMap map) {
        leftFront = map.get(DcMotor.class, "left_front");
        rightFront = map.get(DcMotor.class, "right_front");
        leftBack = map.get(DcMotor.class, "left_back");
        rightBack = map.get(DcMotor.class, "right_back");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void calibrateEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double startArm = arm.startArm;
    }

    public double getCurrentHeading() {
        //Onze order voor de axes is ZYX. Kan zijn dat dit voor jullie verschilt.
        anglesHead = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return getTargetHeading((int) (-1 * anglesHead.firstAngle));
    }

    public int getTargetHeading(int heading) {
        if (heading > 181 && heading < 360) {
            return heading - 360;
        } else return heading;
    }

    public void calibrateIMU(HardwareMap map) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imuHelper.initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //imuHelper.storeInitialOrientation();
    }

    public void Forward(double distance_cm, int heading, double speed, double maxDriveTime, Telemetry telemetry) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double x = get_odo_x_cm(telemetry);
        double startDriveTime = System.currentTimeMillis();
        double relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        while (x < distance_cm && relativeDriveTime < maxDriveTime) {
            if ((distance_cm - x) < 30) {
                if (speed > 0) {
                    speed = 0.2;
                } else {
                    speed = -0.2;
                }
            }

            turn = Kp * Math.abs(speed) * (getTargetHeading(heading) - getCurrentHeading());

            leftFront.setPower(speed + turn);
            leftBack.setPower(speed + turn);

            rightBack.setPower(speed - turn);
            rightFront.setPower(speed - turn);

            telemetry.addData("currentheading", getCurrentHeading());
            x = get_odo_x_cm(telemetry);
            relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        }
        Stop();
    }

    public void Backward(double distance_cm, int heading, double speed, double maxDriveTime, Telemetry telemetry) {
        Forward(distance_cm, heading, -1 * speed, maxDriveTime, telemetry);
    }

    public void Left(int distance_cm, int heading, double speed, Telemetry telemetry) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double y = get_odo_y_cm(telemetry);

        while (y < distance_cm) {
            if ((distance_cm - y) < 15) //was 30
            {
                if (speed > 0) {
                    speed = 0.2;
                } else {
                    speed = -0.2;
                }
            }

            turn = Kp * Math.abs(speed) * (getTargetHeading(heading) - getCurrentHeading());

            leftFront.setPower(-1 * speed + turn);
            leftBack.setPower(speed + turn);
            rightBack.setPower(-1 * speed - turn);
            rightFront.setPower(speed - turn);

            telemetry.addData("currentheading", getCurrentHeading());
            y = get_odo_y_cm(telemetry);
        }
        Stop();
    }

    public void Right(int distance_cm, int heading, double speed, Telemetry telemetry) {
        Left(distance_cm, heading, speed * -1, telemetry);
    }

    public void RotateLeft(int heading, double speed) {
        double error = 20 * speed;
        while (getCurrentHeading() > (getTargetHeading(heading) + error)) {
            leftFront.setPower(speed * -1);
            rightFront.setPower(speed);
            leftBack.setPower(speed * -1);
            rightBack.setPower(speed);
        }
        Stop();
    }

    public void RotateRight(int heading, double speed) {
        double error = 20 * speed;

        while (getCurrentHeading() < (getTargetHeading(heading) - error)) {
            rightFront.setPower(speed * -1);
            leftFront.setPower(speed);
            rightBack.setPower(speed * -1);
            leftBack.setPower(speed);
        }
        Stop();

    }

    public void Stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public double get_odo_y_cm(Telemetry telemetry) {
        double current_odo_y = rightFront.getCurrentPosition() - prev_odo_centerY;
        double distance = current_odo_y * cmPerTickY;
        telemetry.addData("Distance", distance);
        telemetry.update();
        return Math.abs(distance);
    }

    public double get_odo_x_cm(Telemetry telemetry) {
        double current_odo_leftX = leftBack.getCurrentPosition() - prev_odo_leftX;
        double current_odo_rightX = rightBack.getCurrentPosition() - prev_odo_rightX;
        double distance = Math.abs((current_odo_leftX + current_odo_rightX) / (2)) * cmPerTickX;
        telemetry.addData("Distance", distance);
        telemetry.update();
        return distance;
    }

    public void reset_odometry() {
        prev_odo_leftX = leftBack.getCurrentPosition();
        prev_odo_rightX = rightBack.getCurrentPosition();
        prev_odo_centerY = rightFront.getCurrentPosition();
    }
}