package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotCentric extends RobotPart{
    private LinearOpMode myOpMode;

    AlexDistanceSensorUtil distanceSensor = new AlexDistanceSensorUtil();

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    double current_target_heading = 0;
    BNO055IMU imu;
    Orientation anglesHead;
    double WHEEL_RADIUS = 48;//mm
    double ODO_RADIUS = 17.5;//mm?
    double GEAR_RATIO = 1/13.7;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
    final double odoMultiplier = 5.0/3.0;
    final public int robotLength_cm = 38;

    public static double maxSpeed = 0.6;

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotor.class, "left_front");
        rightFront = map.get(DcMotor.class, "right_front");
        leftBack = map.get(DcMotor.class, "left_back");
        rightBack = map.get(DcMotor.class, "right_back");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        calibrateEncoders();

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);

        resetIMU(map);
    }

    public void drive(double forward, double right, double rotate, boolean slowMode) {
        double leftFrontPower = -forward - right + rotate;
        double rightFrontPower = -forward + right - rotate;
        double rightRearPower = -forward - right - rotate;
        double leftRearPower = -forward + right + rotate;
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));

        if(slowMode){
            leftFront.setPower(0.5 * maxSpeed * (leftFrontPower / maxPower));
            rightFront.setPower(0.5 * maxSpeed * (rightFrontPower / maxPower));
            rightBack.setPower(0.5 *maxSpeed * rightRearPower / maxPower);
            leftBack.setPower(0.5 * maxSpeed * (leftRearPower / maxPower));
        } else {
            leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
            rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
            rightBack.setPower(maxSpeed * rightRearPower / maxPower);
            leftBack.setPower(maxSpeed * (leftRearPower / maxPower));
        }
    }

    public void driveY(double position, double speed, Telemetry telemetry) {
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoY_Pos = -leftFront.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(heading-getCurrentHeading_DEGREES());

            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.update();

            leftFront.setPower(speed + turn);
            leftBack.setPower(speed + turn);

            rightBack.setPower(speed - turn);
            rightFront.setPower(speed - turn);

            OdoY_Pos = -leftFront.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void driveX(double position, double speed, Telemetry telemetry) {
        speed = speed * -1;
        //calibrateEncode();
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoX_Pos = -rightFront.getCurrentPosition();
        double tick = position * OURTICKS_PER_CM;
        double dPos = tick - OdoX_Pos;
        while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(heading-getCurrentHeading_DEGREES());

            telemetry.addData("tick", tick);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.update();
            leftFront.setPower(-speed + turn);
            rightFront.setPower(speed - turn);
            leftBack.setPower(speed + turn);
            rightBack.setPower(-speed - turn);
            OdoX_Pos = -rightFront.getCurrentPosition();
            dPos = tick - OdoX_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        double current_heading = -getCurrentHeading_DEGREES();
        double dHeading = target_heading - current_heading;
        double direction;
        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < 15) && myOpMode.opModeIsActive()) {
            direction = checkDirection(current_heading-target_heading);

            leftFront.setPower(-speed * direction);
            rightFront.setPower(speed * direction);
            leftBack.setPower(-speed * direction);
            rightBack.setPower(speed * direction);

            current_heading = getCurrentHeading_DEGREES();
            dHeading = target_heading - current_heading;
            telemetry.addData("curHeading", current_heading);
            telemetry.addData("dHeading",dHeading);
            telemetry.update();
        }
        calibrateEncoders();
        Stop();
        current_target_heading = target_heading;
    }

    public double getCurrentHeading_DEGREES() { //Threaded
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        return -1 * (anglesHead.firstAngle);
    }

    public void Stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void calibrateEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetIMU(HardwareMap map) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = map.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
