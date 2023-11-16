package org.firstinspires.ftc.teamcode.auton.autonParts;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class newAutonMethods {
    private LinearOpMode myOpMode;

    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;


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

    private DcMotor encoderX, encoderY;

    public newAutonMethods(LinearOpMode opmode) {myOpMode = opmode;}

    public void init(HardwareMap map) {
        FrontL = map.get(DcMotor.class, "left_front");
        FrontR = map.get(DcMotor.class, "right_front");
        BackL = map.get(DcMotor.class, "left_back");
        BackR = map.get(DcMotor.class, "right_back");
        FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        calibrateEncoders();

        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);
//        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontR.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        encoderX = map.dcMotor.get("leftFront");
//        encoderY = map.dcMotor.get("leftRear");
        resetIMU(map);
    }
//    positive = forward
    public void driveY(double position, double speed, Telemetry telemetry) {
        //calibrateEncode();
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoY_Pos = -FrontL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());

            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.update();

//            FrontL.setPower(speed + turn);
//            FrontR.setPower(-speed - turn);
//            BackL.setPower(-speed + turn);
//            BackR.setPower(speed - turn);

            FrontL.setPower(speed + turn);
            BackL.setPower(speed + turn);

            BackR.setPower(speed - turn);
            FrontR.setPower(speed - turn);

            OdoY_Pos = -FrontL.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }
    //positive = right
    public void driveX(double position, double speed, Telemetry telemetry) {
        speed = speed * -1;
        //calibrateEncode();
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoX_Pos = -FrontR.getCurrentPosition();
        double tick = position * OURTICKS_PER_CM;
        double dPos = tick - OdoX_Pos;
        while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());

            telemetry.addData("tick", tick);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.update();
            FrontL.setPower(-speed + turn);
            FrontR.setPower(speed - turn);
            BackL.setPower(speed + turn);
            BackR.setPower(-speed - turn);
            OdoX_Pos = -FrontR.getCurrentPosition();
            dPos = tick - OdoX_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

//    public void driveXY(double x, double y, double speed, Telemetry telemetry) {
//        double Kp = 0.03;
//        double turn;
//        double heading = current_target_heading;
//        double STR = speed;
//        double FWD = speed;
//        double Vx, Vy;
//        double OdoX_Pos = -FrontR.getCurrentPosition() * -1;
//        double OdoY_Pos = FrontL.getCurrentPosition() * -1;
//        int tickX = (int) (x * OURTICKS_PER_CM);
//        double dPosX = tickX - OdoX_Pos;
//        int tickY = (int) (y * OURTICKS_PER_CM);
//        double dPosY = tickY - OdoY_Pos;
//        while (((!(dPosX > -200 && dPosX < 200)) || !(dPosY > -200 && dPosY < 200)) && myOpMode.opModeIsActive()) {
//            Vy = speed * Math.cos(heading) - speed * Math.sin(heading);
//            Vx = speed * Math.sin(heading) + speed * Math.cos(heading);
//            if (!(dPosX > -200 && dPosX < 200)) {
//                STR = Vx;
//                if ((dPosX > 0 && STR > 0) || (dPosX < 0 && STR < 0)) {
//                    STR = -STR;
//                }
//            }
//            if (!(dPosY > -200 && dPosY < 200)) {
//
//                if ((dPosY > 0 && FWD > 0) || (dPosX < 0 && FWD < 0)) {
//                    FWD = -FWD;
//                }
//
//            }
//            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());
//            FrontL.setPower(FWD + STR - turn);
//            FrontR.setPower(FWD - STR + turn);
//            BackL.setPower(FWD - STR + turn);
//            BackR.setPower(FWD + STR - turn);
//
//            OdoX_Pos = -FrontR.getCurrentPosition() * -1;
//            telemetry.addData("OdoX", OdoX_Pos);
//            OdoY_Pos = FrontL.getCurrentPosition() * -1;
//            telemetry.addData("OdoY", OdoY_Pos);
//            dPosX = tickX - OdoX_Pos;
//            dPosY = tickY - OdoY_Pos;
//        }
//        Stop();
//    }

    //positive = clockwise
    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        double current_heading = -getCurrentHeading_DEGREES();
        double dHeading = target_heading - current_heading;
        double direction;
        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < 15) && myOpMode.opModeIsActive()) {
            direction = checkDirection(current_heading-target_heading);

            FrontL.setPower(-speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower(-speed * direction);
            BackR.setPower(speed * direction);

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

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    //Movements for aprilTags following

    public void atagForward(double speed) {
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());

        FrontL.setPower(-speed + turn);
        FrontR.setPower(speed - turn);
        BackL.setPower(speed + turn);
        BackR.setPower(-speed - turn);
    }
    public void atagBackward(double speed) {
        atagForward(-speed);
    }
    public void atagLeft(double speed) {
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading_DEGREES());

        FrontL.setPower(speed + turn);
        FrontR.setPower(speed - turn);
        BackL.setPower(speed + turn);
        BackR.setPower(speed - turn);
    }
    public void atagRight(double speed) {
        atagLeft(-speed);
    }
    public void atagX (double distance, double speed, Telemetry telemetry) {
        calibrateEncoders();
        driveX(distance, speed, telemetry);
    }
    public void atagY(double distance, double speed, Telemetry telemetry) {
        calibrateEncoders();
        driveY(distance, speed, telemetry);
    }


    public void FieldCentric(double speed, HardwareMap map) {
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
            resetIMU(map);
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
    //Autonomous drive in any direction
    public void Drive(double target_x_cm, double target_y_cm, double speed, Telemetry telemetry) {
        double Kp = 0.03;
        double turn;
        double heading = getCurrentHeading() + (Math.PI / 2);
        target_x_cm = target_x_cm * OURTICKS_PER_CM;
        target_y_cm = target_y_cm * OURTICKS_PER_CM;
        double cur_x = FrontR.getCurrentPosition();
        double cur_y = FrontL.getCurrentPosition() * -1;
        telemetry.addData("dX: ", target_x_cm - cur_x);
        telemetry.addData("dY: ", target_y_cm - cur_y);
        telemetry.update();
        double Vy, Vx, FWD, STR;
        while (myOpMode.opModeIsActive() && (!(target_x_cm - cur_x > -200 && target_x_cm - cur_x < 200) || !(target_y_cm - cur_y > -200 && target_y_cm - cur_y < 200))) {
            Vy = ((target_x_cm-cur_x) * Math.sin(heading) + (target_y_cm-cur_y) * Math.cos(heading));
            Vx = ((target_x_cm-cur_x) * Math.cos(heading) - (target_y_cm-cur_y) * Math.sin(heading));
            if (!(target_y_cm - cur_y > -0.05 && target_y_cm - cur_y < 0.05))
                FWD = speed / (Vy + Vx) * Vy * checkDirection(target_y_cm - cur_y);
            else FWD = 0;
            if (!(target_x_cm - cur_x > -0.05 && target_x_cm - cur_x < 0.05))
                STR = speed / (Vy + Vx) * Vx * checkDirection(target_x_cm - cur_x);
            else STR = 0;

            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading());

            FrontL.setPower(FWD + STR + turn);
            FrontR.setPower(FWD - STR + turn);
            BackL.setPower(FWD - STR - turn);
            BackR.setPower(FWD + STR - turn);

            cur_x = FrontR.getCurrentPosition();
            cur_y = FrontL.getCurrentPosition() * -1;
            telemetry.addData("dY: ", target_y_cm - cur_y);
            telemetry.addData("dX: ", target_x_cm - cur_x);
            telemetry.addData("FWD: ", FWD);
            telemetry.addData("STR: ", STR);
            telemetry.addData("turn: ", turn);
            telemetry.update();
        }
        Stop();
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

    public double getCurrentHeading() { //Threaded
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.RADIANS);
        return (anglesHead.firstAngle);
        //currentHeading = getTargetHeading((int)(-1*anglesHead.firstAngle));
    }

    //XZY
    public double getCurrentHeading_DEGREES() { //Threaded
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES);
        return -1 * (anglesHead.firstAngle);
        //currentHeading = getTargetHeading((int)(-1*anglesHead.firstAngle));
    }

    public double getTargetHeading(double heading) {
        if(heading> 0 && heading < -180){
            heading =- 360;
        }
//        if (heading > 180 || head) {
//            heading = 360 - heading;
//        }
        return heading;
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