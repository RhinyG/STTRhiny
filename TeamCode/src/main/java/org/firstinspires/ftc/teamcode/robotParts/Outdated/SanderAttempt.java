package org.firstinspires.ftc.teamcode.robotParts.Outdated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;


public class SanderAttempt extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
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

    public static double cmPerTick = 0.0028;
    public static double cmPerTickX = 0.0018;

    public static int SleepAfterCmd = 125;//was 300

//
//
//
//    //OpenCvCamera camera;
//    //AprilTagDetectionPipeline aprilTagDetectionPipeline;
//
//    //public static double FEET_PER_METER = 3.28084;
//
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//
//    int Left = 9;
//    int Middle = 10;
//    int Right = 11;
//
//    int TargetZone;
//    public static int LeftZone = 52;
//    public static int MidZone = 10;
//    public static int RightZone = 55;
//
//    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        initMotors();
        calibrateEncoders();

        calibrateIMU();

        //initCamera();
        runtime.reset();
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
        Forward(10, 0, 0.3, 10);
        Forward(113, 0, 0.5, 10);
        RotateRight(40, 0.3);
        Forward(23, 37, 0.3, 10);
        Backward(15, 35, 0.3, 10);
        RotateLeft(0, 0.4);

        sleep(30000);

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    public void initMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "left_back");

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

    public void calibrateIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imuHelper.initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //imuHelper.storeInitialOrientation();
    }

    public void Forward(double distance_cm, int heading, double speed, double maxDriveTime) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double y = get_odo_x_cm();
        double startDriveTime = System.currentTimeMillis();
        double relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        while (y < distance_cm && opModeIsActive() && relativeDriveTime < maxDriveTime) {
            if ((distance_cm - y) < 30) {
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
            y = get_odo_x_cm();
            relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        }
        Stop();
    }

    public void Backward(double distance_cm, int heading, double speed, double maxDriveTime) {
        Forward(distance_cm, heading, -1 * speed, maxDriveTime);
    }

    public void Left(int distance_cm, int heading, double speed) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double x = get_odo_y_cm();

        while (x < distance_cm && opModeIsActive()) {
            if ((distance_cm - x) < 15) //was 30
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
            x = get_odo_y_cm();
        }
        Stop();
    }

    public void Right(int distance_cm, int heading, double speed) {
        Left(distance_cm, heading, speed * -1);
    }

    public void RotateLeft(int heading, double speed) {
        double error = 20 * speed;
        while (getCurrentHeading() > (getTargetHeading(heading) + error) && opModeIsActive()) {
            leftFront.setPower(speed * -1);
            rightFront.setPower(speed);
            leftBack.setPower(speed * -1);
            rightBack.setPower(speed);
        }
        Stop();
    }

    public void RotateRight(int heading, double speed) {
        double error = 20 * speed;

        while (getCurrentHeading() < (getTargetHeading(heading) - error) && opModeIsActive()) {
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

    public double get_odo_y_cm() {
        double current_odo_y = leftFront.getCurrentPosition() - prev_odo_centerY;
        double distance = current_odo_y * cmPerTickX;
        telemetry.addData("Distance", distance);
        telemetry.update();
        return Math.abs(distance);
    }

    public double get_odo_x_cm() {
        double current_odo_leftY = rightFront.getCurrentPosition() - prev_odo_leftX;
        double current_odo_rightY = leftBack.getCurrentPosition() - prev_odo_rightX;
        double distance = Math.abs((current_odo_leftY + current_odo_rightY) / (2)) * cmPerTick;
        telemetry.addData("Distance", distance);
        telemetry.update();
        return distance;
    }

    public void reset_odometry() {
        prev_odo_leftX = rightFront.getCurrentPosition();
        prev_odo_rightX = leftBack.getCurrentPosition();
        prev_odo_centerY = leftFront.getCurrentPosition();

    }
}
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//    void initCamera() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//    }

    /**
     * Dit is voor als wij niet distance sensors gaan gebruiken voor het kijken waar de teamprop staat.
     * Dit heeft anders geen nut om voor de start te gaan kijken...
     */
//    void findTeamProp() {
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//        if(currentDetections.size() != 0)
//        {
//            boolean tagFound = false;
//
//            for(AprilTagDetection tag : currentDetections)
//            {
//                if(tag.id == Left || tag.id == Middle || tag.id == Right)
//                {
//                    tagOfInterest = tag;
//                    tagFound = true;
//                    break;
//                }
//            }
//
//            if(tagFound)
//            {
//                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                tagToTelemetry(tagOfInterest);
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//
//        }
//        else
//        {
//            telemetry.addLine("Don't see tag of interest :(");
//
//            if(tagOfInterest == null)
//            {
//                telemetry.addLine("(The tag has never been seen)");
//            }
//            else
//            {
//                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                tagToTelemetry(tagOfInterest);
//            }
//
//        }
//
//        telemetry.update();
//        sleep(20);
//    }
//}
