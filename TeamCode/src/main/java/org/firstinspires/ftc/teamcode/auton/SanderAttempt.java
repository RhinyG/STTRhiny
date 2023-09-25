package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class SanderAttempt extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //private PsrAutoUtil robot = new PsrAutoUtil(this);
    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    BNO055IMU imu;
    Orientation anglesHead;
    DigitalChannel digitalTouch;



    double currentArm;
    double startArm;
    int prev_odo_leftY;
    int prev_odo_rightY;
    int prev_odo_centerX;
    int rotations = 0;

    public static double cmPerTick = 0.0028;
    public static double cmPerTickX = 0.0018;

    public static int SleepAfterCmd = 125;//was 300

    public static final double HighJunction = 2020;
    public static final double MidJunction = 1600;
    public static final double LowJunction = 1000;
    public static final double GroundJunction = 100;


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Left = 9;
    int Middle = 10;
    int Right = 11;

    int TargetZone;
    public static int LeftZone = 52;
    public static int MidZone = 10;
    public static int RightZone = 55;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {


        initMotors();
        calibrateEncoders();

        calibrateIMU();

        initCamera();
        runtime.reset();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            findParkingPosition();
        }
        Forward(10, 0, 0.3, 10);
        Forward(113, 0, 0.5, 10);
        RotateRight(40, 0.3);
        Forward(23,37,0.3, 10);
        Backward(15,35,0.3, 10);
        RotateLeft(0,0.4);

        sleep(30000);

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    public void initMotors() {
        FrontL = hardwareMap.get(DcMotor.class, "left_front");
        FrontR = hardwareMap.get(DcMotor.class, "right_front");
        BackL = hardwareMap.get(DcMotor.class, "left_back");
        BackR = hardwareMap.get(DcMotor.class, "left_back");

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontL.setDirection(DcMotor.Direction.REVERSE);
    }
    public void calibrateEncoders() {
        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double startArm = arm.startArm;
    }
    public double getCurrentHeading() {
        //Onze order voor de axes is ZYX. Kan zijn dat dit voor jullie verschilt.
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return getTargetHeading((int)(-1*anglesHead.firstAngle));
    }

    public int getTargetHeading(int heading) {
        if(heading>181&&heading<360){
            return heading-360;
        }
        else return heading;
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
    public void Forward(double distance_cm,int heading, double speed, double maxDriveTime) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double y = get_odometry_y_cm();
        double startDriveTime = System.currentTimeMillis();
        double relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        while (y < distance_cm && opModeIsActive() && relativeDriveTime < maxDriveTime) {
            if ((distance_cm-y)<30)
            {
                if (speed>0)
                {speed=0.2;}
                else
                {speed=-0.2;}
            }

            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading());

            FrontL.setPower(speed+turn);
            BackL.setPower(speed+turn);

            BackR.setPower(speed-turn);
            FrontR.setPower(speed-turn);

            telemetry.addData("currentheading",getCurrentHeading());
            y = get_odometry_y_cm();
            relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        }
        Stop();
    }

    public void Backward(double distance_cm, int heading, double speed, double maxDriveTime) {
        Forward(distance_cm, heading, -1*speed, maxDriveTime);
    }

    public void Left(int distance_cm, int heading, double speed) {
        double Kp = 0.03;
        double turn;

        reset_odometry();
        double x = get_odometry_x_cm();

        while (x < distance_cm && opModeIsActive()) {
            if ((distance_cm-x)<15) //was 30
            {
                if (speed>0)
                {speed=0.2;}
                else
                {speed=-0.2;}
            }

            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading());

            FrontL.setPower(-1*speed+turn);
            BackL.setPower(speed+turn);
            BackR.setPower(-1*speed-turn);
            FrontR.setPower(speed-turn);

            telemetry.addData("currentheading",getCurrentHeading());
            x = get_odometry_x_cm();
        }
        Stop();
    }

    public void Right(int distance_cm, int heading, double speed) {
        Left(distance_cm, heading, speed * -1);
    }

    public void RotateLeft(int heading, double speed) {
        double error = 20*speed;
        while (getCurrentHeading() > (getTargetHeading(heading)+error) && opModeIsActive()) {
            FrontL.setPower(speed * -1);
            FrontR.setPower(speed);
            BackL.setPower(speed * -1);
            BackR.setPower(speed);
        }
        Stop();
    }

    public void RotateRight(int heading, double speed) {
        double error = 20*speed;

        while (getCurrentHeading() < (getTargetHeading(heading)-error) && opModeIsActive()) {
            FrontR.setPower(speed * -1);
            FrontL.setPower(speed);
            BackR.setPower(speed * -1);
            BackL.setPower(speed);
        }
        Stop();

    }

    public void Stop() {
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }

    public double get_odometry_x_cm() {
        double current_odo_x = BackL.getCurrentPosition() - prev_odo_centerX;
        double distance = current_odo_x*cmPerTickX;
        telemetry.addData("Distance",distance);
        telemetry.update();
        return Math.abs(distance);
    }

    public double get_odometry_y_cm() {
        double current_odo_y1 = FrontR.getCurrentPosition() - prev_odo_leftY;
        double current_odo_y2 = FrontL.getCurrentPosition() - prev_odo_rightY;
        double distance = Math.abs((current_odo_y1 + current_odo_y2) / (2))*cmPerTick;
        telemetry.addData("Distance",distance);
        telemetry.update();
        return distance;
    }

    public void reset_odometry() {
        prev_odo_leftY = FrontR.getCurrentPosition();
        prev_odo_rightY = FrontL.getCurrentPosition();
        prev_odo_centerX = BackL.getCurrentPosition();

    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    void findParkingPosition() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == Left || tag.id == Middle || tag.id == Right)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);
    }
}
