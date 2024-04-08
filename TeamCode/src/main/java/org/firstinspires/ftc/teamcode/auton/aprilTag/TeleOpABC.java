package org.firstinspires.ftc.teamcode.auton.aprilTag;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;
@Disabled
@TeleOp(group = "Tests")
public class TeleOpABC extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.0508;//meter

    double AvgYawToBackBoard;
    double AvgPoseXToBackBoard;
    double AvgPoseZToBackBoard;
    int yawsReceived;

    @Override
    public void runOpMode() {
        drive.initRobot();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                AvgYawToBackBoard = 0;
                yawsReceived = 0;
                AvgPoseZToBackBoard = 0;
                AvgPoseXToBackBoard = 0;

                for(AprilTagDetection tag : currentDetections) {
                    Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                    if(rot.secondAngle < 0){
                        AvgYawToBackBoard += rot.firstAngle;
                        yawsReceived++;
                    }
                    AvgPoseZToBackBoard += tag.pose.z;
                    AvgPoseXToBackBoard += tag.pose.x;
                }
                AvgYawToBackBoard /= yawsReceived;
                AvgPoseZToBackBoard /= currentDetections.size();
                AvgPoseXToBackBoard /= currentDetections.size();

                telemetry.addData("AvgYawToBackBoard",AvgYawToBackBoard);
                telemetry.addData("AvgPoseXToBackBoard",AvgPoseXToBackBoard);
                telemetry.addData("AvgPoseZToBackBoard",AvgPoseZToBackBoard);
                telemetry.addData("CorrectedZ",127*AvgPoseZToBackBoard+4.71);
                telemetry.addData("CorrectedX",Math.atan(Math.toRadians(AvgYawToBackBoard))*(127*AvgPoseZToBackBoard+4.71));
                telemetry.update();
            }

            //TODO: Test this shit
            if (gamepad1.right_stick_button) {
                drive.AprilTagBackboardCorrection(AvgYawToBackBoard, AvgPoseZToBackBoard);
            }
        }
    }
}