package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVTeamPropDetection {
    public enum robotPositions {
        BlueBackstage,
        BlueWing,
        RedBackstage, //.1
        RedWing
    }
    LinearOpMode myOpMode;
    OpenCvWebcam webcam = null;

    public int pos = 1; //Left 0, Middle 1, Right 2
    double leftAvgFin;
    double rightAvgFin;
    public OpenCVTeamPropDetection(LinearOpMode opMode) {
        myOpMode = opMode;
        telemetry = opMode.telemetry;
        map = opMode.hardwareMap;
    }
    Telemetry telemetry;
    HardwareMap map;

    public void findScoringPosition(robotPositions robotPos, HardwareMap map) {
        WebcamName webcamName;
        if (robotPos == robotPositions.RedWing || robotPos == robotPositions.RedBackstage) {
             webcamName = map.get(WebcamName.class, "Webcam Right");
        } else {
            webcamName = map.get(WebcamName.class, "Webcam Left");
        }
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(new brightnessPipeline(robotPos));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void stopStreaming(){
        webcam.stopStreaming();
    }
    class brightnessPipeline extends OpenCvPipeline {
        robotPositions robotPos;
        brightnessPipeline(robotPositions robotPosition){robotPos = robotPosition;}
        Mat HSV = new Mat();
        Rect
            pos0LeftRect = new Rect(190,330, 200, 220),
            pos0RightRect = new Rect(690, 220, 180, 220),
            pos1LeftRect = new Rect(390,270, 190, 210),
            pos1RightRect = new Rect(960, 220, 200, 225),
            pos2LeftRect = new Rect(390,310, 180, 199),
            pos2RightRect = new Rect(820, 430, 220, 240),
            pos3LeftRect = new Rect(80,340, 200, 230),
            pos3RightRect = new Rect(660, 360, 190, 210);
        Mat outPut = new Mat();
        Scalar redColor = new Scalar(255.0, 0.0, 0.0);
        Scalar greenColor = new Scalar(0.0, 255.0, 0.0);
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            input.copyTo(outPut);
            Mat leftCrop,rightCrop;

            Rect leftRect = pos0LeftRect,rightRect = pos0RightRect;

            if (robotPos == robotPositions.RedWing) {
                leftRect = pos3LeftRect;
                rightRect = pos3RightRect;
            } else if (robotPos == robotPositions.RedBackstage) {
                leftRect = pos2LeftRect;
                rightRect = pos2RightRect;
            } else if (robotPos == robotPositions.BlueWing) {
                leftRect = pos1LeftRect;
                rightRect = pos1RightRect;
            }

            Imgproc.rectangle(outPut, leftRect, redColor, 2);
            Imgproc.rectangle(outPut, rightRect, redColor, 2);
            leftCrop = HSV.submat(leftRect);
            rightCrop = HSV.submat(rightRect);

            // For HSV: measures intensity so always use coi = 1. No clue what the other values do/measure/are.
            // For YCbCr: Blue = 1, Red = 2
            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar leftAvg = Core.mean(leftCrop),rightAvg = Core.mean(rightCrop);

            leftAvgFin = leftAvg.val[0];
            rightAvgFin = rightAvg.val[0];

            telemetry.addData("valueLeft", leftAvgFin);
            telemetry.addData("valueRight", rightAvgFin);

            if (robotPos == robotPositions.RedWing || robotPos == robotPositions.BlueBackstage) {
                if (Math.abs(leftAvgFin - rightAvgFin) < 20) {
                    pos = 2;
                } else if (rightAvgFin > leftAvgFin) {
                    Imgproc.rectangle(outPut, leftRect, greenColor, 2);
                    pos = 1;
                } else if (leftAvgFin > rightAvgFin) {
                    Imgproc.rectangle(outPut, rightRect, greenColor, 2);
                    pos = 0;
                }
            } else {
                if (Math.abs(leftAvgFin - rightAvgFin) < 20) {
                    pos = 0;
                } else if (rightAvgFin > leftAvgFin) {
                    Imgproc.rectangle(outPut, rightRect, greenColor, 2);
                    pos = 2;
                } else {
                    Imgproc.rectangle(outPut, leftRect, greenColor, 2);
                    pos = 1;
                }
            }

            if (pos == 0) {
                telemetry.addData("Conclusion", "left");
            } else if (pos == 1) {
                telemetry.addData("Conclusion", "mid");
            } else {
                telemetry.addData("Conclusion", "right");
            }
            telemetry.addData("pos", pos);
            telemetry.update();
            return (outPut);
        }
    }
}