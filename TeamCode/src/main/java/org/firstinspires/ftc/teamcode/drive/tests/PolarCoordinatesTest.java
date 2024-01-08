package org.firstinspires.ftc.teamcode.drive.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Polar Coordinates", group = "Tests")
public class PolarCoordinatesTest extends LinearOpMode {
    public Servo wrist;
    @Override
    public void runOpMode() throws InterruptedException {

        int outtakePos;
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = Math.sqrt(x * x + y * y);
        double theta;

        wrist = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (x >= 0 && y >= 0) {
                theta = Math.atan(y / x);
            } else if (x<0) {
                theta = Math.atan(y / x) + Math.PI;
            } else {
                theta = Math.atan(y / x) + 2 * Math.PI;
            }

            if(r > 0.5){
                if (theta < 2.0/6.0*Math.PI) {
                    outtakePos = 1;
                } else if (theta < 4.0/6.0*Math.PI) {
                    outtakePos = 2;
                } else if (theta < 6.0/6.0*Math.PI) {
                    outtakePos = 3;
                } else if (theta < 8.0/6.0*Math.PI) {
                    outtakePos = 4;
                } else if (theta < 10.0/6.0*Math.PI) {
                    outtakePos = 5;
                } else {
                    outtakePos = 6;
                }

                switch (outtakePos) {
                    case 1:
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                    case 2:
                        wrist.setPosition(1);
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                    case 3:
                        wrist.setPosition(0.08);
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                    case 4:
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                    case 5:
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                    case 6:
                        telemetry.addData("OuttakePos", outtakePos);
                        break;
                }
            } else if (gamepad1.left_stick_button) {
                outtakePos = 0;
                wrist.setPosition(0.5);
                telemetry.addData("Going for neutral",outtakePos);
            }

            wrist.setPosition(y);

            telemetry.addData("x",x);
            telemetry.addData("y",y);
            telemetry.addData("r",r);
            telemetry.addData("theta radians",theta);
            telemetry.addData("theta degrees", theta*(180/Math.PI));
            telemetry.update();
        }
    }
}