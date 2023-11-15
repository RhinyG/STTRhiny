package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVRandomization;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;

@Autonomous(name = "AutonV1")
public class AutonV1 extends LinearOpMode {
    OpenCVRandomization camera = new OpenCVRandomization(this);
    newAutonMethods methods = new newAutonMethods(this);

    public void runOpMode() {
        methods.init(hardwareMap);
        methods.calibrateEncoders();

        methods.resetIMU(hardwareMap);
        camera.findScoringPosition();
        // rotateToHeading(); is in radialen, en kijk of je assen kloppen de XYZ van imu, anders doet ie niks
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("localPos", camera.pos);
            if (camera.pos == 0) {
                methods.driveY(-20, 0.3, telemetry);
                methods.driveX(50, 0.3, telemetry);
            } else if (camera.pos == 1) {
                methods.driveY(-50, 0.3, telemetry);
                methods.driveX(15, 0.3, telemetry);
                methods.driveY(-80, 0.3, telemetry);
                methods.driveY(-50, 0.3, telemetry);
                methods.driveX(0, 0.3, telemetry);
            } else if (camera.pos == 2){
                methods.driveX(20,0.3, telemetry);
                methods.driveY(-50, 0.3, telemetry);
                methods.rotateToHeading(0.5*Math.PI, 0.2, telemetry);
                methods.driveY(-80, 0.3, telemetry);
                methods.driveY(-50, 0.3, telemetry);
            }
            if (camera.pos == 1 || camera.pos == 2) {}
            //rest of shit
            sleep(30000);
        }
    }
}
