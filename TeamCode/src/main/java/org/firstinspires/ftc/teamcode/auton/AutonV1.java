package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVRandomization;
import org.firstinspires.ftc.teamcode.auton.autonParts.newAutonMethods;

@Autonomous(name = "AutonV1")
public class AutonV1 extends LinearOpMode {
    OpenCVRandomization random = new OpenCVRandomization(this);
    newAutonMethods methods = new newAutonMethods(this);

    public void initAuton() {
        random.findScoringPosition();
    }



    public void runOpMode() {
        initAuton();
        double localPos = random.pos;

        waitForStart();

        if (localPos == 0) {
            //place pixel and go back
            methods.driveX();
        } else if ( localPos == 1) {
            //place pixel and go back
            methods.driveY(80/0.75,0.2,telemetry);
        } else {
            //place pixel and go back
        }
        //rest of shit
        telemetry.update();
    }
}
