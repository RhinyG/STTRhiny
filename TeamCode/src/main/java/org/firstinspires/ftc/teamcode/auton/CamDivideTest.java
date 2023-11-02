package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.autonParts.OpenCVRandomization;
@Autonomous(name = "CamDivideTest")
public class CamDivideTest extends OpMode {
    OpenCVRandomization random = new OpenCVRandomization(this);

    public void init() {
        random.findScoringPosition();
    }

    public void loop() {}
}
