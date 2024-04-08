package org.firstinspires.ftc.teamcode.robotParts.Outdated;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

public class IntakeReza extends RobotPart {

    CRServo intake;

    public void init(HardwareMap map) {
        intake = map.get(CRServo.class, "intake");

        // reverse one motor
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // motors
        crServos.put("intake", intake);
    }

    public void update(boolean up, boolean down) {
        double power;
        if (up) {
            power = 1;
        } else if (down) {
            power = -1;
        } else {
            power = 0;
        }
        allCrPower(power);
    }

    public void runOpMode() throws InterruptedException {

    }
}