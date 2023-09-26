package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake extends RobotPart{

    Servo leftClaw;
    Servo leftRotate;
    DcMotor slide;
    double leftClawPos;
    double leftRotatePos;

    public void init(HardwareMap map) {
        leftClaw = map.get(Servo.class, "leftClaw");
        leftRotate = map.get(Servo.class, "leftRotate");
        slide  = map.dcMotor.get("slide");
    }

    /**
     *
     * @param mode 1 = intakePos, 2 = movePos, 3 = outtakePos
     */
    public void updateLeftRotate(int mode) {
        if (mode == 1) {
            leftClawPos = 1.0;
        } else if (mode == 2) {
            leftClawPos = 0.2; //0.5
        } else if (mode == 3) {
            leftClawPos = 0.5;
        }

        leftClaw.setPosition(leftClawPos);
    }

    /**
     *
     * @param mode 1 = release, 2 = grab
     */
    public void updateLeftClaw(int mode) {
        if (mode == 1) {
            leftRotatePos = 0.5;
        } else if (mode == 2) {
            leftRotatePos= 0.25;
        }
        leftRotate.setPosition(leftRotatePos);
    }

    /**
     * I mean this is pretty self-explanatory
     * @param Power between -1 and 1
     */
    public void slideMove(double Power){
        slide.setPower(Power);
    }
    /**
     * Hiervoor moet de slides eerst encoder cables hebben.
     * @param height encoder height
     */
    public void outtakeSequence(int height) {
        updateLeftClaw(1);
        updateLeftRotate(2);
//        slide.goToPosition(0);
        updateLeftRotate(1);
        updateLeftClaw(2);
//        slide.goToPosition(height);
        updateLeftRotate(3);
        updateLeftClaw(1);
        updateLeftRotate(2);
    }
}