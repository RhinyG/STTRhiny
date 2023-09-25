package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake extends RobotPart{

    Servo leftClaw;
    Servo leftRotate;
    double leftClawPos;
    double leftRotatePos;

    public void init(HardwareMap map) {
        leftClaw = map.get(Servo.class, "leftClaw");
        leftRotate = map.get(Servo.class, "leftRotate");
    }

    public void updateLeftClaw(boolean up, boolean down) {
        if (up) {
            leftClawPos = 1;
        } else if (down) {
            leftClawPos = -1;
        }
        leftClaw.setPosition(leftClawPos);
    }
    public void updateLeftRotate(boolean up, boolean down) {
        if (up) {
            leftRotatePos = 1;
        } else if (down) {
            leftRotatePos= -1;
        }
        leftRotate.setPosition(leftRotatePos);
    }
}