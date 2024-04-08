package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

//TODO: add camera

@Autonomous(group = "Tests")
public class FiftyFSM extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    double armTimer;
    int rotateGoal;
    double rotateSpeed = 1.0;
    int slideGoal;
    double slideSpeed = 0.7;

    @Override
    public void runOpMode() {
        drive.initRobot();
        arm.init();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Global Drive state",drive.state);
            telemetry.addData("driveState",drive.driveState);
            telemetry.addData("arm state",arm.state);
            telemetry.addData("slideGoal",slideGoal);
            telemetry.addData("slidePos",arm.armExtend1.getCurrentPosition());
            switch (drive.state) {
                case 0:
                    drive.driveState = 0;
                    drive.state++;
                    break;
                case 1:
                    drive.drive(87,-85,0.7,3000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.drive(-10,35,0.4,2500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 3:
                    if (arm.state > 5) {
                        drive.drive(-70,-30,0.7,4000);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                    }
            }
            switch (arm.state) {
                case 0:
                    rotateGoal = 2900;
                    arm.armRotate.setTargetPosition(rotateGoal);
                    arm.state++;
                    break;
                case 1:
                    if (Math.abs(arm.armRotate.getCurrentPosition()-rotateGoal) < 1000) {
                        slideGoal = 500;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 2:
                    if (drive.state >= 2) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.releaseLeft.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 3:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        rotateGoal = 0;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.armRotate.setTargetPosition(rotateGoal);
                        arm.elbow.setPosition(Crumblz.ElbowPositions.intakePos.getPosition());
                        arm.state++;
                    }
                    break;
                case 4:
                    if(Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 100) {
                        slideGoal = 850;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 5:
                    if(Math.abs(slideGoal - arm.armExtend1.getCurrentPosition()) < 10) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.releaseRight.getPosition());
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
            }
            arm.FSMArm(rotateGoal,rotateSpeed);
            arm.FSMSlide(slideGoal, slideSpeed);
            telemetry.update();
        }
    }
}