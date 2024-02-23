package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

//TODO: add camera
@Disabled
@Autonomous(group = "Test")
public class DemoAuton extends LinearOpMode {
    //TODO: explain this
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    //TODO: explain variables
    double armTimer;
    int rotateGoal;
    double rotateSpeed = 1.0;
    int slideGoal;
    double slideSpeed = 0.7;

    @Override
    public void runOpMode() {
        //TODO: explain init
        drive.init(hardwareMap);
        arm.init(hardwareMap);

        //TODO: explain
        waitForStart();

        //TODO: explain
        while (opModeIsActive()){

            //TODO: explain
            telemetry.addData("Global Drive state",drive.state);
            telemetry.addData("driveState",drive.driveState);
            telemetry.addData("arm state",arm.state);
            telemetry.addData("slideGoal",slideGoal);
            telemetry.addData("slidePos",arm.armExtend.getCurrentPosition());

            //TODO: explain
            switch (drive.state) {
                case 0:
                    //TODO: explain
                    drive.driveState = 0;
                    drive.state++;
                    break;
                case 1:
                    //TODO: explain
                    drive.drive(87,-85,0.7,3000);
                    //TODO: explain reset for next case
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    //TODO: explain timer
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.drive(-10,35,0.4,2500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 3:
                    //TODO: explain if from other switch
                    if (arm.state > 5) {
                        drive.drive(-70,-30,0.7,4000);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                    }
            }

            //TODO: explain
            switch (arm.state) {
                case 0:
                    //TODO: explain
                    rotateGoal = 2900;
                    arm.armRotate.setTargetPosition(rotateGoal);
                    arm.state++;
                    break;
                case 1:
                    //TODO: explain
                    if (Math.abs(arm.armRotate.getCurrentPosition()-rotateGoal) < 1000) {
                        slideGoal = 500;
                        arm.armExtend.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 2:
                    //TODO: explain
                    if (drive.state >= 2) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.releaseLeft.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
            }

            //TODO: explain
            arm.FSMArm(rotateGoal,rotateSpeed);
            arm.FSMSlide(slideGoal, slideSpeed);

            //TODO: explain
            telemetry.update();
        }
    }
}