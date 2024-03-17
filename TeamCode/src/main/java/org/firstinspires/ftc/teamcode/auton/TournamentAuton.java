package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(group = "A")
public class TournamentAuton extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);
    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);

    double
            armTimer,
            slideSpeed = 0.7;
    int rotateGoal, slideGoal;
    @Override
    public void runOpMode() {
        drive.initRobot();
        arm.init();
        camera.findScoringPosition(OpenCVTeamPropDetection.robotPositions.RedBackstage,hardwareMap);

        waitForStart();

        camera.stopStreaming();

        int finalPos = camera.pos;

        while (opModeIsActive()){
            telemetry.addData("Global Drive state",drive.state);
            telemetry.addData("driveState",drive.driveState);
            telemetry.addData("arm state",arm.state);
            switch (drive.state) {
                case 0:
                    drive.driveState = 0;
                    drive.state++;
                    break;
                case 1:
                    if (finalPos == 0) {
                        drive.drive(93, -85, 0.7, 3000);
                    } else if (finalPos == 1) {
                        drive.drive(79,-85,0.7,3000);
                    } else if (finalPos == 2) {
                        drive.drive(55,-85,0.7,3000);
                    }
                    //TODO: make this shit a method or part of drive.drive()
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        if (finalPos == 0) {
                            drive.drive(-10,40,0.4,2500);
                        } else if (finalPos == 1) {
                            drive.drive(21,13,0.4,2500);
                        } else if (finalPos == 2) {
                            drive.drive(26,35,0.4,2500);
                        }
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 3:
                    if (arm.state > 5) {
                        if (finalPos == 0) {
                            drive.drive(-75,-36,0.8,3800);
                        } else if (finalPos == 1) {
                            drive.drive(-87,-15,0.8,3800);
                        } else if (finalPos == 2) {
                            drive.drive(-75,-36,0.8,3800);
                        }
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
            }

            switch (arm.state) {
                case 0:
                    rotateGoal = 2800;
                    arm.state++;
                    break;
                case 1:
                    if (Math.abs(arm.armRotate.getCurrentPosition()-rotateGoal) < 1000) {
                        slideGoal = 360;
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
                        rotateGoal = 320;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.elbow.setPosition(Crumblz.ElbowPositions.intakePos.getPosition());
                        arm.state++;
                    }
                    break;
                case 4:
                    if(Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 100) {
                        if (finalPos != 2) {
                            slideGoal = 850;
                            arm.armExtend1.setTargetPosition(slideGoal);
                        }
                        arm.state++;
                    }
                    break;
                case 5:
                    if(Math.abs(slideGoal - arm.armExtend1.getCurrentPosition()) < 10) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.releaseRight.getPosition());
                        rotateGoal = 0;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
            }

            //TODO: make this a method
            arm.rotateArm();

            telemetry.addData("target",rotateGoal);
            arm.FSMSlide(slideGoal,slideSpeed);
            telemetry.update();
        }
    }
}