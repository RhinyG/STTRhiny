package org.firstinspires.ftc.teamcode.auton.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "RB 2+2", group = "B")
public class SixtyRed extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);
//    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);

    double
            p = 0.008,
            i = 0,
            d = 0,
            f = 0.08,
            armTimer,
            slideSpeed = 0.7;
    int
            rotateGoal,
            slideGoal;
    @Override
    public void runOpMode() {
        drive.init();
        arm.init();
//        camera.findScoringPosition(OpenCVTeamPropDetection.robotPositions.RedBackstage,hardwareMap);
        PIDController controller = new PIDController(p, i, d);

        waitForStart();

//        camera.stopStreaming();

        int finalPos = 0;

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
                        drive.drive(96, -85, 0.7, 3000);
                    } else if (finalPos == 1) {
                        drive.drive(79,-85,0.7,3000);
                    } else if (finalPos == 2) {
                        drive.drive(55,-85,0.7,3000);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        if (finalPos == 0) {
                            drive.drive(-13,40,0.4,2500);
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
                        drive.drive(54,-10,0.7,2500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 4:
                    drive.drive(18,196,0.7,5000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 5:
                    drive.drive(0,15,0.3,5000);
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
                        rotateGoal = 600;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 6:
                    if (drive.state > 3) {
                        slideGoal = 500;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        rotateGoal = 300;
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.openLeft.getPosition());
                        arm.elbow.setPosition(Crumblz.ElbowPositions.stackFivePos.getPosition());
                    }
                case 7:
                    if (drive.state > 5) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.grabLeft.getPosition());
                        arm.state++;
                        armTimer = System.currentTimeMillis();
                    }
                    break;
            }

            controller.setPID(p,i,d);
            int armPos = arm.armRotate.getCurrentPosition();
            double pid = controller.calculate(armPos,rotateGoal);
            if (armPos > 1500) {
                f = 0;
            } else {
                f = 0.08;
            }
            double ticks_in_degree = 5281.1 / 180.0;
            double ff = Math.cos(Math.toRadians(rotateGoal / ticks_in_degree)) * f;

            double power = pid + ff;
            arm.armRotate.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target",rotateGoal);
            arm.FSMSlide(slideGoal,slideSpeed);
            telemetry.update();
        }
    }
}