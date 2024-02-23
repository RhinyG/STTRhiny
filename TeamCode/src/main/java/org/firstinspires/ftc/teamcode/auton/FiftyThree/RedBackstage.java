package org.firstinspires.ftc.teamcode.auton.FiftyThree;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "RB 2+0", group = "A")
public class RedBackstage extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);
    OpenCVTeamPropDetection camera = new OpenCVTeamPropDetection(this);

    double
            p = 0.008,
            i = 0,
            d = 0,
            f = 0.08;
    int
            rotateGoal,
            slideGoal;
    double
            armTimer,
            slideSpeed = 0.7;
    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        arm.init(hardwareMap);
        camera.findScoringPosition(OpenCVTeamPropDetection.robotPositions.RedBackstage,hardwareMap);
        PIDController controller = new PIDController(p, i, d);

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
                        arm.armExtend.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 2:
                    if (drive.state >= 2) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.RELEASELEFT.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 3:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        rotateGoal = 320;
                        slideGoal = 0;
                        arm.armExtend.setTargetPosition(slideGoal);
                        arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
                        arm.state++;
                    }
                    break;
                case 4:
                    if(Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 100) {
                        if (finalPos != 2) {
                            slideGoal = 850;
                            arm.armExtend.setTargetPosition(slideGoal);
                        }
                        arm.state++;
                    }
                    break;
                case 5:
                    if(Math.abs(slideGoal - arm.armExtend.getCurrentPosition()) < 10) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.RELEASERIGHT.getPosition());
                        rotateGoal = 0;
                        slideGoal = 0;
                        arm.armExtend.setTargetPosition(slideGoal);
                        arm.state++;
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