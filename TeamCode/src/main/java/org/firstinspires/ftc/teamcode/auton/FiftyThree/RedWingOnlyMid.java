package org.firstinspires.ftc.teamcode.auton.FiftyThree;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.OpenCVTeamPropDetection;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "RW Mid 2+0", group = "A")
public class RedWingOnlyMid extends LinearOpMode {
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
        drive.initRobot();
        arm.init();
        camera.findScoringPosition(OpenCVTeamPropDetection.robotPositions.RedWing,hardwareMap);
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
                    drive.drive(66,14,0.7,3000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    drive.rotateToHeading(90,0.3);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                        drive.Stop();
                    }
                    break;
                case 3:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.rotateToHeading(0, 0.3);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                        drive.Stop();
                    }
                    break;
                case 4:
                    if (arm.state > 4) {
                        drive.drive(3, -200,0.7,4500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                        drive.Stop();
                    }
                    break;
                case 5:
                    drive.drive(-1,-53,0.3,2500);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                        drive.Stop();
                    }
                    break;
                case 6:
                    if (arm.state > 7) {
                        drive.drive(57,5,0.7,3000);
                    }
            }
            switch (arm.state) {
                case 0:
                    if (drive.state > 1) {
                        arm.elbow.setPosition(Crumblz.ElbowPositions.intakePos.getPosition());
                        rotateGoal = 320;
                        slideGoal = 150;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 1:
                    if (drive.state > 2) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.openRight.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        arm.elbow.setPosition(Crumblz.ElbowPositions.stackFivePos.getPosition());
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 3:
                    if((Math.abs(arm.armExtend1.getCurrentPosition() - slideGoal) < 20) && drive.state > 3){
                        rotateGoal = 300;
                        arm.state++;
                    }
                    break;
                case 4:
                    arm.clawRight.setPosition((Crumblz.ClawPositions.grabRight.getPosition()));
                    slideGoal = 0;
                    arm.armExtend1.setTargetPosition(slideGoal);
                    arm.state++;
                case 5:
                    if (drive.state > 4) {
                        arm.elbow.setPosition(Crumblz.ElbowPositions.outtakeBackSidePos.getPosition());
                        rotateGoal = 2800;
                        slideGoal = 360;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 6:
                    if (drive.state > 5) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.openRight.getPosition());
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.openLeft.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 7:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        rotateGoal = 0;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
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