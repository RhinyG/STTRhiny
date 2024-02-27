package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;
@Autonomous(name = "FSM Test", group = "Test")
public class FSMTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    public static double p = 0.008, i = 0, d = 0;
    public static double f = 0.08;
    public static int rotateGoal;

    double armTimer;
    int slideGoal;
    double slideSpeed = 0.7;

    @Override
    public void runOpMode() {
        drive.init();
        arm.init();
        PIDController controller = new PIDController(p, i, d);

        //44.5
        waitForStart();
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
                    drive.drive(89,-85,0.7,3000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.drive(-10,42,0.4,2500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 3:
                    if (arm.state > 5) {
                        drive.drive(54,20,0.8,3500);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 4:
                    drive.drive(0,130,0.7,5000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 5:
                    drive.drive(-5,15,0.5,2000);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 6:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.drive(0,-200,6000);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 7:
                    drive.drive(-70,-45,2500);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
            }
            switch (arm.state) {
                case 0:
                    rotateGoal = 2900;
                    arm.state++;
                    break;
                case 1:
                case 11:
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
                case 13:
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
                        slideGoal = 850;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 5:
                    if(Math.abs(slideGoal - arm.armExtend1.getCurrentPosition()) < 10) {
                        arm.clawRight.setPosition(Crumblz.ClawPositions.releaseRight.getPosition());
                        rotateGoal = 320;
                        slideGoal = 0;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                        armTimer = System.currentTimeMillis();
                    }
                    break;
                case 6:
                    if (drive.state > 3) {
                        slideGoal = 500;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.openLeft.getPosition());
                        arm.state++;
                    }
                    break;
                case 7:
                    if (Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 20 && Math.abs(arm.armExtend1.getCurrentPosition() - slideGoal) < 20) {
                        arm.elbow.setPosition(Crumblz.ElbowPositions.stackFivePos.getPosition());
                        arm.state++;
                    }
                    break;
                case 8:
                    if (drive.state > 5) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.grabLeft.getPosition());
                        arm.state++;
                        armTimer = System.currentTimeMillis();
                    }
                    break;
                case 9:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        slideGoal = 200;
                        arm.armExtend1.setTargetPosition(slideGoal);
                        arm.state++;
                    }
                    break;
                case 10:
                    if (drive.state > 6) {
                        rotateGoal = 2900;
                        arm.state++;
                    }
                    break;
                case 12:
                    if (drive.state >= 7) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.releaseLeft.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
            }
//            switch (arm.state) {
//                case 0:
//                    arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
//                    arm.clawLeft.setPosition(Crumblz.ClawPositions.OPENLEFT.getPosition());
//                    rotateGoal = 310;
//                    slideGoal = 500;
//                    arm.armExtend.setTargetPosition(slideGoal);
//                    arm.state++;
//                    break;
//                case 1:
//                    if (Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 20 && Math.abs(arm.armExtend.getCurrentPosition() - slideGoal) < 20) {
//                        arm.elbow.setPosition(Crumblz.ElbowPositions.STACKFIVEPOS.getPosition());
//                        armTimer = System.currentTimeMillis();
//                        arm.state++;
//                    }
//                    break;
//                case 2:
//                    arm.clawLeft.setPosition(Crumblz.ClawPositions.GRABLEFT.getPosition());
//                    arm.state++;
//                    armTimer = System.currentTimeMillis();
//                case 3:
//                    if(armTimer + 500 < System.currentTimeMillis()) {
//                        rotateGoal = 700;
//                        slideGoal = 100;
//                    }
//            }

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