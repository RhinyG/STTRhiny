package org.firstinspires.ftc.teamcode.auton.FiftyThreePoints;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "50 FSM", group = "Test")
public class FiftyFSM extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    double armTimer;

    @Override
    public void runOpMode() {
        drive.init(hardwareMap);
        arm.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Global Drive state",drive.state);
            telemetry.addData("driveState",drive.driveState);
            telemetry.addData("rotateState",drive.rotateState);
            telemetry.addData("arm state",arm.state);
            switch (drive.state) {
                case 0:
                    drive.driveState = 0;
                    drive.state++;
                    break;
                case 1:
                    drive.FSMDrive(77,-59,0.7,3000,telemetry);
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 2:
                    if (arm.state >= 4) {
                        drive.FSMDrive(10, -25, 0.4, 2500, telemetry);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 3:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        drive.FSMDrive(-10,35,0.4,2500,telemetry);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 4:
                    if (arm.state > 11) {
                        drive.FSMDrive(-70,-30,0.5,4000,telemetry);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                    }
            }
            switch (arm.state) {
                case 0:
                    //TODO: make one method with that setTargetPosition included (like FSMDrive)
                    arm.armRotate.setTargetPosition(2900);
                    arm.state++;
                    break;
                case 1:
                    arm.FSMArm(2900);
                    break;
                case 2:
                    arm.armExtend.setTargetPosition(500);
                    arm.state++;
                    break;
                case 3:
                    arm.FSMSlide(500);
                    break;
                case 4:
                    if (drive.state >= 3) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.RELEASELEFT.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 5:
                    if (System.currentTimeMillis() > 100 + armTimer) {
                        arm.armExtend.setTargetPosition(0);
                        arm.armRotate.setTargetPosition(0);
                        arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
                        arm.state++;
                    }
                    break;
                case 6:
                    arm.FSMSlide(0);
                    break;
                case 7:
                    arm.FSMArm(0);
                    break;
                case 8:
                    arm.armExtend.setTargetPosition(850);
                    arm.state++;
                    break;
                case 9:
                    arm.FSMSlide(850);
                    break;
                case 10:
                    arm.clawRight.setPosition(Crumblz.ClawPositions.RELEASERIGHT.getPosition());
                    arm.armExtend.setTargetPosition(0);
                    arm.state++;
                case 11:
                    arm.FSMSlide(0);
            }
            telemetry.update();
        }
    }
}