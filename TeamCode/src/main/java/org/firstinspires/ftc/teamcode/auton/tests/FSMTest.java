package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "FSM Test", group = "Test")
public class FSMTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

    double armTimer;
    int rotateGoal;
    double rotateSpeed = 1.0;
    int slideGoal;
    double slideSpeed = 0.7;

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
                    if (arm.state >= 2) {
                        drive.FSMDrive(0,10,0.5,1000,telemetry);
                    }
                    if (drive.driveState == 2) {
                        drive.state++;
                        drive.driveState = 0;
                    }
                    break;
                case 1:
                    if (arm.state >= 3) {
                        drive.FSMDrive(0,-10,0.5,1000,telemetry);
                    }
            }
            switch (arm.state) {
                case 0:
                    arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
                    arm.clawLeft.setPosition(Crumblz.ClawPositions.OPENLEFT.getPosition());
                    arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
                    rotateGoal = 290;
                    arm.armRotate.setTargetPosition(rotateGoal);
                    slideGoal = 50;
                    arm.armExtend.setTargetPosition(slideGoal);
                    arm.state++;
                    break;
                case 1:
                    if (Math.abs(arm.armRotate.getCurrentPosition() - rotateGoal) < 20) {
                        arm.elbow.setPosition(Crumblz.ElbowPositions.STACKFIVEPOS.getPosition());
                        armTimer = System.currentTimeMillis();
                        arm.state++;
                    }
                    break;
                case 2:
                    if (drive.state >= 1) {
                        arm.clawLeft.setPosition(Crumblz.ClawPositions.GRABLEFT.getPosition());
                        arm.state++;
                    }
            }
            arm.FSMArm(rotateGoal,rotateSpeed);
            arm.FSMSlide(slideGoal,slideSpeed);
            telemetry.update();
        }
    }
}