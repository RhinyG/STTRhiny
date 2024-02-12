package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Crumblz;
import org.firstinspires.ftc.teamcode.robotParts.newAutonMethods;

@Autonomous(name = "FSM Test", group = "Test")
public class FSMTest extends LinearOpMode {
    newAutonMethods drive = new newAutonMethods(this);
    Crumblz arm = new Crumblz(this);

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
            switch (arm.state) {
                case 0:
                    arm.elbow.setPosition(Crumblz.ElbowPositions.INTAKEPOS.getPosition());
                    arm.clawRight.setPosition(Crumblz.ClawPositions.OPENRIGHT.getPosition());
                    arm.armRotate.setTargetPosition(370);
                    arm.state++;
                    break;
                case 1:
                    arm.FSMArm(370);
                    break;
                case 2:
                    arm.armExtend.setTargetPosition(850);
                    arm.state++;
                    break;
                case 3:
                    arm.FSMSlide(850);
                case 4:
                    arm.elbow.setPosition(Crumblz.ElbowPositions.STACKFIVEPOS.getPosition());
                    arm.clawRight.setPosition(Crumblz.ClawPositions.GRABRIGHT.getPosition());
            }
            telemetry.update();
        }
    }
}