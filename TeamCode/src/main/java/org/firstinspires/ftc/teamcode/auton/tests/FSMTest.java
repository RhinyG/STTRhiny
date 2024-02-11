package org.firstinspires.ftc.teamcode.auton.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.FSMMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robotParts.Crumblz;

@Autonomous(name = "FSM Test", group = "Test")
public class FSMTest extends LinearOpMode {
    FSMMecanumDrivetrain drive = new FSMMecanumDrivetrain(this);
    Crumblz arm = new Crumblz(this);

    @Override
    public void runOpMode() {
        drive.init();
        waitForStart();
        if (opModeIsActive()){
            //TODO: see if this switch case works. Otherwise, use commented out one.
            switch (drive.state) {
                case 0:
                    drive.driveState = 0;
                    drive.FSMDrive(0,50,0.5,4000);
                    break;
                case 1:
                    //TODO: see if this works
                    //TODO: better way to provide feedback to the other switch
                    if (System.currentTimeMillis() > 5000) {
                        drive.rotateState = 0;
                        drive.FSMRotate(90, 0.4);
                    }
                    break;
                case 2:
                    drive.driveState = 0;
                    drive.FSMDrive(50,0,0.5,4000);
                    break;
            }
//            switch (drive.state) {
//                case 0:
//                    drive.setFSMDrive(0,100,0.5);
//                    break;
//                case 1:
//                    drive.runFSMDrive(0.5,8000);
//                    break;
//                case 2:
//                    drive.endFSMDrive();
//                    break;
//            }
            //TODO: Transfer armMethods to FSM methods
            switch (arm.state) {
                case 0:
                    if (drive.state >= 1) {
                        telemetry.addLine("OMG this shit works");
                        arm.state++;
                    }
                    break;
                case 1:
                    telemetry.addLine("OMG that shit has worked");
            }
            arm.updateElbow();
            telemetry.update();
        }
    }
}