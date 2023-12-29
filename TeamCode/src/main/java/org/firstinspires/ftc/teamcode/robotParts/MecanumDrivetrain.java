package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrivetrain {
    private final LinearOpMode myOpMode;
    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;
    double current_target_heading = 0;
    IMU imu;
    double WHEEL_RADIUS = 48;//mm
    double GEAR_RATIO = 1/13.7;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
    final double odoMultiplier = 1.76;
    public MecanumDrivetrain(LinearOpMode opmode) {myOpMode = opmode;}

    /**
     * This methods initialises the mecanum drivetrain and sets all the directions and modes to their correct settings.
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     *            This can alternatively be done with myOpMode.hardwareMap.get but that's longer so we don't.
     */
    public void init(HardwareMap map) {
        FrontL = map.get(DcMotor.class, "left_front");
        FrontR = map.get(DcMotor.class, "right_front");
        BackL = map.get(DcMotor.class, "left_back");
        BackR = map.get(DcMotor.class, "right_back");

//        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontL.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.FORWARD);
        BackR.setDirection(DcMotorSimple.Direction.REVERSE);

//        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);

        resetIMU(map);
    }

    /**
     * This method is a shorthand for the regular driveY, with the speed and telemetry filled in already.
     * This makes long strings of auton with similar driveY's easier to write and read.
     * @param position - How far you want to travel, in centimeters
     */
    public void driveY (double position){
        driveY(position,0.3, myOpMode.telemetry);
    }

    /**
     * This method is for autonomously driving in the primary axis of motion, which, for us, means
     * forwards or backwards. You could rename this to driveX as the X-axis is usually the primary
     * axis for vehicles, but we chose not to. This method does not remember its coordinate system because of the calibrateEncoders, it resets to
     * zero each time.
     * @param position - How far you want to travel, in centimeters. A positive position means forward, a negative position means backwards.
     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
     * @param telemetry - Gives telemetry from the opmode for the method to use. Not having this parameter would result in an NPE.
     *                  This can alternatively be done with myOpMode.telemetry.addData but that's longer so we don't.
     */
    public void driveY(double position, double speed, Telemetry telemetry) {//TODO: make work like newAutonMethods
        calibrateEncoders();
        double Kp = 0.03;
        double turn;
        double TargetHeading = getCurrentHeadingDegrees();
        double OdoY_Pos = FrontL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(TargetHeading-getCurrentHeadingDegrees());

            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("turn",turn);
            telemetry.addData("CurrentHeading", getCurrentHeadingDegrees());
            telemetry.addData("TargetHeading", TargetHeading);
            telemetry.update();

            FrontL.setPower(speed + turn);
            BackL.setPower(speed + turn);
            BackR.setPower(speed - turn);
            FrontR.setPower(speed - turn);

            OdoY_Pos = FrontL.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    /**
     * This method is a shorthand for the regular driveX, with the speed and telemetry filled in already.
     * This makes long strings of auton with similar driveX's easier to write and read.
     * @param position - How far you want to travel, in centimeters
     */
    public void driveX(double position){
        driveX(position,0.3, myOpMode.telemetry);
    }

    /**
     * This method is for autonomously driving in the secondary axis of motion, which, for us, means
     * to the left or right. You could rename this to driveY as the Y-axis is usually the secondary
     * axis for vehicles, but we chose not to. This method does not remember its coordinate system because of the calibrateEncoders, it resets to
     * zero each time.
     * @param position - How far you want to travel, in centimeters. A positive position means to the right, a negative position means to the left.
     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
     * @param telemetry - Gives telemetry from the opmode for the method to use. Not having this parameter would result in an NPE.
     *                  This can alternatively be done with myOpMode.telemetry.addData but that's longer so we don't.
     */
    public void driveX(double position, double speed, Telemetry telemetry) {
        speed = speed * -1;
        calibrateEncoders();
        double Kp = 0.03;
        double turn;
        double heading = current_target_heading;
        double OdoX_Pos = -BackL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoX_Pos;
        while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            turn = Kp*Math.abs(speed)*(heading-getCurrentHeadingDegrees());

            telemetry.addData("tick", tick);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("Turn",turn);
            telemetry.update();

            FrontL.setPower(-speed + turn);
            FrontR.setPower(speed - turn);
            BackL.setPower(speed + turn);
            BackR.setPower(-speed - turn);

            OdoX_Pos = -BackL.getCurrentPosition();
            dPos = tick - OdoX_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }

    /**
     * This method is a shorthand for the regular rotateToHeading, with the speed and telemetry filled in already.
     * This makes long strings of auton with similar rotateToHeading's easier to write and read.
     * @param target_heading - To which angle you want to turn, in degrees. This means that with two successive
     *                       rotateToHeading(90)'s, the second rotateToHeading is useless, as you are at that heading already.
     */
    public void rotateToHeading(double target_heading){
        rotateToHeading(target_heading,0.2, myOpMode.telemetry);
    }

    /**
     * This method is for autonomously driving turning around the Z-axis or yaw. It might make sense to reverse target_heading,
     * so that positive is counterclockwise, because that is how degrees usually work. We preferred clockwise as it made more intuitive sense.
     * @param target_heading - To which angle you want to turn, in degrees. This means that with two successive
     *                       rotateToHeading(90)'s, the second rotateToHeading is useless, as you are at that heading already.
     *                       You could change this by making the parameter the dHeading (delta Heading). A positive target_heading means clockwise,
     *                       a negative heading means counterclockwise. It's interval is therefore [-180,180]
     * @param speed - Is how fast you want to travel. In autonomous, it is generally smart to move slowly,
     *              because that gives you more precision. If you run into time constraints, you can try going quicker.
     * @param telemetry - Gives telemetry from the opmode for the method to use. Not having this parameter would result in an NPE.
     *                  This can alternatively be done with myOpMode.telemetry.addData but that's longer so we don't.     */
    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        double current_heading = -getCurrentHeadingDegrees();
        double dHeading = target_heading - current_heading;
        double direction;
        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < 1) && myOpMode.opModeIsActive()) {
            direction = checkDirection(current_heading-target_heading);

            FrontL.setPower(-speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower(-speed * direction);
            BackR.setPower(speed * direction);

            current_heading = getCurrentHeadingDegrees();
            dHeading = target_heading - current_heading;
            telemetry.addData("curHeading", current_heading);
            telemetry.addData("dHeading",dHeading);
            telemetry.update();
        }
        calibrateEncoders();
        Stop();
        current_target_heading = target_heading;
    }

    /**
     * This method checks if the parameter is positive or negative, which is used to check if the
     * rotateToHeading needs to rotate clockwise or counterclockwise.
     * @param val - Interval can technically be anything, but since we use it for motor power should
     *            range from [-1,1].
     * @return - Returns -1 if a negative number is inputted and 1 if a positive number is inputted.
     */
    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    /**
     * This method is for driving around in Tele-Op. RobotCentric drive means that directions are
     * relative to the robot. This might be slightly difficult to learn at first but it is not impossible
     * to us. Certain drivers prefer RobotCentric drive.
     * This code has a 'slow mode' toggle and also corrects for input overflow, when motor powers become > 1.
     */
    public void RobotCentric() {
        double FWD = myOpMode.gamepad1.left_stick_y;
        double STR = myOpMode.gamepad1.left_stick_x;
        double ROT = myOpMode.gamepad1.right_stick_x;
        double speed = 1.0;
        double maxPower = 1.0;

        if(myOpMode.gamepad1.right_stick_button){speed = 0.5;}
        double FrontLPower = ((-FWD + STR + ROT) * speed);
        double FrontRPower = ((-FWD - STR - ROT) * speed);
        double BackLPower = ((-FWD - STR + ROT) * speed);
        double BackRPower = ((-FWD + STR - ROT) * speed);

        maxPower = Math.max(maxPower, Math.abs(FrontLPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRPower));
        maxPower = Math.max(maxPower, Math.abs(BackLPower));
        maxPower = Math.max(maxPower, Math.abs(BackRPower));

        FrontL.setPower(FrontLPower/maxPower);
        FrontR.setPower(FrontRPower/maxPower);
        BackL.setPower(BackLPower/maxPower);
        BackR.setPower(BackRPower/maxPower);
    }

    public void BackBoardCorrection(double dHeading) {
        double speed = 0.5;

        if (Math.abs(dHeading) > 2.0){
            if(dHeading < 0.0){speed = -0.5;}
            FrontL.setPower(speed);
            FrontR.setPower(-speed);
            BackL.setPower(speed);
            BackR.setPower(-speed);
        } else {
            Stop();
        }
    }

    public void IMUBackBoardCorrection() {
        double margin = 2.0;
        double speed = 0.5;
        double currentHeading = getCurrentHeadingDegrees();
        double targetHeading = -90.0;
        double dHeading = Math.abs(currentHeading - targetHeading);
        if (currentHeading < targetHeading) {
            speed = -0.5;
            if (dHeading > margin) {
                FrontL.setPower(speed);
                FrontR.setPower(-speed);
                BackL.setPower(speed);
                BackR.setPower(-speed);
            } else {
                FrontL.setPower((dHeading/margin) * -0.1);
                FrontR.setPower((dHeading/margin) * 0.1);
                BackL.setPower((dHeading/margin) * -0.1);
                BackR.setPower((dHeading/margin) * 0.1);
            }
        } else if (currentHeading > targetHeading) {
            if (dHeading > margin) {
                FrontL.setPower(speed);
                FrontR.setPower(-speed);
                BackL.setPower(speed);
                BackR.setPower(-speed);
            } else {
                FrontL.setPower((dHeading/margin) * 0.1);
                FrontR.setPower((dHeading/margin) * -0.1);
                BackL.setPower((dHeading/margin) * 0.1);
                BackR.setPower((dHeading/margin) * -0.1);
            }
        }
    }

    /**
     * This method is for Tele-Op. FieldCentric means that directions are relative to the field and will not
     * change based on the robots orientation. This is much easier to learn and, for some drivers, makes more
     * complex maneuvering possible. The code itself is a copy of RobotCentric, with correction to FWD, STR and
     * ROT, which need to be relative to the robot.
     * @param speed - Decides at what speed the entire method should function. It's interval is [0, 1].
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     *            This can alternatively be done with myOpMode.hardwareMap.get but that's longer so we don't.
     * @param telemetry - Gives telemetry from the opmode for the method to use. Not having this parameter would result in an NPE.
     *                  This can alternatively be done with myOpMode.telemetry.addData but that's longer so we don't.
     */
    public void FieldCentric(double speed, HardwareMap map, Telemetry telemetry) {
        double theta = getCurrentHeadingRadians();
        double FWD = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
        double STR = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
        double ROT = myOpMode.gamepad1.right_stick_x;
        double maxPower = 1.0; //TODO: Igor says this felt slow, probably cause that gets too high, print it and look at it.

        double FrontLPower = ((-FWD + STR + ROT) * speed);
        double FrontRPower = ((-FWD - STR - ROT) * speed);
        double BackLPower = ((-FWD - STR + ROT) * speed);
        double BackRPower = ((-FWD + STR - ROT) * speed);

        maxPower = Math.max(maxPower, Math.abs(FrontLPower));
        maxPower = Math.max(maxPower, Math.abs(FrontRPower));
        maxPower = Math.max(maxPower, Math.abs(BackLPower));
        maxPower = Math.max(maxPower, Math.abs(BackRPower));

        FrontL.setPower(FrontLPower/maxPower);
        FrontR.setPower(FrontRPower/maxPower);
        BackL.setPower(BackLPower/maxPower);
        BackR.setPower(BackRPower/maxPower);

        if (myOpMode.gamepad1.right_bumper && myOpMode.gamepad1.left_bumper) {
            resetIMU(map);
        }
        telemetry.addData("Current heading",getCurrentHeadingDegrees());
        telemetry.update();
    }

    /**
     * This method is used at the end of other methods to ensure all motors are at zero power. The alternative,
     * stopping with commands, might result in never stopping or a large distance at the end of each method,
     * during which the robot slowly slows to a halt.
     */
    public void Stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * This method uses the 2023 universal IMU code (works for both BHI260AP and BNO055 chips) and
     * the Control Hub's or Expansion Hub's IMU chip to figure out the robots heading. This value
     * does not reset when switching from Autonomous to Tele-Op opmodes. We use this method for autonomous
     * because we have more intuition with degrees.
     * @return - Returns the robots current heading, in degrees.
     */
    public double getCurrentHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (orientation.getYaw(AngleUnit.DEGREES));
    }

    /**
     * This method uses the 2023 universal IMU code (works for both BHI260AP and BNO055 chips) and
     * the Control Hub's or Expansion Hub's IMU chip to figure out the robots heading. This value
     * does not reset when switching from Autonomous to Tele-Op opmodes. We use this method for FieldCentric,
     * because the Math.sin and Math.cos methods use radians.
     * @return - Returns the robots current heading, in radians.
     */
    public double getCurrentHeadingRadians() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (orientation.getYaw(AngleUnit.RADIANS));
    }

    /**
     * This method resets (or initialises, if it's the first time) the IMU. This is used in init and
     * FieldCentric.
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     *            This can alternatively be done with myOpMode.hardwareMap.get but that's longer so we don't.
     */
    public void resetIMU(HardwareMap map) {//TODO: This don't work in FieldCentric
        imu = map.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * This method resets the encoders to a new zero position, so the next method starts from position zero again.
     */
    public void calibrateEncoders() {
        FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}