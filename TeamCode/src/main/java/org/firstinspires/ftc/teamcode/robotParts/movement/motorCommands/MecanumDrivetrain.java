package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

//TODO: is extends RobotPart really worth it/the proper way?
public class MecanumDrivetrain extends RobotPart {
    public DcMotorEx FrontL, FrontR, BackL, BackR;
//    public DcMotorEx[] motors = {FrontL,FrontR,BackL,BackR};
    LinearOpMode myOpMode;

    final double
            angle = Math.PI * 1/3,
            xConstant = toCartesian(1,angle)[0],
            yConstant = toCartesian(1,angle)[1];
    double minYValue,powerMultiplier,maxPower;
    double[] motorPowers = {0,0,0,0};

    Vector2d baseLVector = new Vector2d(xConstant, yConstant);
    Vector2d baseRVector = new Vector2d(-xConstant, yConstant);
    Vector2d LVector,RVector,sumVector,checkVector = new Vector2d();

    public MecanumDrivetrain(LinearOpMode opmode) {
        telemetry = opmode.telemetry;
        myOpMode = opmode;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * @param map - Gives a hardwareMap from the opmode for the method to use. Not having this parameter would result in an NPE.
     *            This can alternatively be done with myOpMode.hardwareMap.get but that's longer so we don't.
     */
    public void init(HardwareMap map, boolean TeleOp) {
        FrontL = map.get(DcMotorEx.class, "left_front");
        FrontR = map.get(DcMotorEx.class, "right_front");
        BackL = map.get(DcMotorEx.class, "left_back");
        BackR = map.get(DcMotorEx.class, "right_back");

        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackR.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (TeleOp) {
            initIMU(map);
        }
    }

    public void drive(Vector2d drivePower, double rotatePower) {
        LVector = baseLVector;
        RVector = baseRVector;
        LVector = LVector.rotateBy(-drivePower.angle());
        RVector = RVector.rotateBy(-drivePower.angle());
        if (Math.abs(LVector.angle()) > 0.5 * Math.PI) {
            LVector = LVector.unaryMinus();
        }
        if (Math.abs(RVector.angle()) > 0.5 * Math.PI) {
            RVector = RVector.unaryMinus();
        }

        minYValue = Math.min(LVector.getY(),RVector.getY());
        LVector = LVector.scale(minYValue/LVector.getY());
        RVector = RVector.scale(minYValue/RVector.getY());

        sumVector = LVector.plus(LVector).plus(RVector).plus(RVector);
        powerMultiplier = drivePower.magnitude()/sumVector.magnitude();
        LVector = LVector.scale(powerMultiplier);
        RVector = RVector.scale(powerMultiplier);
        checkVector = sumVector.scale(powerMultiplier);

        motorPowers[0] = LVector.magnitude() - rotatePower;
        motorPowers[1] = RVector.magnitude() + rotatePower;
        motorPowers[2] = RVector.magnitude() - rotatePower;
        motorPowers[3] = LVector.magnitude() + rotatePower;

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

        if (maxPower > 1) {
            for (int i = 0; i < 4;i++) {
                motorPowers[i] /= maxPower;
            }
        }
//        FrontL.setPower(motorPowers[0]);
//        FrontR.setPower(motorPowers[1]);
//        BackL.setPower(motorPowers[2]);
//        BackR.setPower(motorPowers[3]);
        telemetry.addData("baseL",baseLVector);
        telemetry.addData("baseR",baseRVector);
        telemetry.addData("LVector",LVector);
        telemetry.addData("RVector",RVector);
        telemetry.addData("drivePower",drivePower);
        telemetry.addData("check",checkVector);
        telemetry.addData("rotatePower",rotatePower);
        telemetry.addData("minY",minYValue);
        telemetry.addData("sum",sumVector);
        telemetry.addData("maxpower",maxPower);
        telemetry.addData("FL",motorPowers[0]);
        telemetry.addData("FR",motorPowers[1]);
        telemetry.addData("BL",motorPowers[2]);
        telemetry.addData("BR",motorPowers[3]);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
