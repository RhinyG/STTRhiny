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
    double[]
            motorPowers = {0,0,0,0},
            baseLVector = toPolar(xConstant, yConstant),
            baseRVector = toPolar(-xConstant, yConstant),
            LVector,RVector,sumVector,checkVector;

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

    public void drive(double[] drivePower, double rotatePower) {
        LVector = new double[]{baseLVector[0],baseLVector[1]-drivePower[1]};
        RVector = new double[]{baseRVector[0],baseRVector[1]-drivePower[1]};

        telemetry.addData("drivePower r",drivePower[0]);
        telemetry.addData("drivePower theta",drivePower[1]);
        telemetry.addData("L rotate X",LVector[0]);
        telemetry.addData("L rotate Y",LVector[1]);
        telemetry.addData("R rotate X",RVector[0]);
        telemetry.addData("R rotate Y",RVector[1]);

        if (LVector[1] > 0.5 * Math.PI) {
            LVector[1] -= Math.PI;
        } else if (LVector[1] < -0.5 * Math.PI) {
            LVector[1] += Math.PI;
        }
        if (RVector[1] > 0.5 * Math.PI) {
            RVector[1] -= Math.PI;
        } else if (RVector[1] < -0.5 * Math.PI) {
            RVector[1] += Math.PI;
        }

        minYValue = Math.min(Math.abs(toCartesian(LVector)[1]),Math.abs(toCartesian(RVector)[1]));
        LVector[0] *= minYValue/LVector[1];
        RVector[0] *= minYValue/RVector[1];

        sumVector = toPolar(2 * toCartesian(LVector)[0] + 2 * toCartesian(RVector)[0],2 * toCartesian(LVector)[1] + 2 * toCartesian(RVector)[1]);
        powerMultiplier = drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        telemetry.addData("checkVector r",sumVector[0]);
        telemetry.addData("checkVector theta",sumVector[1]+drivePower[1]-0.5*Math.PI);

        motorPowers[0] = LVector[0] - rotatePower;
        motorPowers[1] = RVector[0] + rotatePower;
        motorPowers[2] = RVector[0] - rotatePower;
        motorPowers[3] = LVector[0] + rotatePower;

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

        if (maxPower > drivePower[0]) {
            for (int i = 0; i < 4;i++) {
                motorPowers[i] /= (maxPower*drivePower[0]);
            }
        }
        FrontL.setPower(motorPowers[0]);
        FrontR.setPower(motorPowers[1]);
        BackL.setPower(motorPowers[2]);
        BackR.setPower(motorPowers[3]);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
