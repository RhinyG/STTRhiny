package org.firstinspires.ftc.teamcode.robotParts.movement.motorCommands;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robotParts.RobotPart;

//TODO: is extends RobotPart really worth it/the proper way?
public class MecanumDrivetrain extends RobotPart {
    public DcMotorEx FrontL, FrontR, BackL, BackR;
    double
            angle = Math.PI * 2/3,
            mecanumVectorX = toCartesian(1,angle)[0],
            mecanumVectorY = toCartesian(1,angle)[1];
    Vector2d FrontLVector, BackRVector = new Vector2d(mecanumVectorX,mecanumVectorY).normalize();
    Vector2d FrontRVector, BackLVector = new Vector2d(-mecanumVectorX,mecanumVectorY).normalize();

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

        if (TeleOp) {
            initIMU(map);
        }
    }

    public void drive(Vector2d drivePower, double rotatePower) {
        FrontLVector.rotateBy(-drivePower.angle());
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
