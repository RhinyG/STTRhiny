package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(group = "Tests")
public class GlobalCoordinates extends LinearOpMode {
    double k = 0.1, robotX = 0, robotY = 0, dX, dY;
    public static int startOffset = 0;
    /**
     * heading: to the right is 90 degrees.
     */
    public static double targetX = 0, targetY = 0, heading = 0, cur_x = 0, cur_y = 0;

    private double[] calculateOdoDistance(double x, double y, double heading) {
        double[][] eMatrix = {{Math.cos(heading), -Math.sin(heading)}, {Math.sin(heading), Math.cos(heading)}};
        double phi = 0;
        double[] gradientPhi = {x, -y};
        double[] term1 = {
                eMatrix[0][0] * gradientPhi[0] + eMatrix[0][1] * gradientPhi[1],
                eMatrix[1][0] * gradientPhi[0] + eMatrix[1][1] * gradientPhi[1]
        };
        double term2 = k * phi;
        double[] term2Vector = {term2 * gradientPhi[0], term2 * gradientPhi[1]};
        return new double[]{term1[0] - term2Vector[0], term1[1] - term2Vector[1]};
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            dX = targetX - cur_x;
            dY = targetY - cur_y;
            double[] odoDistances = calculateOdoDistance(dX,-dY, Math.toRadians(heading + startOffset));
            robotX = odoDistances[0];
            robotY = odoDistances[1];
            telemetry.addData("robotX",(int)robotX);
            telemetry.addData("robotY",(int)robotY);
            telemetry.update();
        }
    }
}
