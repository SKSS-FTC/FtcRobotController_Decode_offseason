package org.firstinspires.ftc.teamcode.Subsystem;

public class driveTrain {
    public double x, y, r;

    public double[] robotOriented(double gamepad1_left_x, double gamepad1_left_y, double gamepad1_right_x, double gamepad1_right_y) {
        x = gamepad1_left_x;
        y = gamepad1_left_y;
        r = gamepad1_right_x;

        double leftUp = -x + y - r;
        double leftDown = -x - y - r;
        double rightUp = x + y - r;
        double rightDown = x - y - r;

        return new double[]{leftUp, leftDown, rightUp, rightDown};

    }

}
