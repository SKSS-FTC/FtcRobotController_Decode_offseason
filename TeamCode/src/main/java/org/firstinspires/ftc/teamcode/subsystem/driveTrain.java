package org.firstinspires.ftc.teamcode.subsystem;

public class driveTrain {
    public static double[] robotOriented(double gamepad1_left_x, double gamepad1_left_y, double gamepad1_right_x) {
        double x = gamepad1_left_x;
        double y = gamepad1_left_y;
        double r = gamepad1_right_x;

        double leftUp = -x + y - r;
        double leftDown = -x - y - r;
        double rightUp = x + y - r;
        double rightDown = x - y - r;

        return new double[]{leftUp, leftDown, rightUp, rightDown};

    }

}
