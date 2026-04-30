package org.firstinspires.ftc.teamcode.subsystem;

public class driveTrain {
    private double pastYaw = -99999999;
    private boolean isFirstRun = false;

    public double[] robotOriented(double gamepad1_left_x, double gamepad1_left_y, double gamepad1_right_x, double yaw) {
        double x = gamepad1_left_x;
        double y = gamepad1_left_y;
        double r = 0;

        if (!isFirstRun){
            pastYaw = yaw;
            isFirstRun = true;
        }
        double currentYaw = yaw - pastYaw;
//        if (reset needed ){
//            pastYaw = yaw;
//        }
        if (gamepad1_right_x == 0) {
            r = yaw * 0.002;
        } else {
            r = gamepad1_right_x;
        }

        double leftUp = -x + y - r;
        double leftDown = x + y - r;
        double rightUp = x + y + r;
        double rightDown = -x + y + r;

        return new double[]{leftUp, leftDown, rightUp, rightDown};

    }

}
