package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

public class driveMotorLocalizer extends LinearOpMode {
    private DcMotorEx LF, RF, LD, RD;
    private final double radius = 0.0375; // in meter
    private final double LX = 0.16; // in meter
    private final double LY = 0.175;//in meter
    private double pastTime = 0;
    private ElapsedTime timer;
    private double[] currentPosition = {0,0,0};

    @Override
    public void runOpMode() {
        RD = hardwareMap.get(DcMotorEx.class, "RD");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LD = hardwareMap.get(DcMotorEx.class, "LD");
        LF = hardwareMap.get(DcMotorEx.class, "LF");

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            double[] angularVelocity = {LF.getVelocity(AngleUnit.RADIANS), RF.getVelocity(AngleUnit.RADIANS), LD.getVelocity(AngleUnit.RADIANS), RD.getVelocity(AngleUnit.RADIANS)};
            double currentTime = timer.seconds();
            double deltaTime = currentTime - pastTime;
            pastTime = currentTime;
            currentPosition = getPosition(currentPosition,angularVelocity,deltaTime);
            telemetry.addData("x",currentPosition[0]);
            telemetry.addData("y",currentPosition[1]);
            telemetry.addData("heading",currentPosition[2]);
            telemetry.update();
        }
    }

    private double[] getPosition(double[] pastPosition, double[] angularVelocity, double deltaTime) {
        //position[x,y,heading]
        //angularVelocity[0,1,2,3] = [LF, RF, LD, RD]
        //deltaTime in second
        double[] currentPosition = {0, 0, 0};
        currentPosition[0] = (Arrays.stream(angularVelocity).sum() * radius / 4) * deltaTime + pastPosition[0];
        currentPosition[1] = ((-angularVelocity[0] + angularVelocity[1] + angularVelocity[2] - angularVelocity[3]) * radius / 4) * deltaTime + pastPosition[1];
        currentPosition[2] = ((-angularVelocity[0] + angularVelocity[1] - angularVelocity[2] + angularVelocity[3]) * radius / 4 / (LX + LY)) * deltaTime + pastPosition[1];
        return currentPosition;
    }
}
