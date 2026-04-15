package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

// the following program using radian in heading, rad/s in angular velocity, meter in all unit
// take moving clockwise (turning right in heading) as positive

@TeleOp
public class driveMotorLocalizer extends LinearOpMode {
    private DcMotorEx LF, RF, LD, RD;
    private final double radius = 0.0375; // in meter
    private final double LX = 0.16; // in meter
    private final double LY = 0.175;//in meter
    private double pastTime = 0;
    private ElapsedTime timer;
    private boolean enableDrive = false;
    private double[] currentPosition = {0, 0, 0};

    @Override
    public void runOpMode() {
        RD = hardwareMap.get(DcMotorEx.class, "RD");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LD = hardwareMap.get(DcMotorEx.class, "LD");
        LF = hardwareMap.get(DcMotorEx.class, "LF");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LD.setDirection(DcMotorSimple.Direction.REVERSE);
        RD.setDirection(DcMotorSimple.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if (enableDrive) {
                double x = gamepad1.left_stick_x;
                double y = gamepad1.left_stick_y;
                double rotation = gamepad1.right_stick_x;
                LF.setPower(min(1, max(-1, x - y - rotation)));
                RF.setPower(min(1, max(-1, x + y + rotation)));
                LD.setPower(min(1, max(-1, x + y - rotation)));
                RD.setPower(min(1, max(-1, x - y + rotation)));
            } else {
                LF.setPower(0);
                RF.setPower(0);
                LD.setPower(0);
                RD.setPower(0);
            }
            if (gamepad1.cross){
                enableDrive = false;
            }else if (gamepad1.circle){
                enableDrive = true;
            }
            double[] angularVelocity = {LF.getVelocity(AngleUnit.RADIANS), RF.getVelocity(AngleUnit.RADIANS), LD.getVelocity(AngleUnit.RADIANS), RD.getVelocity(AngleUnit.RADIANS)};
            double currentTime = timer.seconds();
            double deltaTime = currentTime - pastTime;
            pastTime = currentTime;
            currentPosition = getPosition(currentPosition, angularVelocity, deltaTime);
            telemetry.addData("LF, RF, LD, RD", angularVelocity);
            telemetry.addData("x", currentPosition[0]);
            telemetry.addData("y", currentPosition[1]);
            telemetry.addData("heading", currentPosition[2]);
            telemetry.update();
        }
    }

    private double[] getPosition(double[] pastPosition, double[] angularVelocity, double deltaTime) {
        //position[x,y,heading]
        //angularVelocity[0,1,2,3] = [LF, RF, LD, RD]
        //deltaTime in second
        double[] currentPosition = {0, 0, 0};
        double[] deltaPosition_robotFrame = {(Arrays.stream(angularVelocity).sum() * radius / 4) * deltaTime,
                ((-angularVelocity[0] + angularVelocity[1] + angularVelocity[2] - angularVelocity[3]) * radius / 4) * deltaTime,
                ((-angularVelocity[0] + angularVelocity[1] - angularVelocity[2] + angularVelocity[3]) * radius / 4 / (LX + LY)) * deltaTime
        };
        //rotate the delta position to the field frame by -1 * heading
        double[] deltaPosition_fieldFrame = {deltaPosition_robotFrame[0] * Math.cos(-pastPosition[2]) - deltaPosition_robotFrame[1] * Math.sin(-pastPosition[2]),
                deltaPosition_robotFrame[0] * Math.sin(-pastPosition[2]) - deltaPosition_robotFrame[1] * Math.cos(-pastPosition[2]),
                deltaPosition_robotFrame[2]};
        currentPosition[0] = deltaPosition_fieldFrame[0] + pastPosition[0];
        currentPosition[1] = deltaPosition_fieldFrame[1] + pastPosition[1];
        currentPosition[2] = deltaPosition_fieldFrame[2] + pastPosition[2];
        return currentPosition;
    }
}
