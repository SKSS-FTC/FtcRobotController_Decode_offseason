package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystem.driveTrain;

import java.util.Locale;

@TeleOp
public class main extends LinearOpMode {
    public DcMotor leftUp, leftDown, rightUp, rightDown;
    public DcMotor frontIntake;

    public IMU imu;

    public Orientation angles;
    private YawPitchRollAngles orientation;
    private double yaw = 0;

    private driveTrain driveTrainInstance = new driveTrain();


    @Override
    public void runOpMode() {


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));


        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");

        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotor.Direction.REVERSE);
        rightUp.setDirection(DcMotor.Direction.REVERSE);
        rightDown.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setDirection(DcMotor.Direction.REVERSE);

        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftUp.setPower(0);
        leftDown.setPower(0);
        rightUp.setPower(0);
        rightDown.setPower(0);
        frontIntake.setPower(0);

        telemetry.addData("this is the true one","1");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            double left_gamepad_x = gamepad1.left_stick_x;
            double left_gamepad_y = gamepad1.left_stick_y;
            double right_gamepad_x = gamepad1.right_stick_x;
            boolean input = gamepad1.right_bumper;

            if (input) {
                frontIntake.setPower(0.3);
            } else {
                frontIntake.setPower(0);
            }

            orientation   = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw(AngleUnit.DEGREES);


            double[] DT = driveTrainInstance.robotOriented(left_gamepad_x, left_gamepad_y, right_gamepad_x, yaw);

            telemetry.addLine()
                    .addData("yaw", new Func<String>() {
                        @Override public String value() {
                            return formatAngle(AngleUnit.DEGREES, yaw);
                        }
                    });

            leftUp.setPower(DT[0]);
            leftDown.setPower(DT[1]);
            rightUp.setPower(DT[2]);
            rightDown.setPower(DT[3]);


        }


    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

