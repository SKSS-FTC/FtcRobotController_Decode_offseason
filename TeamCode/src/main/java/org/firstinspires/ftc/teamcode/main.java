package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystem.driveTrain;

@TeleOp
public class main extends LinearOpMode {
    public DcMotor leftUp, leftDown, rightUp, rightDown;
    public DcMotor frontIntake;

    @Override
    public void runOpMode() {
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

            double[] DT = driveTrain.robotOriented(left_gamepad_x, left_gamepad_y, right_gamepad_x);

            leftUp.setPower(DT[0]);
            leftDown.setPower(DT[1]);
            rightUp.setPower(DT[2]);
            rightDown.setPower(DT[3]);


        }


    }

}

