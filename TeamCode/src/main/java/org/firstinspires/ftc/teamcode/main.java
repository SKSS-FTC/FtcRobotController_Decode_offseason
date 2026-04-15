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

    @Override
    public void runOpMode() {
        leftUp = hardwareMap.get(DcMotor.class, "leftUp");
        leftDown = hardwareMap.get(DcMotor.class, "leftDown");
        rightUp = hardwareMap.get(DcMotor.class, "rightUp");
        rightDown = hardwareMap.get(DcMotor.class, "rightDown");

        leftUp.setDirection(DcMotor.Direction.REVERSE);
        leftDown.setDirection(DcMotorSimple.Direction.REVERSE);
        rightUp.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDown.setDirection(DcMotorSimple.Direction.FORWARD);

        leftUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftUp.setPower(0);
        leftDown.setPower(0);
        rightUp.setPower(0);
        rightDown.setPower(0);

        waitForStart();


        while (opModeIsActive()) {
            double left_gamepad_x = gamepad1.left_stick_x;
            double left_gamepad_y = gamepad1.left_stick_y;
            double right_gamepad_x = gamepad1.right_stick_x;

            double[] DT = driveTrain.robotOriented(left_gamepad_x, left_gamepad_y, right_gamepad_x);

            leftUp.setPower(DT[0]);
            leftDown.setPower(DT[1]);
            rightUp.setPower(DT[2]);
            rightDown.setPower(DT[3]);


        }


    }

}

