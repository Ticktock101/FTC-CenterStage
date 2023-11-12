package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name = "LinearSlideTest")
public class LinearSlide extends LinearOpMode {


    DcMotor linearSlide1;

    @Override
    public void runOpMode() throws InterruptedException {

          linearSlide1 = hardwareMap.get(DcMotor.class, "claw1");



        waitForStart();

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double rx = gamepad1.right_stick_x;

            linearSlide1.setPower(y);

        }
    }
}
