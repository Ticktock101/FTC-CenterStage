package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "MecanumDrive")
public class MecanumDrive extends LinearOpMode {

    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;


    @Override
    public void runOpMode() throws InterruptedException {
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            leftFront.setPower((y + x + turn) / denominator);
            leftBack.setPower((y - x + turn) / denominator);
            rightFront.setPower((y - x - turn) / denominator);
            rightBack.setPower((y + x - turn) / denominator);

        }
    }
}
