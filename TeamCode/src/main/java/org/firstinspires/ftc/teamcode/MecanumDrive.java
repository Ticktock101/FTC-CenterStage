package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp (name = "MecanumDrive")
public class MecanumDrive extends LinearOpMode {

    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    Servo rightClaw;
    Servo leftClaw;

//    BHI260IMU imu;
//    IMU.Parameters myIMUparameters;


    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");

        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

//        imu = hardwareMap.get(BHI260IMU.class, "imu");

//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                        )
//                )
//        );

//
//
//        myIMUparameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
//                )
//        );
//
//        imu.initialize(myIMUparameters);



        waitForStart();

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double rx = gamepad1.right_stick_x;


//            double botHeading = -imu.getRobotOrientation()

//            double botHeading = -imu.getRobotOrientation(
//                    AxesReference.INTRINSIC,
//                    AxesOrder.XYZ,
//                    AngleUnit.DEGREES
//            ).firstAngle;

//            double botHeading = -imu.getRobotOrientation().firstAngle;

//            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
//            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
//
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            if (gamepad1.a)
            {
                rightClaw.setPosition(-90);
                leftClaw.setPosition(90);
            }
            else if (gamepad1.b)
            {
                rightClaw.setPosition(0);
                leftClaw.setPosition(0);
            }


        }
    }
}
