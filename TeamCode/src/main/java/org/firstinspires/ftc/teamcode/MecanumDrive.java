package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    DcMotor rightArm1;
    DcMotor leftArm1;
    DcMotor wristMotor;

//    BHI260IMU imu;
//    IMU.Parameters myIMUparameters;


    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        wristMotor = hardwareMap.get(DcMotor.class, "wrist");

        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");


        waitForStart();

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double rx = gamepad1.right_stick_x;

            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            int rightPosition = rightArm1.getCurrentPosition();
            int leftPosition = leftArm1.getCurrentPosition();

            if (leftPosition > -115 && (rt - lt) == 0)
            {


                telemetry.addData("# right position value", rightPosition);
                telemetry.addData("# left position value", leftPosition);



                telemetry.update();

                rightArm1.setPower(0.5);
                leftArm1.setPower(-.5);


            }

            rightArm1.setPower(rt - lt);
            leftArm1.setPower(-(rt - lt));


            leftFront.setPower(-(y + x + rx));
            leftBack.setPower(-(y - x + rx));
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            
            if (gamepad1.a)
            {
                rightClaw.setPosition(0);
                leftClaw.setPosition(0);
            }
            else if (gamepad1.b)
            {
                rightClaw.setPosition(90);
                leftClaw.setPosition(90);
            }
            if (gamepad1.right_bumper) {
                wristMotor.setPower(.3);
            }
            else if (gamepad1.left_bumper) {
                wristMotor.setPower(-.3);
            } else {
                wristMotor.setPower(0);
            }
        }
    }
}
