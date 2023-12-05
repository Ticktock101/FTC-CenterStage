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

    DcMotor slide;

    Servo airplane;

    double targetArmHeight = 0;

//    BHI260IMU imu;
//    IMU.Parameters myIMUparameters;




    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        wristMotor = hardwareMap.get(DcMotor.class, "wrist");
        slide = hardwareMap.get(DcMotor.class, "slide");

        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        airplane = hardwareMap.get(Servo.class, "planeLauncher");
        airplane.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        boolean isReset = false;

        int armRightStart = rightArm1.getCurrentPosition();
        int armLeftStart = leftArm1.getCurrentPosition();

        int wristStart = wristMotor.getCurrentPosition();


        boolean gripClaw = true;
        boolean aButtonUpdate = false;

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double rx = gamepad1.right_stick_x;

            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            int rightPosition = rightArm1.getCurrentPosition();
            int leftPosition = leftArm1.getCurrentPosition();

            telemetry.addData("wrist start", wristStart);
            telemetry.addData("wrist curr", wristMotor.getCurrentPosition());


            double direction = 0;
            double wristDirection = 0;
            // Ryan Strobel is a legend
            if (!isReset) {
                 direction = rt - lt;
                 if (gamepad1.right_bumper) {wristDirection = 1;}
                 else if (gamepad1.left_bumper) {wristDirection = -1;}
                 else {wristDirection = 0;};
            }
            else {
                direction = -1 * (rightArm1.getCurrentPosition() - armRightStart);
                wristDirection = (wristMotor.getCurrentPosition() - wristStart);
                if (Math.abs(direction) < 10) {
                    isReset = false;
                }
            }

                if (direction == 0) {
                    rightArm1.setPower(0);
                    leftArm1.setPower(0);
                }

                if (direction > 0) {
                    leftArm1.setPower(-1);
                    rightArm1.setPower(1);
                }
                if (direction < 0) {
                    leftArm1.setPower(1);
                    rightArm1.setPower(-1);
                }
                if (wristDirection == 0){
                    wristMotor.setPower(0);
                }
                if (wristDirection > 0) {
                    wristMotor.setPower(.3);
                }
                if (wristDirection < 0) {
                    wristMotor.setPower(-.3);
                }




            telemetry.addData( "left claw position", leftClaw.getPosition());
            telemetry.addData( "right claw position", rightClaw.getPosition());

            telemetry.update();


            double wheelSpeed = 0.5; //0.0 - 1


            leftFront.setPower(-(y + x + rx) * wheelSpeed);
            leftBack.setPower(-(y - x + rx) * wheelSpeed);
            rightFront.setPower((y - x - rx) * wheelSpeed);
            rightBack.setPower((y + x - rx) * wheelSpeed);

            if (gamepad1.a) {
                aButtonUpdate = true;
            }
            else {
                if (aButtonUpdate == true) {
                    gripClaw = !gripClaw;
                    aButtonUpdate = false;
                }
            }


            
            if (gripClaw)
            {

//                telemetry.addData("right servo position", rightClaw.getPosition());
//                telemetry.addData("left servo position", leftClaw.getPosition());

                telemetry.update();
                rightClaw.setPosition(0);
                leftClaw.setPosition(0);
            }
            else
            {
                telemetry.addData("right servo position", rightClaw.getPosition());
//                telemetry.addData("left servo position", leftClaw.getPosition());

                telemetry.update();

                rightClaw.setPosition(0.1);
                leftClaw.setPosition(0.1);

            }


            if (gamepad1.left_stick_button) {
                wheelSpeed = 1;
            }
            else {
                wheelSpeed = 0.5;
            }
            if (gamepad1.right_stick_button) {
                wheelSpeed = 0.25;
            }
            else {
                wheelSpeed = 0.5;
            }
            telemetry.addData("Reset", gamepad1.y);

            if (gamepad1.y) {
                rightArm1.setTargetPosition(armRightStart);
                leftArm1.setTargetPosition(armLeftStart);
                isReset = true;

            }

            if(gamepad2.right_trigger - gamepad2.left_trigger != 0)
            {
                slide.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            }

            if (gamepad2.a)
            {
                airplane.setPosition(0);
            }


        }
    }
}
