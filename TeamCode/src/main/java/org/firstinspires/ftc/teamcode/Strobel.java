package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;

@Autonomous(name = "Strobel")
public class Strobel extends LinearOpMode {

    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    Servo rightClaw;
    Servo leftClaw;

    DcMotor rightArm1;
    DcMotor leftArm1;
    DcMotor wristMotor;

    double targetArmHeight = 0;

//    BHI260IMU imu;
//    IMU.Parameters myIMUparameters;


    private void strobelEvent(String name) {
            if (name == "auto") {
                driveRotate(1,.5);
                sleep(500);
                driveThrottle(1,1);
                sleep(1000);
                gripClaw(false);
            }
            if (name == "ready") {
                moveArm(1,.5);
                moveArm(-1,.5);
                gripClaw(false);
                gripClaw(true);
            }


    }

    boolean autonomous = false;

    private class StrobelAnimationSequence {

        private String[] eventSequence = new String[]{
                "auto"
        };
        StrobelAnimationSequence(String[] eventSequence) {
            this.eventSequence = eventSequence;
        }


        public void play() {
            for (String anim : eventSequence) {
                strobelEvent(anim);
            }
        }
    }

    private void driveRotate(double direction, double time) {
        double wheelSpeed = 0.5; //0.0 - 1.0

        double y = -direction;
        double x = direction;

        double rx = 0;

        leftFront.setPower(-(y + x + rx) * wheelSpeed);
        leftBack.setPower(-(y - x + rx) * wheelSpeed);
        rightFront.setPower((y - x - rx) * wheelSpeed);
        rightBack.setPower((y + x - rx) * wheelSpeed);

        while (time >= 0) {
            time -= .1;
            sleep(100);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void driveThrottle(double strength, double time) {
        double wheelSpeed = 0.5; //0.0 - 1
        double rx = strength;


        leftFront.setPower(-(0 + 0 + rx) * wheelSpeed);
        leftBack.setPower(-(0 - 0 + rx) * wheelSpeed);
        rightFront.setPower((0 - 0 - rx) * wheelSpeed);
        rightBack.setPower((0 + 0 - rx) * wheelSpeed);

        while (time >= 0) {
            time -= .1;
            sleep(100);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }

    private void moveArm(double direction, double time) {

        if (direction > 0) {
            leftArm1.setPower(-1);
            rightArm1.setPower(1);
        }
        if (direction < 0) {
            leftArm1.setPower(1);
            rightArm1.setPower(-1);
        }
        if (direction == 0) {
            leftArm1.setPower(0);
            rightArm1.setPower(0);
        }

        while (time >= 0) {
            time -= .1;
            sleep(100);
        }

        leftArm1.setPower(0);
        rightArm1.setPower(0);
    }

    private void bendWrist(double direction, double time) {

        if (direction > 0) {
            wristMotor.setPower(1);;
        }
        if (direction < 0) {
            wristMotor.setPower(-1);
        }
        if (direction == 0) {
            wristMotor.setPower(0);
        }

        while (time >= 0) {
            time -= .1;
            sleep(100);
        }

        wristMotor.setPower(0);
    }

    private void gripClaw(boolean closed) {
        if (closed)
        {
            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
        }
        else
        {
            rightClaw.setPosition(0.1);
            leftClaw.setPosition(0.1);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        wristMotor = hardwareMap.get(DcMotor.class, "wrist");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");
        rightClaw.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        String[] sequence = new String[]{"auto"};

        StrobelAnimationSequence anim = new StrobelAnimationSequence(sequence);
        anim.play();

        stop();

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
                direction = -0.25 * (rightArm1.getCurrentPosition() - armRightStart);
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


        }
    }
}
