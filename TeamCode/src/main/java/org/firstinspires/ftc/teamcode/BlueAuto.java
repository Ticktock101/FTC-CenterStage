package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Timer;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Blue Auto")
//@Disabled
public class BlueAuto extends LinearOpMode {

    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    Servo rightClaw;
    Servo leftClaw;

    DcMotor rightArm1;
    DcMotor leftArm1;
    DcMotor wristMotor;

    private ElapsedTime runtime = new ElapsedTime();

    double targetArmHeight = 0;

    Servo pixelPush;

    private int lfPos; private int rfPos; private int lrPos; private int rrPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 62.7863478; // empirically measured
    private double clicksPerDeg = 1.5555555555556; // empirically measured

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "BluePyramidv2.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private int position = 1;
    private int count = 0;

    private boolean isFound = false;

    private int countFalse = 0;
    private int laPos; private int raPos;

    private RevColorSensorV3 rightColor;
    RevColorSensorV3 leftColor;


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        wristMotor = hardwareMap.get(DcMotor.class, "wrist");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setZeroPosition();

        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        pixelPush = hardwareMap.get(Servo.class, "pixel");
        // pixelPush.setDirection(Servo.Direction.REVERSE);

        rightColor = hardwareMap.get(RevColorSensorV3.class, "rightColor");
        leftColor = hardwareMap.get(RevColorSensorV3.class, "leftColor");

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm1.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the drive motor run modes:
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while(opModeInInit()) {
            pixelPush.setPosition(0);
            telemetry.addData("l:", leftColor.getDistance(DistanceUnit.CM));
            telemetry.addData("r:", rightColor.getDistance(DistanceUnit.CM));
            telemetry.update();

            rightClaw.setPosition(0);
            leftClaw.setPosition(0);
        }

        waitForStart();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(650);
        rightFront.setTargetPosition(650);
        leftBack.setTargetPosition(650);
        rightBack.setTargetPosition(650);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.3);
        rightFront.setPower(.3);
        leftBack.setPower(.3);
        rightBack.setPower(.3);

        while(leftFront.getCurrentPosition() < 600) {};

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(850);
        rightFront.setTargetPosition(850);
        leftBack.setTargetPosition(850);
        rightBack.setTargetPosition(850);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(.3);
        rightFront.setPower(.3);
        leftBack.setPower(.3);
        rightBack.setPower(.3);

        boolean detected = false;

        while(!detected) {
            if (leftColor.getDistance(DistanceUnit.CM) < 5) {
                //left
                position = 1;
                detected = true;
            } else if (rightColor.getDistance(DistanceUnit.CM) < 6) {
                //right
                position = 2;
                detected = true;
            } else if (leftFront.getCurrentPosition() > 850) {
                //center
                position = 0;
                detected = true;
            }
        }
        switch(position) {
            case 0:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(.3);
                rightFront.setPower(-.3);
                leftBack.setPower(-.3);
                rightBack.setPower(.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 100) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                moveServo();

                moveForward(-2, 0.5);


                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(-.3);
                rightFront.setPower(-.3);
                leftBack.setPower(-.3);
                rightBack.setPower(-.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 200) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(-.3);
                rightFront.setPower(.3);
                leftBack.setPower(.3);
                rightBack.setPower(-.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 1500) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(-.3);
                rightFront.setPower(.3);
                leftBack.setPower(-.3);
                rightBack.setPower(.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 900) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                moveArm(500, 0.5);

                wristMotor.setPower(1);
                sleep(300);
                wristMotor.setPower(0);

                moveArm(25, 0.5);

                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

                moveForward(10, 0.5);

                moveLeft2(50);

                moveForward(5, 0.5);
                ElapsedTime timer = new ElapsedTime();

                while(timer.time() < 2)
                {
                    rightClaw.setPosition(0.2);
                    leftClaw.setPosition(0.2);
                }



                moveForward(-5, 0.5);


                break;
            case 1:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(-.3);
                rightFront.setPower(.3);
                leftBack.setPower(-.3);
                rightBack.setPower(.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 850) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);


                moveForward(-1, 0.5);



                moveServo();
                moveForward(-1, 0.5);


                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);


                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(-.3);
                rightFront.setPower(.3);
                leftBack.setPower(.3);
                rightBack.setPower(-.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 600) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

                moveForward(18, 0.5);

                moveArm(500, 0.5);

                wristMotor.setPower(1);
                sleep(290);
                wristMotor.setPower(0);

                moveArm(25, 0.5);

                moveRight2(125);

                moveForward(20, 0.5);

                ElapsedTime timer3 = new ElapsedTime();

                while(timer3.time() < 2)
                {
                    rightClaw.setPosition(0.2);
                    leftClaw.setPosition(0.2);
                }

                moveForward(-5, 0.5);


                break;
            case 2:
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(.3);
                rightFront.setPower(-.3);
                leftBack.setPower(.3);
                rightBack.setPower(-.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 950) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                // moveRight2(100);

                moveForward(2, 0.5);

                moveLeft2(100);

                moveServo();

                moveForward(-25, 0.5);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

                leftFront.setPower(.3);
                rightFront.setPower(-.3);
                leftBack.setPower(.3);
                rightBack.setPower(-.3);

                while(Math.abs(leftFront.getCurrentPosition()) < 1825) {};

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

                moveArm(500, 0.5);

                wristMotor.setPower(1);
                sleep(300);
                wristMotor.setPower(0);

                moveArm(25, 0.5);

                moveRight2(175);

                moveForward(15, 0.5);

                ElapsedTime timer2 = new ElapsedTime();

                while(timer2.time() < 2)
                {
                    rightClaw.setPosition(0.2);
                    leftClaw.setPosition(0.2);
                }



                moveForward(-5, 0.5);

                break;
        }


    }   // end runOpMode()

    public void moveLeft2(int distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        leftFront.setPower(-.3);
        rightFront.setPower(.3);
        leftBack.setPower(.3);
        rightBack.setPower(-.3);

        while(Math.abs(leftFront.getCurrentPosition()) < distance) {};

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


    }

    public void moveRight2(int distance)
    {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        leftFront.setPower(.3);
        rightFront.setPower(-.3);
        leftBack.setPower(-.3);
        rightBack.setPower(.3);

        while(Math.abs(leftFront.getCurrentPosition()) < distance) {};

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }


    public boolean moveServo()
    {
        pixelPush.setPosition(1);

        runtime.reset();
        while(runtime.seconds() < 2);

        return true;
    }

    private void moveArm(int whatAngle, double speed){
        // fetch motor position
        raPos = rightArm1.getCurrentPosition();
        laPos = leftArm1.getCurrentPosition();

        // calculate new targets
        raPos += whatAngle * clicksPerDeg;
        laPos += whatAngle * clicksPerDeg;

        // move arm to desired position
        rightArm1.setTargetPosition(raPos);
        leftArm1.setTargetPosition(laPos);
        rightArm1.setPower(speed);
        leftArm1.setPower(speed);

        rightArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for move to complete
        while (rightArm1.isBusy() && leftArm1.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Arm Moved");
            telemetry.addData("Target", "%7d :%7d", raPos, laPos);
            telemetry.addData("Actual", "%7d :%7d", rightArm1.getCurrentPosition(), leftArm1.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        rightArm1.setPower(0);
        leftArm1.setPower(0);
    }

    private void moveWrist(int time){

        wristMotor.setPower(-0.5);
        sleep(time);
        wristMotor.setPower(0);
    }

    private boolean moveForward(int howMuch, double speed) {

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // howMuch is in inches. A negative howMuch moves backward.

        setZeroPosition();

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lrPos = leftBack.getCurrentPosition();
        rrPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lrPos);
        rightBack.setTargetPosition(rrPos);



        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Foward");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        return true;
    }

    private void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        setZeroPosition();

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lrPos = leftBack.getCurrentPosition();
        rrPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        leftFront.setTargetPosition(lfPos);
        rightFront.setTargetPosition(rfPos);
        leftBack.setTargetPosition(lrPos);
        rightBack.setTargetPosition(rrPos);
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        // wait for move to complete
        while (leftFront.isBusy() && rightFront.isBusy() &&
                leftBack.isBusy() && rightBack.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

    private void turnClockwise(int whatAngle, double speed) {
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.

        setZeroPosition();

        // fetch motor positions
        lfPos = leftFront.getCurrentPosition();
        rfPos = rightFront.getCurrentPosition();
        lrPos = leftBack.getCurrentPosition();
        rrPos = rightBack.getCurrentPosition();

        // calculate new targets
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // // move robot to new position
        // leftFront.setTargetPosition(lfPos);
        // rightFront.setTargetPosition(rfPos);
        // leftBack.setTargetPosition(lrPos);
        // rightBack.setTargetPosition(rrPos);

        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);

        // wait for move to complete
        while (leftFront.getCurrentPosition() < whatAngle) {

            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(), leftBack.getCurrentPosition(),
                    rightBack.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void setZeroPosition()
    {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}   // end class
