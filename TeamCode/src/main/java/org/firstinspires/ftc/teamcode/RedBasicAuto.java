package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Red Basic Auto", group = "Concept")
//@Disabled
public class RedBasicAuto extends LinearOpMode {

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

    Servo pixelPush;

    private int lfPos; private int rfPos; private int lrPos; private int rrPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 50.238; // empirically measured
    private double clicksPerDeg = 13.55; // empirically measured
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedPyramid2.tflite";
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

    private int position = 2;
    private int count = 0;

    private boolean isFound = false;

    private int countFalse = 0;
    private int laPos; private int raPos;


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



//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        setZeroPosition();



        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
//        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");
        rightClaw.setDirection(Servo.Direction.REVERSE);

        pixelPush = hardwareMap.get(Servo.class, "pixel");
        pixelPush.setDirection(Servo.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm1.setDirection(DcMotor.Direction.FORWARD);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the drive motor run modes:
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


//                telemetryTfod();

//
                ElapsedTime myLoopTimer = new ElapsedTime();

                while (count == 0)
                {

                    while (myLoopTimer.time() < 8 && !telemetryTfod())
                    {
                        telemetryTfod();
                    }

                    if (position == 0)
                    {
//                        moveForward(10, medium);
//                        turnClockwise(-28, 0.5);
//                        moveForward(13, medium);
//
//                        pixelPush.setPosition(0);

                        moveForward(22, medium);
                        turnClockwise(-38, 0.5);
//                        moveForward(7, medium);

//                        pixelPush.setPosition(0);
                        ElapsedTime newTimer = new ElapsedTime();

                        while (newTimer.time() < 3)
                        {
                            moveServo();

                        }

                        moveForward(-5, medium);

//
//
//
//                        turnClockwise(-22, 0.5);
//                        moveForward(15, medium);
//
//
//                        rightArm1.setPower(0.5);
//                        leftArm1.setPower(0.5);
//                        sleep(160);
//
//                        rightArm1.setPower(0);
//                        leftArm1.setPower(0);
//
//                        wristMotor.setPower(0.5);
//                        sleep(25);
//                        wristMotor.setPower(0);
//
//
//
//
////                        moveArm(20, 0.5);
//
////                        turnClockwise(5, 0.5);
//                        moveForward(4, medium);
                    }

                    else if (position == 1)
                    {
                        moveForward(26, medium);
//                        pixelPush.setPosition(0);

                        ElapsedTime newTimer = new ElapsedTime();

                        while (newTimer.time() < 3)
                        {
                            moveServo();

                        }

                        moveForward(-5, medium);
                    }
                    else
                    {
//                        moveForward(10, medium);
//                        turnClockwise(13, 0.5);
//                        moveForward(7, medium);
//
//                        pixelPush.setPosition(0);

                        moveForward(12, medium);
                        turnClockwise(10, 0.5);
                        moveForward(4, medium);

                        ElapsedTime newTimer = new ElapsedTime();

                        while (newTimer.time() < 3)
                        {
                            moveServo();

                        }

                        moveForward(-5, medium);

//                        moveArm(10, medium);
                    }


                    count++;
                }
//
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//
//                // Save CPU resources; can resume streaming when needed.
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                if (position == 0)
//                {
//                    moveForward(10, medium);
//                    telemetry.addData("position", position);
//                }

//                turnRight();

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    public void moveServo()
    {
        pixelPush.setPosition(0);

        sleep(400);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private boolean telemetryTfod() {
//        sleep(200);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("X position : ", x);
            telemetry.addData("Y position", y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x < 100)
            {
                position = 0;
                return true;
            }
            else if (x > 300)
            {
                position = 1;
                return true;
            }


        }   // end for() loop

//        if(currentRecognitions.size() == 0)
//        {
//            position = 2;
//            return true;
//        }


        return false;
    }   // end method telemetryTfod()

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

    private void moveForward(int howMuch, double speed) {
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
    private void moveToLine(int howMuch, double speed) {
        // howMuch is in inches. The robot will stop if the line is found before
        // this distance is reached. A negative howMuch moves left, positive moves right.

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
