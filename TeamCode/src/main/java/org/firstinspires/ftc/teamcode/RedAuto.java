package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


// import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous (name = "RedAuto")
public class RedAuto extends LinearOpMode {
    //declaring stuff
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    DcMotor leftArm1;
    DcMotor rightArm1;

    DcMotor wristMotor;

    Servo rightClaw;
    Servo leftClaw;

    // drive motor position variables
    private int lfPos; private int rfPos; private int lrPos; private int rrPos;
    //subsystem position variables
    private int laPos; private int raPos; private int wmPos;

    // operational constants
    private double fast = 0.5; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.3; // medium speed
    private double slow = 0.1; // slow speed
    private double clicksPerInch = 50.238; // empirically measured
    private double clicksPerDeg = 13.55; // empirically measured
    private double lineThreshold = 0.7; // floor should be below this value, line above
    private double redThreshold = 1.9; // red should be below this value, blue above

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        // Declare Stuff
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        wristMotor = hardwareMap.get(DcMotor.class, "wrist");
        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightClaw = hardwareMap.get(Servo.class, "rightServo");
        leftClaw = hardwareMap.get(Servo.class, "leftServo");

        // Reverse stuff
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftArm1.setDirection(DcMotor.Direction.FORWARD);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Encoder Stuff
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the drive motor run modes:
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        turnClockwise(45, medium);
        moveArm(45, medium);
        moveWrist(20, medium);
    }
    //moves th arm dependng on what angle wanted and what speed
    /** TODO: Test, fix angle
     * might need to make new variable clicksPerDeg for arm motor **/
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

    /** TODO: Test, fix angle
     * might need to make new variable clicksPerDeg for wrist motor **/
    private void moveWrist(int whatAngle, double speed) {
        // fetch motor positions
        wristMotor.getCurrentPosition();

        // calculate new targets
        wmPos += whatAngle * clicksPerDeg;

        // move arm to desired position
        wristMotor.setTargetPosition(wmPos);
        wristMotor.setPower(speed);

        // wait for move to complete
        while (wristMotor.isBusy()){

            // Display it for the driver.
            telemetry.addLine("Wrist Moved");
            telemetry.addData("Target", "%7d :%7d", wmPos);
            telemetry.addData("Actual", "%7d :%7d", wristMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        wristMotor.setPower(0);
    }

    /** TODO: Test, add things?
     * might need to reverse or put in negative for position variable **/
    private void clawMov(double position) {
        rightClaw.setPosition(position);
        leftClaw.setPosition(position);
    }

    private void moveForward(int howMuch, double speed) {

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
}