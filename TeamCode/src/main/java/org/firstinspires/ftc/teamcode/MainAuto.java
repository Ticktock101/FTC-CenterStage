package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous (name = "Main Auto")
public class MainAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    private double x;
    private double y;

    private double DcMotorPower = 0.5;

    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    Servo rightClaw;
    Servo leftClaw;


    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();

        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");


        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        waitForStart();

        while (opModeIsActive())
        {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }

            while (x != 0)
            {
                if (x < 0)
                {

                }
                else if (x > 0)
                {

                }
            }

            rightFront.setPower(DcMotorPower);
            leftFront.setPower(DcMotorPower);
            rightBack.setPower(DcMotorPower);
            leftBack.setPower(DcMotorPower);

            sleep(500);

            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            leftBack.setPower(0);
        }

    }

    private void initTfod()
    {
        tfod = new TfodProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.addProcessor(tfod);
        visionPortal = builder.build();


    }
}
