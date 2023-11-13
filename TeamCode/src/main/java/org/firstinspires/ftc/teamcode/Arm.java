package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="Arm Test")
public class Arm extends LinearOpMode {

    DcMotor rightArm1;
    DcMotor leftArm1;

    @Override
    public void runOpMode() throws InterruptedException {

        rightArm1 = hardwareMap.get(DcMotor.class, "rightArm");
        leftArm1 = hardwareMap.get(DcMotor.class, "leftArm");
        rightArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            telemetry.addData("lt power", lt);
            telemetry.update();

            if (rt - lt == 0)
            {
                int rightPosition = rightArm1.getCurrentPosition();
                int leftPosition = leftArm1.getCurrentPosition();

                telemetry.addData("# right position value", rightPosition);
                telemetry.addData("# left position value", leftPosition);



                telemetry.update();

                rightArm1.setPower(-1);
                leftArm1.setPower(1);


            }

            rightArm1.setPower(rt - lt);
            leftArm1.setPower(-(rt - lt));



        }

    }
    private static double calculateExponentialPower(double a, double b, int position) {
        return a * Math.exp(b * position + 115);
    }
}
