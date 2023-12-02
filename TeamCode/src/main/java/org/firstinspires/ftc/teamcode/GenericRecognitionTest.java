package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Control Hub ADB Terminal Command for Reference
// adb.exe connect 192.168.43.1:5555

@Autonomous(name = "Rec test Thread")

public class GenericRecognitionTest extends LinearOpMode {

    private TestDetector rf = null;
    private String result = "";

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            // Initialize and start the detection thread
            try {
                rf = new TestDetector(this.hardwareMap, this, telemetry);
                Thread detectThread = new Thread(rf);
                detectThread.start();
                telemetry.update();

            } catch (InterruptedException e) {
                telemetry.addData("Error", "Thread interrupted");
                telemetry.update();
                Thread.currentThread().interrupt(); // Restore interrupted status
                sleep(5000);
                return;
            } catch (Exception ex) {
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                telemetry.update();
                sleep(5000);
            }



            // Wait for the game to start (driver presses PLAY)
            telemetry.update();
            waitForStart();

            // Continue with your autonomous code after the start button is pressed
          rf.stopDetection();
            result = rf.getResult();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Result", result);
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        } finally {
            if (rf != null) {
                rf.stopDetection();
            }
        }
    }
}
