package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class PropVisionAuto extends LinearOpMode {

    private PropVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        visionProcessor = new PropVisionProcessor();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        telemetry.addLine("Vision Portal Initialized");
        telemetry.update();


        Hardware robot = new Hardware(this);
        robot.init();
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        telemetry.addData("Identified", visionProcessor.getSelection());

        waitForStart();

        visionPortal.stopStreaming();



        if (visionProcessor.getSelection() == PropVisionProcessor.Selected.RIGHT) {
            // move to that spike pos
            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(500);
            robot.setDrivePower(0.5, 0.5, 0, 0);
            sleep(100);
            robot.setDrivePower(-0.5, -0.5, -0.5, -0.5);
            sleep(100);
            robot.setDrivePower(0.5, 0.5, 0, 0);
            sleep(100);


        } else if (visionProcessor.getSelection() == PropVisionProcessor.Selected.MIDDLE) {
            // move to that spike pos


        } else {
            // pick the last spike pos even if nothing was seen

        }


    }

    /**
    @Override
    public void init() {
        visionProcessor = new PropVisionProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

        telemetry.addData("Identified", visionProcessor.getSelection());

    **/

}
