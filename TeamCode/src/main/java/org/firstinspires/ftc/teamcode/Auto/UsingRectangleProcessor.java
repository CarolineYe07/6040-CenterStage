package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.RectangleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class UsingRectangleProcessor extends OpMode {

    private RectangleProcessor rectangleProcessor;
    private VisionPortal visionPortal;


    @Override
    public void init() {
        rectangleProcessor = new RectangleProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), rectangleProcessor);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {

    }
}
