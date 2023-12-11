package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.PropVisionProcessor;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PropVisionAuto extends LinearOpMode {

    private PropVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    private double SPEED;
    private int FORWARD_TIME;
    private int STRAFE_TIME;
    private int TURN_TIME;

    @Override
    public void runOpMode() throws InterruptedException {

        visionProcessor = new PropVisionProcessor();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
        telemetry.addLine("Vision Portal Initialized");
        telemetry.update();

        SPEED = 0.75;
        FORWARD_TIME = 1500;
        STRAFE_TIME = 1000;
        TURN_TIME = 500;


        Hardware robot = new Hardware(this);
        robot.init();
        telemetry.addLine("Robot Initialized");
        telemetry.update();

        Servo lClamp = hardwareMap.get(Servo.class, "lClamp");



        waitForStart();

        sleep(1000);
        Object zone = visionProcessor.getSelection();

        while (opModeIsActive()) {

            telemetry.addData("Selection: ", visionProcessor.getSelection());
            telemetry.addData("Right Rect Sat: ", visionProcessor.satRectRight);
            telemetry.addData("Middle Rect Sat: ", visionProcessor.satRectMiddle);
            telemetry.update();


            if (zone == PropVisionProcessor.Selected.LEFT) {
                // move forward (strafe because robot starts turned)
                robot.setDrivePower(-SPEED, SPEED, SPEED, -SPEED);
                sleep(FORWARD_TIME);
                robot.setDrivePower(0, 0, 0, 0);

                // turn to the left
                robot.setDrivePower(SPEED, 0, 0, SPEED);
                sleep(TURN_TIME * 2);
                robot.setDrivePower(0 ,0, 0,0);

                // release pixel
                lClamp.setPosition(1);


            } else if (zone == PropVisionProcessor.Selected.MIDDLE) {
                // move forward (strafe because robot starts turned)
                robot.setDrivePower(-SPEED, SPEED, SPEED, -SPEED);
                sleep(FORWARD_TIME);
                robot.setDrivePower(0, 0, 0, 0);

                // turn
                robot.setDrivePower(SPEED, 0, 0, SPEED);
                sleep(TURN_TIME);


                // release pixel
                lClamp.setPosition(1);


                // drive back
                robot.setDrivePower(-SPEED, -SPEED, -SPEED, -SPEED);
                sleep(FORWARD_TIME);
                robot.setDrivePower(0, 0, 0, 0);

                // strafe to backdrop
                robot.setDrivePower(SPEED, -SPEED, -SPEED, SPEED);
                sleep(5000);
                robot.setDrivePower(0, 0, 0, 0);


            } else {
                // move forward (strafe because robot starts turned)
                robot.setDrivePower(-SPEED, SPEED, SPEED, -SPEED);
                sleep(FORWARD_TIME + 500);
                robot.setDrivePower(0, 0, 0, 0);

                // release pixel
                lClamp.setPosition(1);
            }
        }
    }
}
