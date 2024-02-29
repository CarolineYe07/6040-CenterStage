package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class AutoTimeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        robot.init();

        Object zone = PropVisionProcessor.Selected.LEFT;

        waitForStart();

        if (zone == PropVisionProcessor.Selected.MIDDLE) {
            // drive forward
            robot.setDrivePower(-0.5, -0.5, -0.5, -0.5);
            sleep(1250);
            robot.setDrivePower(0, 0, 0,0);

            robot.setDrivePower(0.5, 0.5, 0.5,0.5);
            sleep(1000);
            robot.setDrivePower(0,0,0,0);

            robot.setDrivePower(-0.5, 0.5, 0.5, -0.5);
            sleep(3000);
            robot.setDrivePower(0,0,0,0);

        } else if (zone == PropVisionProcessor.Selected.LEFT) {

            telemetry.addLine("Driving forward");
            telemetry.update();
            robot.setDrivePower(-0.5, -0.5, -0.5, -0.5);
            sleep(1000);
            robot.setDrivePower(0, 0, 0,0);

            telemetry.addLine("Turn 90 degrees");
            telemetry.update();
            // pretty much 90 degrees?
            robot.setDrivePower(-0.5, -0.5, 0.5, 0.5);
            sleep(950);

            telemetry.addLine("Push pixel backwards to line");
            telemetry.update();
            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(100);


            robot.setDrivePower(-0.5, -0.5, -0.5,-0.5);
            sleep(400);


            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(200);


            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
            sleep(1000);

            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(3000);

        }
    }
}
