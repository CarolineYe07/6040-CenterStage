package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

@Autonomous
public class BasicParkAutoBlue extends LinearOpMode {


    // private ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware robot = new Hardware(this);
        robot.init();

        Servo leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        Servo rightIntake = hardwareMap.get(Servo.class, "rightIntake");

        // runtime = new ElapsedTime();

        waitForStart();

        // runtime.reset();

        while (opModeIsActive()) {
            robot.setDrivePower(0.5, 0.5, 0.5, 0.5);
            sleep(50);

            robot.setDrivePower(-0.5, 0.5, 0.5, -0.5);
            sleep(2000);

            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
            sleep(300);

            break;
        }

        /**
        while (opModeIsActive() && runtime.seconds() < 1) {
            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        }
        robot.setDrivePower(0, 0, 0,0);


        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 5) {
            robot.setDrivePower(0.5, -0.5, -0.5, 0.5);
        }
        robot.setDrivePower(0, 0, 0,0);
        **/

    }
}
