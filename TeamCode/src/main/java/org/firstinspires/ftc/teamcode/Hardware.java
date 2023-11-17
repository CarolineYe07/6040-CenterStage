package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class Hardware {

    private LinearOpMode myOpMode = null;

    private DcMotor leftFront, leftBack, rightFront, rightBack = null;
    private DcMotor encoderLeft, encoderRight, encoderFront = null;
    private CRServo scuffIntake = null;
    private Servo drone = null;

    public DcMotor testMotor;
    public CRServo testServo;

    private ElapsedTime runtime;

    // public IMU imu;


    // Constant values here

    public Hardware (LinearOpMode opMode) { myOpMode = opMode; }

    public void init() {

        runtime = new ElapsedTime();

        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "lf");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "lb");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rf");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "rb");

        drone = myOpMode.hardwareMap.get(Servo.class, "drone");
        drone.setDirection(Servo.Direction.REVERSE);

        scuffIntake = myOpMode.hardwareMap.get(CRServo.class, "scuffIntake");
        scuffIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        testMotor = myOpMode.hardwareMap.get(DcMotor.class, "hang");
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        testServo = myOpMode.hardwareMap.get(CRServo.class, "test");


        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set to which ports the encoders on the odo pods are plugged into
        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderFront = rightFront;

        // IMU stuff
        // Retrieve the IMU from the hardware map



        myOpMode.telemetry.addLine("Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // odo constants
    final static double L = 38.48;    // distance between encoders 1 & 2 in cm
    final static double B = 0.0;    // distance between the midpoints of encoders 1 & 2 and encoder 3
    final static double R = 3.5 / 2;    // radius of odo pod
    final static double N = 8192;    // ticks per rev
    final static double cm_per_tick = 2 * Math.PI * R / N;

    // keep track of pos in last loop vs. pos in this loop
    public int currentPosRight = 0;
    public int currentPosLeft = 0;
    public int currentPosFront = 0;

    public int oldPosRight = 0;
    public int oldPosLeft = 0;
    public int oldPosFront = 0;

    // tuple (x, y, h) where x is x-coord, y is y-coord, h is heading (angle)
    public double START_POS[] = new double[]{0, 0, 0};
    public double pos[] = START_POS;


    public void setDrivePower(double lf, double lb, double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    public void driveForwardByTime(double seconds, double power) {
        while (myOpMode.opModeIsActive() && runtime.seconds() < seconds) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
    }

    public void setDroneShooter(double position) {
        drone.setPosition(position);
    }

    public void scuffIntakePower(double power) {
        scuffIntake.setPower(power);
    }

    public void three_wheel_odo() {
        oldPosRight = currentPosRight;
        oldPosLeft = currentPosLeft;
        currentPosFront = oldPosFront;

        currentPosLeft = encoderLeft.getCurrentPosition();
        currentPosRight = -encoderRight.getCurrentPosition();
        currentPosFront = -encoderFront.getCurrentPosition();

        int dn1 = currentPosLeft - oldPosLeft;
        int dn2 = currentPosRight - oldPosRight;
        int dn3 = currentPosFront - oldPosFront;

        double dtheta = cm_per_tick * (dn2 - dn1) / L;
        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);

        double theta = pos[2] + dtheta / 2;
        pos[0] += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos[1] += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos[2] += dtheta;

        myOpMode.telemetry.addData("hi", "hello");
    }

    public void two_wheel_odo() {

    }
}
