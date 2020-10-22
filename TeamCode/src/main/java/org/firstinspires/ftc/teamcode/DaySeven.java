package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Basic: Linear OpMode", group="Linear Opmode")
public class DaySeven extends LinearOpMode {

    //making runtime variable
    private ElapsedTime runtime = new ElapsedTime();

    //initialising motors
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;

    public void runOpMode() {
        //defining motors on hardware map
        leftFront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBack = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBack = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //set motor direction
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
        }
        public void frontBack ( double Time, double Speed){
            runtime.reset();
            while (runtime.seconds() < time && opModeIsActive()) {
                leftFront.setPower(Speed);
                leftBack.setPower(Speed);
                rightBack.setPower(Speed);
                rightFront.setPower(Speed);
            }
        }
        public void centerTurn ( double Time, double Speed){
            runtime.reset();
            while (runtime.seconds() < time && opModeIsActive()) {
                leftFront.setPower(Speed);
                leftBack.setPower(Speed);
                rightBack.setPower(-Speed);
                rightFront.setPower(-Speed);
            }
        }
        public void leftRight ( double Time, double Speed){
            runtime.reset();
            while (runtime.seconds() < time && opModeIsActive()) {
                leftFront.setPower(Speed);
                leftBack.setPower(-Speed);
                rightBack.setPower(Speed);
                rightFront.setPower(-Speed);
            }
        }
        public void brake(){
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }



    }




}
