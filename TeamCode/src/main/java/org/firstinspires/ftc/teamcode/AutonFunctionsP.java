package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//time based auton assuming the robot moves 5 inches every 7 seconds and turns 15 degrees/second
//conveyor belt moves each ring 1 inch per second along the conveyor belt
public class AutonFunctionsP {
    ElapsedTime time = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor conveyorBelt = null;
    private DcMotor shooter = null;
    public int ringLocation = -1;
    private TFObjectDetector tfod;
    private int encoderValPerInch = 1;
    public AutonFunctionsP(DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive,
                           DcMotor rightBackDrive, TFObjectDetector tfod, DcMotor conveyorBelt,
                           DcMotor shooter){
        this.leftBackDrive = leftBackDrive;
        this.leftFrontDrive = leftFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.rightFrontDrive = rightFrontDrive;
        this.tfod = tfod;
        this.conveyorBelt = conveyorBelt;
        this.shooter = shooter;

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotateCenter(double degrees, String direction){
        double seconds = degrees/15;
        time.reset();
        if(direction.equals("clockwise")) {
            leftBackDrive.setPower(1);
            leftFrontDrive.setPower(1);
            rightBackDrive.setPower(-1);
            rightFrontDrive.setPower(-1);
        }
        else{
            leftBackDrive.setPower(-1);
            leftFrontDrive.setPower(-1);
            rightBackDrive.setPower(1);
            rightFrontDrive.setPower(1);
        }
        while(time.seconds() < seconds){
        }
        stop();
    }

    public void rotateFrontCenter(double degrees, String direction){
        double seconds = degrees/15;
        time.reset();
        if(direction.equals("clockwise")) {
            leftBackDrive.setPower(1);
            rightBackDrive.setPower(-1);
        }
        else{
            leftBackDrive.setPower(-1);
            rightBackDrive.setPower(1);
        }
        while(time.seconds() < seconds){
        }
        stop();
    }

    public void right(int inches){
        int distance = inches * encoderValPerInch;

        leftBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontDrive.setTargetPosition(distance);
        rightFrontDrive.setTargetPosition(-distance);
        leftBackDrive.setTargetPosition(-distance);
        rightBackDrive.setTargetPosition(distance);

        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(1);

        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()
                && rightBackDrive.isBusy()){}

        stop();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void left(int inches){
        int distance = inches * encoderValPerInch;

        leftBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontDrive.setTargetPosition(-distance);
        rightFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(-distance);

        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(-1);

        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()
                && rightBackDrive.isBusy()){}

        stop();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backward(int inches){
        int distance = inches * encoderValPerInch;

        leftBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontDrive.setTargetPosition(-distance);
        rightFrontDrive.setTargetPosition(-distance);
        leftBackDrive.setTargetPosition(-distance);
        rightBackDrive.setTargetPosition(-distance);

        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(-1);
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(-1);

        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()
                && rightBackDrive.isBusy()){}

        stop();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forward(int inches){
        int distance = inches * encoderValPerInch;

        leftBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);

        leftFrontDrive.setTargetPosition(distance);
        rightFrontDrive.setTargetPosition(distance);
        leftBackDrive.setTargetPosition(distance);
        rightBackDrive.setTargetPosition(distance);

        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(1);

        while(leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()
        && rightBackDrive.isBusy()){}

        stop();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(){
        time.reset();
        shooter.setPower(1);
        conveyorBelt.setPower(1);
        while(time.seconds() < 5){
        }
        shooter.setPower(0);
        conveyorBelt.setPower(0);
    }
    public void pause(double seconds){
        time.reset();
        while(time.seconds() < seconds){
        }
    }

     public void senseRings(){
         List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
         if (updatedRecognitions != null) {
             for (Recognition recognition : updatedRecognitions) {
             }
         }
         else{
             ringLocation = 0;
         }
     }
    public void stop(){
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}
