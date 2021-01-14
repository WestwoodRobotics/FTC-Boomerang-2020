package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Autonomous(name="functions")
public class BoomerangAutonFunctions{
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;

    leftBackMotor  = hardwareMap.get(DcMotor.class, "left_back_drive");
    rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_drive");
    leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_drive");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_drive");

    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    static void forward(double distance) {
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(encodervalue>distance or encodervalue<distance){

        }
        leftBackMotor.setDirection(forward);
        rightBackMotor.setDirection(forward);
        leftFrontMotor.setDirection(forward);
        rightFrontMotor.setDirection(forward);
    }
}