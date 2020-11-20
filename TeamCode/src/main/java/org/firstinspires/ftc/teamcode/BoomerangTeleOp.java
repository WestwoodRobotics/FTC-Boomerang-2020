package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class BoomerangTeleOp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private boolean slowMode = false;
    private DcMotor roller = null;
    private boolean isIntakeRunning = false;
    private DcMotor conveyorBeltL = null;
    private DcMotor conveyorBeltR = null;
    private DcMotor shooterMotor = null;
    private CRServoImpl wobbleArmServo = null;
    private CRServoImpl wobbleClawServo = null;
    //Power variables
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double maxPower;
    double turn;
    double conveyorPower = 0;
    double shooterPower = 0;
    double wobbleArmPower = 0;
    double wobbleClawPower = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        roller = hardwareMap.get(DcMotor.class, "roller");
        conveyorBeltL = hardwareMap.get(DcMotor.class, "conveyorL");
        conveyorBeltR = hardwareMap.get(DcMotor.class, "conveyorR");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        wobbleArmServo = hardwareMap.get(CRServoImpl.class, "wobbleArm");
        wobbleClawServo = hardwareMap.get(CRServoImpl.class, "wobbleClaw")

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        roller.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltL.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltR.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleArmServo.setDirection(CRServoImpl.Direction.FORWARD);
        wobbleClawServo.setDirection(CRServoImpl.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        turn = gamepad1.right_stick_x;

        rightFrontPower = -gamepad1.left_stick_y - gamepad1.left_stick_x -turn;
        rightBackPower = -gamepad1.left_stick_y + gamepad1.left_stick_x -turn;
        leftFrontPower =-gamepad1.left_stick_y + gamepad1.left_stick_x +turn;
        leftBackPower = -gamepad1.left_stick_y - gamepad1.left_stick_x +turn;

        maxPower = Math.abs(rightFrontPower);
        if(Math.abs(rightBackPower)>maxPower){
            maxPower = rightBackPower;
        }
        if(Math.abs(leftBackPower)>maxPower){
            maxPower = leftBackPower;
        }
        if(Math.abs(leftFrontPower)>maxPower){
            maxPower = leftFrontPower;
        }
        if(maxPower>1){
            rightFrontPower/=maxPower;
            leftFrontPower/=maxPower;
            rightBackPower/=maxPower;
            leftBackPower/=maxPower;
        }
        if(gamepad1.dpad_up){
            slowMode = false;
        }
        else if(gamepad1.dpad_down){
            slowMode = true;
        }
        if(slowMode == true){
            leftFrontPower *= .25;
            rightFrontPower *= .25;
            leftBackPower *= .25;
            rightBackPower *= .25;
        }

        leftBackDrive.setPower(leftBackPower);
        leftFrontDrive.setPower(leftFrontPower);
        rightBackDrive.setPower(rightBackPower);
        rightFrontDrive.setPower(rightFrontPower);

        double rollerPower;


        if(gamepad2.a){
            isIntakeRunning = true;
        }
        if(gamepad2.b){
            isIntakeRunning = false;
        }
        if(isIntakeRunning){
            rollerPower = 1;
        }
        else{
            rollerPower = 0;
        }

        roller.setPower(rollerPower);

        if(gamepad2.right_bumper){
            conveyorPower = 1;
        }
        else{
            conveyorPower = 0;
        }

        conveyorBeltL.setPower(conveyorPower);
        conveyorBeltR.setPower(conveyorPower);

        if(gamepad2.right_trigger != 0){
             shooterPower= 1;
        }
        else{
            shooterPower = 0;
        }

        shooterMotor.setPower(shooterPower);

        //Make an if statement to prevent the servo from going to far in certain directions.
        wobbleArmPower = -gamepad2.left_stick_y;

        //Same as before, add if statement to prevent servo angle from going too far.
        if(gamepad2.x){
            wobbleClawPower = 1;
        }
        else if(gamepad2.y){
            wobbleClawPower = -1;
        }
        else{
            wobbleClawPower = 0;
        }

        wobbleClawServo.setPower(wobbleClawPower);


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    @Override
    public void stop() {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}