package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="red right inner wobble goal high goal", group="Linear Opmode")
public class BoomerangPowerShotAuton extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor conveyorBelt = null;
    private TFObjectDetector tfod;
    private DcMotor shooter = null;

    /*private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        tfod.activate();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyorBelt");

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();
        AutonFunctionsP autonFunctions = new AutonFunctionsP(leftFrontDrive, rightFrontDrive,
                leftBackDrive, rightBackDrive, tfod, conveyorBelt, shooter);
        while (opModeIsActive()) {

            //code for power shot auton
            autonFunctions.left(20.25);
            autonFunctions.forward(54);
            autonFunctions.shoot();
            //rotate counterclockwise 5.95 degrees
            autonFunctions.shoot();
            //rotate counterclockwise 5.81 degrees
            autonFunctions.shoot();
            //rotate clockwise 11.76
            autonFunctions.forward(16);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}