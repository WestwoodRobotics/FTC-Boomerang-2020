package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="red right inner wobble goal high goal", group="Linear Opmode")
public class BoomerangAuton extends LinearOpMode {

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

            //code for how the robot will move
            autonFunctions.left(3);
            autonFunctions.pause(1);
            autonFunctions.forward(18);
            autonFunctions.pause(1);
            //sense number of rings
            autonFunctions.left(21);
            autonFunctions.pause(1);
            autonFunctions.forward(36);
            autonFunctions.pause(1);
            autonFunctions.right(21);
            autonFunctions.pause(1);
            autonFunctions.shoot();
            autonFunctions.shoot();
            autonFunctions.shoot();
            autonFunctions.right(3);
            autonFunctions.pause(1);
            autonFunctions.forward(21);
            autonFunctions.pause(1);
            if(autonFunctions.ringLocation == 0) {
                autonFunctions.rotateCenter(180, "clockwise");
                //drop wobble goal
                autonFunctions.pause(1);
            }
            else if(autonFunctions.ringLocation == 1){
                autonFunctions.rotateCenter(90, "clockwise");
                //drop wobble goal
                autonFunctions.pause(1);
            }
            else if(autonFunctions.ringLocation == 2){
                autonFunctions.forward(36);
                autonFunctions.pause(1);
                autonFunctions.rotateCenter(180, "counterclockwise");
                //drop wobble goal
                autonFunctions.forward(36);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
    /*private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    */
}