package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.BoomerangAutonFunctions.forward;

@Autonomous(name="HighGoal")
public class HighGoal extends OpMode {

    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor conveyorBeltMotor = null;
    DcMotor conveyorBeltL = null;
    DcMotor conveyorBeltR = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightBack");
        conveyorBeltL = hardwareMap.get(DcMotor.class, "conveyorL");
        conveyorBeltR = hardwareMap.get(DcMotor.class, "conveyorR");
        conveyorBeltMotor = hardwareMap.get(DcMotor.class, "shooter");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        forwardRobot(21);
        leftRobot(21);
        //Shoot
        forwardRobot(21);

    }

    private void stopRobot() {
            leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
        }

        private void forwardRobot(double inches) {
            double inchesEncoderValue = 1; //(((inches/circumference)/gearingRatio))/28; must be fixed later
            double motorEncoderValue = leftBackMotor.getCurrentPosition();
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBackMotor.setPower(1);
            rightBackMotor.setPower(1);
            leftFrontMotor.setPower(1);
            rightFrontMotor.setPower(1);
            while(motorEncoderValue<inchesEncoderValue) {
                motorEncoderValue = leftBackMotor.getCurrentPosition();
            }
            stopRobot();
        }

        private void backwardRobot(double inches) {
            double inchesEncoderValue = 1; //(((inches/circumference)/gearingRatio))/28; must be fixed later
            double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBackMotor.setPower(-1);
            rightBackMotor.setPower(-1);
            leftFrontMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
            while(motorEncoderValue<inchesEncoderValue) {
                motorEncoderValue = leftBackMotor.getCurrentPosition();
            }
            stopRobot();
        }

        private void leftRobot(double inches){
            double inchesEncoderValue = 1; //(((inches/circumference)/gearingRatio))/28; must be fixed later
            double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBackMotor.setPower(1);
            rightBackMotor.setPower(-1);
            leftFrontMotor.setPower(-1);
            rightFrontMotor.setPower(1);
            while (motorEncoderValue < inchesEncoderValue) {
                motorEncoderValue = leftBackMotor.getCurrentPosition();
            }
            stopRobot();
        }

        private void rightRobot(double inches){
            double inchesEncoderValue = 1; //(((inches/circumference)/gearingRatio))/28; must be fixed later
            double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBackMotor.setPower(-1);
            rightBackMotor.setPower(1);
            leftFrontMotor.setPower(1);
            rightFrontMotor.setPower(-1);
            while (motorEncoderValue < inchesEncoderValue) {
                motorEncoderValue = rightBackMotor.getCurrentPosition();
            }
            stopRobot();
        }

        private void rotateRobot(double degrees){
            double degreesEncoderValue = 1; //must be fixed later
            double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftBackMotor.setPower(1);
            rightBackMotor.setPower(-1);
            leftFrontMotor.setPower(1);
            rightFrontMotor.setPower(-1);
            while (motorEncoderValue < degreesEncoderValue) {
                motorEncoderValue = leftBackMotor.getCurrentPosition();
            }
            stopRobot();
        }

        private void shoot(double power) {
            // Write Code Here
        }

        // The following code is being used for reference. It may or may not be included in the final code.
/*        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();*/
        }
    }
}

