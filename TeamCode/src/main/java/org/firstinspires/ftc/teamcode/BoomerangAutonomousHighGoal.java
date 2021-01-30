package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math.*;


@Autonomous(name="BoomerangAutonomousHighGoal")
public class BoomerangAutonomousHighGoal extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private CRServoImpl wobbleClawServo = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor conveyorBeltMotor = null;
    DcMotor conveyorBeltL = null;
    DcMotor conveyorBeltR = null;


    public BoomerangAutonomousHighGoal() {
        super();
    }

   /*  public void main() throws InterruptedException {
        highGoalAction();
    } */

    private void highGoalAction() {
        forwardRobot(21);
        leftRobot(21);
        shoot();
        forwardRobot(21);
    }

    @Override
    public void runOpMode() {
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
        wobbleClawServo = hardwareMap.get(CRServoImpl.class, "wobbleClaw");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltL.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltR.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltMotor.setDirection(DcMotor.Direction.FORWARD);
        wobbleClawServo.setDirection(CRServoImpl.Direction.FORWARD);
        stopRobot();

        resetEncoders();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) highGoalAction();



       /* while(opModeIsActive()) {
            highGoalAction();
            break;
            // wobbleGoalAction();
        }*/
    }

    private void stopRobot() {
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBeltL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBeltR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        conveyorBeltL.setPower(0);
        conveyorBeltR.setPower(0);
        conveyorBeltMotor.setPower(0);
        wobbleClawServo.setPower(0);
    }

    private void forwardRobot(double inches) {
        int inchesEncoderValue = (int)((inches/(Math.PI*98))*(20))/28; //(((inches/circumference)/gearingRatio))/28; must be fixed later
        double motorEncoderValue = leftBackMotor.getCurrentPosition();
        resetEncoders();

        leftBackMotor.setTargetPosition((inchesEncoderValue));

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            // Continue
        }
        stopRobot();
    }

    private void backwardRobot(double inches) {
        int inchesEncoderValue = (int)((inches/(Math.PI*98))*(20))/28; //(((inches/circumference)/gearingRatio))/28; must be fixed later
        double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
        resetEncoders();

        leftBackMotor.setTargetPosition((inchesEncoderValue));

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(-1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(-1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            // Continue
        }
        stopRobot();
    }

    private void leftRobot(double inches){
        int inchesEncoderValue = (int)((inches/(Math.PI*98))*(20))/28; //(((inches/circumference)/gearingRatio))/28; must be fixed later
        double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
        resetEncoders();

        leftBackMotor.setTargetPosition((inchesEncoderValue));
        rightBackMotor.setTargetPosition(inchesEncoderValue);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(-1);
        rightFrontMotor.setPower(1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            // Continue
        }
        stopRobot();
    }

    private void rightRobot(double inches){
        int inchesEncoderValue = (int)((inches/(Math.PI*98))*(20))/28;
        double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
        resetEncoders();

        rightBackMotor.setTargetPosition((inchesEncoderValue));
        leftBackMotor.setTargetPosition(inchesEncoderValue);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(-1);
        rightBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            // Continue
        }
        stopRobot();
    }

    private void resetEncoders() {
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void rotateRobot(double degrees){
        int degreesEncoderValue = (int)((degrees/(Math.PI*98))*(20))/28; //must be fixed later
        double motorEncoderValue = (leftBackMotor.getCurrentPosition())*-1;
        resetEncoders();

        leftBackMotor.setTargetPosition((degreesEncoderValue));
        rightBackMotor.setTargetPosition(degreesEncoderValue);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftBackMotor.setPower(1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            // Continue
        }
        stopRobot();
    }

    private void shoot() {
        runtime.reset();
        while (getRuntime() < 10) {
            conveyorBeltR.setPower(1);
            conveyorBeltL.setPower(1);
            conveyorBeltMotor.setPower(1);
            getRuntime();
        }
        stopRobot();
    }





    /* private void wobbleGoalAction() {
            leftRobot(3);
            forwardRobot(18);
            //sense for # of rings
            leftRobot(21);
            forwardRobot(36);
            rightRobot(21);
            shoot();
            rightRobot(3);
            forwardRobot(24);
            // if A:
            rotateRobot(180);
            wobbleClawServo.setPower(-1); // drop wobble goal
            //if B:
            rotateRobot(90);
            wobbleClawServo.setPower(-1); // drop wobble goal
            //if C:
            forwardRobot(36);
            rotateRobot(-180);
            wobbleClawServo.setPower(-1); // drop wobble goal
            forwardRobot(36);
        } */
}
