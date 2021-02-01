/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/* BoomerangAuton is a Java program by Kapilesh P. and Paras N. that controls the Boomerang
robot during the Autonomous Period of the FTC Ultimate Goal competition. */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
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


@Autonomous(name="BoomerangAuton")
public class BoomerangAuton extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private CRServoImpl wobbleClawServo = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor shooter = null;
    private CRServoImpl conveyorBeltL = null;
    private CRServoImpl conveyorBeltR = null;

    private void highGoalAction() {
        forwardRobot(21);
        leftRobot(21);
        shoot();
        forwardRobot(21);
    }

    /* private void powerShotAction() {
        leftRobot(20.25);
        forwardRobot(54);
        shoot();
        rotateRobot(5.95); // rotate counterclockwise, make negative if needed
        shoot();
        rotateRobot(5.81); // rotate counterclockwise, make negative if needed
        shoot();
        rotateRobot(11.76); // rotate clockwise, make negative if needed
        forwardRobot(16);
    } */

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
            rotateRobot(180); // rotate clockwise, make negative if needed
            wobbleClawServo.setPower(-1); // drop wobble goal
            //if B:
            rotateRobot(90); // rotate clockwise, make negative if needed
            wobbleClawServo.setPower(-1); // drop wobble goal
            //if C:
            forwardRobot(36);
            rotateRobot(-180); // rotate clockwise, make negative if needed
            wobbleClawServo.setPower(-1); // drop wobble goal
            forwardRobot(36);
    } */

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
        conveyorBeltL = hardwareMap.get(CRServoImpl.class, "conveyorL");
        conveyorBeltR = hardwareMap.get(CRServoImpl.class, "conveyorR");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wobbleClawServo = hardwareMap.get(CRServoImpl.class, "wobbleClaw");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltL.setDirection(DcMotor.Direction.FORWARD);
        conveyorBeltR.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        wobbleClawServo.setDirection(CRServoImpl.Direction.FORWARD);

        stopRobot();
        resetEncoders();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            highGoalAction();
            // wobbleGoalAction();
            // powerShotAction();
            break;
        }
    }

    private void stopRobot() {
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        conveyorBeltL.setPower(0);
        conveyorBeltR.setPower(0);
        shooter.setPower(0);
        wobbleClawServo.setPower(0);
    }

    private void forwardRobot(double inches) {
        double inchesEncoderValue = Math.round(inches*((28*20)/(2*Math.PI*(49/25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValueRounded = (int) inchesEncoderValue;
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValueRounded);
        rightBackMotor.setTargetPosition(encoderValueRounded);
        leftFrontMotor.setTargetPosition(encoderValueRounded);
        rightFrontMotor.setTargetPosition(encoderValueRounded);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(1);

        telemetry.addData("leftBack", (inchesEncoderValue));
        telemetry.update();

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void backwardRobot(double inches) {
        double inchesEncoderValue = Math.round(inches*((28*20)/(2*Math.PI*(49/25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValueRounded = (int) inchesEncoderValue;
        resetEncoders();

        leftBackMotor.setTargetPosition(-encoderValueRounded);
        rightBackMotor.setTargetPosition(-encoderValueRounded);
        leftFrontMotor.setTargetPosition(-encoderValueRounded);
        rightFrontMotor.setTargetPosition(-encoderValueRounded);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(-1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(-1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void rightRobot(double inches){
        double inchesEncoderValue = Math.round(inches*((28*20)/(2*Math.PI*(49/25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValueRounded = (int) inchesEncoderValue;
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValueRounded);
        rightBackMotor.setTargetPosition(-encoderValueRounded);
        leftFrontMotor.setTargetPosition(-encoderValueRounded);
        rightFrontMotor.setTargetPosition(encoderValueRounded);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(-1);
        rightFrontMotor.setPower(1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void leftRobot(double inches){
        double inchesEncoderValue = Math.round(inches*((28*20)/(2*Math.PI*(49/25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValueRounded = (int) inchesEncoderValue;
        resetEncoders();

        leftBackMotor.setTargetPosition(-encoderValueRounded);
        rightBackMotor.setTargetPosition(encoderValueRounded);
        leftFrontMotor.setTargetPosition(encoderValueRounded);
        rightFrontMotor.setTargetPosition(-encoderValueRounded);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(-1);
        rightBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
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
        double degreesEncoderValue = Math.round(degrees*((28*20)/(2*Math.PI*(49/25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValueRounded = (int) degreesEncoderValue;
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValueRounded);
        rightBackMotor.setTargetPosition(-encoderValueRounded);
        leftFrontMotor.setTargetPosition(encoderValueRounded);
        rightFrontMotor.setTargetPosition(-encoderValueRounded);

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackMotor.setPower(1);
        rightBackMotor.setPower(-1);
        leftFrontMotor.setPower(1);
        rightFrontMotor.setPower(-1);

        while(leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void shoot() {
        runtime.reset();
        while (getRuntime() < 10) {
            conveyorBeltR.setPower(1);
            conveyorBeltL.setPower(1);
            shooter.setPower(1);
            getRuntime();
        }
        stopRobot();
    }
}
