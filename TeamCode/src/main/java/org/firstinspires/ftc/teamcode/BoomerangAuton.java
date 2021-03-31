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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BoomerangAuton")
public class BoomerangAuton extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    CRServoImpl wobbleClawServo = null;
    CRServoImpl wobbleArmServo = null;
    DcMotorEx leftBackMotor = null;
    DcMotorEx rightBackMotor = null;
    DcMotorEx leftFrontMotor = null;
    DcMotorEx rightFrontMotor = null;
    DcMotorEx shooter = null;
    DcMotorEx conveyorBelt = null;
    DcMotorEx intake = null;

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
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightBack");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        wobbleClawServo = hardwareMap.get(CRServoImpl.class, "wobbleClaw");
        wobbleArmServo = hardwareMap.get(CRServoImpl.class, "wobbleArm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(2000, 0, 0, 0);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        wobbleClawServo.setDirection(CRServoImpl.Direction.FORWARD);
        wobbleArmServo.setDirection(CRServoImpl.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        stopRobot();
        resetEncoders();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            highGoalAction();
            forwardRobot(12, 1);
            // wobbleGoalAction();
            // powerShotAction();
            break;
        }
    }

    private void highGoalAction() {
        // start left of the leftmost red line
        forwardRobot(60, 1);
        rotateRobot(-10, 0.3);
        shoot(0.5,0.5, 0.52);
        rotateRobot(10, 0.3);
    }

    private int calculateEncoderTicks(double inches) {
        double inchesEncoderValue = Math.round(inches * ((28 * 20) / (2 * Math.PI * (49 / 25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValue = (int) inchesEncoderValue;
        return encoderValue;
    }

    private void stopRobot() {
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        conveyorBelt.setPower(0);
        shooter.setPower(0);
        wobbleClawServo.setPower(0);
        intake.setPower(0);
        wobbleArmServo.setPower(0);

        sleep(1000);
    }

    private void backwardRobot(double inches, double power) {
        int encoderValue = calculateEncoderTicks(inches);
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValue);
        rightBackMotor.setTargetPosition(encoderValue);
        leftFrontMotor.setTargetPosition(encoderValue);
        rightFrontMotor.setTargetPosition(encoderValue);

        wheelEncoderSettings();

        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);

        while (leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void forwardRobot(double inches, double power) {
        int encoderValue = calculateEncoderTicks(inches);
        resetEncoders();

        leftBackMotor.setTargetPosition(-encoderValue);
        rightBackMotor.setTargetPosition(-encoderValue);
        leftFrontMotor.setTargetPosition(-encoderValue);
        rightFrontMotor.setTargetPosition(-encoderValue);

        wheelEncoderSettings();

        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(-power);

        while (leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void rightRobot(double inches, double power) {
        int encoderValue = calculateEncoderTicks(inches);
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValue);
        rightBackMotor.setTargetPosition(-encoderValue);
        leftFrontMotor.setTargetPosition(-encoderValue);
        rightFrontMotor.setTargetPosition(encoderValue);

        wheelEncoderSettings();

        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);

        while (leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void leftRobot(double inches, double power) {
        int encoderValue = calculateEncoderTicks(inches);
        resetEncoders();

        leftBackMotor.setTargetPosition(-encoderValue);
        rightBackMotor.setTargetPosition(encoderValue);
        leftFrontMotor.setTargetPosition(encoderValue);
        rightFrontMotor.setTargetPosition(-encoderValue);

        wheelEncoderSettings();

        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);

        while (leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
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

    private void wheelEncoderSettings() {
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void rotateRobot(double degrees, double power) {
        int encoderValue = calculateEncoderTicks(degrees);
        resetEncoders();

        leftBackMotor.setTargetPosition(encoderValue);
        rightBackMotor.setTargetPosition(-encoderValue);
        leftFrontMotor.setTargetPosition(encoderValue);
        rightFrontMotor.setTargetPosition(-encoderValue);

        wheelEncoderSettings();

        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);

        while (leftBackMotor.isBusy() && rightBackMotor.isBusy() && leftFrontMotor.isBusy() && rightFrontMotor.isBusy()) {
            //continue
        }
        stopRobot();
    }

    private void shoot(double intakePower, double conveyorPower, double shooterPower) {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setVelocity(2000);
        sleep(4200);
        conveyorBelt.setPower(conveyorPower);
        sleep(1000);
        conveyorBelt.setPower(0);
        sleep(2000);
        conveyorBelt.setPower(conveyorPower);
        intake.setPower(intakePower);
        sleep(1000);
        conveyorBelt.setPower(0);
        sleep(2000);
        conveyorBelt.setPower(conveyorPower);
        sleep(3000);
        stopRobot();
    }
}