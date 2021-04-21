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


/* BoomerangAuton is a Java program by Kapilesh P. with assistance from Paras N. that controls the
Boomerang robot (BoomerBoi) during the Autonomous Period of the FTC Ultimate Goal competition. */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImpl;
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
        rotateRobot(-5.95);
        shoot();
        rotateRobot(-5.81);
        shoot();
        rotateRobot(11.76);
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

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(70, 0, 0, 0);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        wobbleClawServo.setDirection(CRServoImpl.Direction.FORWARD);
        wobbleArmServo.setDirection(CRServoImpl.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        stopRobot();
        resetEncoders();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            highGoalAction();
            forwardRobot(40, 1);  // Parking (5 pts)
            // wobbleGoalAction();
            // powerShotAction();
            break;
        }
    }

    private void highGoalAction() {
        // start left of the leftmost red line
        forwardRobot(30, 0.1);
        rotateRobot(-5.5, 0.1);
        shoot(0,-1);
        rotateRobot(5.5, 0.1);
    }

    private int calculateEncoderTicks(double inches) {
        double inchesEncoderValue = Math.round(inches * ((28 * 20) / (2 * Math.PI * (49 / 25.4)))); //Formula for Encoder Ticks per Revolution = (encoderTicksPerRevolution*gearingRatio)/circumference, circumference = 2*pi*radius
        int encoderValue = (int) inchesEncoderValue;
        return encoderValue;
    }

    private void stopRobot() {
        leftBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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
        leftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void wheelEncoderSettings() {
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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

    private void shoot(double intakePower, double conveyorPower) {
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setVelocity(2325);
        sleep(4000);
        telemetry.addData("Speed:", shooter.getVelocity());
        telemetry.update();
        conveyorBelt.setPower(conveyorPower);
        sleep(100);
        conveyorBelt.setPower(0);
        sleep(4200);
        telemetry.addData("Speed:", shooter.getVelocity());
        telemetry.update();
        conveyorBelt.setPower(conveyorPower);
        // intake.setPower(intakePower);
        sleep(400);
        conveyorBelt.setPower(0);
        sleep(4200);
        telemetry.addData("Speed:", shooter.getVelocity());
        telemetry.update();
        conveyorBelt.setPower(conveyorPower);
        sleep(900);
        stopRobot();
    }
}