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


/* BoomerangTeleOp is a Java Program developed by Paras N. with assistance from Kapilesh P. that
allows a user to use controllers to control the Boomerang robot (BoomerBoi) during the
driver-controlled period of the FTC Ultimate Goal Competition. */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BoomerangTeleOp", group = "Iterative Opmode")

public class BoomerangTeleOp extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    // Power variables
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
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private boolean slowMode = false;
    private DcMotor intake = null;
    private boolean isIntakeRunning = false;
    private boolean isIntakeRunningRev = false;
    private boolean isShooterRunning = false;
    private DcMotor conveyorBelt = null;
    private DcMotorEx shooterMotor = null;
    private CRServoImpl wobbleArmServo = null;
    private CRServoImpl wobbleClawServo = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        conveyorBelt = hardwareMap.get(DcMotor.class, "conveyor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        wobbleArmServo = hardwareMap.get(CRServoImpl.class, "wobbleArm");
        wobbleClawServo = hardwareMap.get(CRServoImpl.class, "wobbleClaw");

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor.setVelocityPIDFCoefficients(90,0,0,0);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);
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
        telemetry.addData("Speed:", shooterMotor.getVelocity());

        turn = -gamepad1.right_stick_x;

        rightFrontPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + turn;
        rightBackPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + turn;
        leftFrontPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - turn;
        leftBackPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - turn;


        maxPower = Math.abs(rightFrontPower);
        if (Math.abs(rightBackPower) > maxPower) {
            maxPower = rightBackPower;
        }
        if (Math.abs(leftBackPower) > maxPower) {
            maxPower = leftBackPower;
        }
        if (Math.abs(leftFrontPower) > maxPower) {
            maxPower = leftFrontPower;
        }
        if (maxPower > 1) {
            rightFrontPower /= maxPower;
            leftFrontPower /= maxPower;
            rightBackPower /= maxPower;
            leftBackPower /= maxPower;
        }
        if (gamepad1.dpad_up) {
            slowMode = false;
        } else if (gamepad1.dpad_down) {
            slowMode = true;
        }
        if (slowMode) {
            leftFrontPower *= -0.35;
            rightFrontPower *= -0.35;
            leftBackPower *= -0.35;
            rightBackPower *= -0.35;
        }
        if (!slowMode) {
            leftFrontPower *= -0.6;
            rightFrontPower *= -0.6;
            leftBackPower *= -0.6;
            rightBackPower *= -0.6;
        }


        leftBackDrive.setPower(leftBackPower);
        leftFrontDrive.setPower(leftFrontPower);
        rightBackDrive.setPower(rightBackPower);
        rightFrontDrive.setPower(rightFrontPower);

        double intakePower = 0;


        if (gamepad2.a) {
            isIntakeRunning = true;
        }
        if (gamepad2.y) {
            isIntakeRunningRev = true;
        }
        if (gamepad2.b) {
            isIntakeRunningRev = false;
            isIntakeRunning = false;
        }

        if (isIntakeRunning) {
            intakePower = 1;
        }

        if (isIntakeRunningRev) {
            intakePower = -1;
        }

        intake.setPower(intakePower);

        if (gamepad2.right_bumper) {
            conveyorPower = 1;
        }
        else if (gamepad2.left_bumper) {
            conveyorPower = -0.7;
        }
        else {
            conveyorPower = 0;
        }

        conveyorBelt.setPower(conveyorPower);

        //if (gamepad2.right_trigger != 0) {
        //shooterPower = 1;
        //} else {
        //shooterPower = 0;
        //}

        if (gamepad2.right_trigger != 0) {
            isShooterRunning = true;
        }
        if (gamepad2.left_trigger != 0) {
            isShooterRunning = false;
        }
        if (isShooterRunning) {
            shooterPower = -2325;
        }
        else {
            shooterPower = 0;
        }
        shooterMotor.setVelocity(shooterPower);

        //Make an if statement to prevent the servo from going to far in certain directions.
        wobbleArmPower = -gamepad2.left_stick_y;

        //Same as before, add if statement to prevent servo angle from going too far.
        if (gamepad2.x) {
            wobbleClawPower = 1;
        } else if (gamepad2.y) {
            wobbleClawPower = -1;
        } else {
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