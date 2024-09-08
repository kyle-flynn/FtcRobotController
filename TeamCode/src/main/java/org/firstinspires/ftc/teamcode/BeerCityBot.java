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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Beer City Bot", group="Linear OpMode")
public class BeerCityBot extends LinearOpMode {
    // [LIFT] Constant declarations
    private final int LIFT_HOME_POSITION = 0;
    private final int LIFT_LOW_POSITION = 2000;
    private final int LIFT_HIGH_POSITION = 4000;
    private final double LIFT_POWER = 0.5;

    // [INTAKE] Constant declarations
    private final int INTAKE_ARM_HOME_POSITION = 0;
    private final int INTAKE_ARM_DOWN_POSITION = -8000;
    private final double INTAKE_ARM_POWER = 0.6;
    private final double INTAKE_SERVO_POWER = -0.5;

    // Robot state declarations
    private enum RobotState {
        DRIVE,
        INTAKE,
        HAND_OFF,
        LOW_GOAL,
        HIGH_GOAL
    }
    private RobotState robotState;
    private boolean robotStateChangeRequested;

    // Motor declarations
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor lift;
    private DcMotor intakeArm;

    // Motor stall declarations
    private boolean isLiftStalling;
    private double liftStallTime;
    private int liftEncoderDrift;
    private boolean isArmStalling;
    private double armStallTime;
    private int armEncoderDrift;

    // Servo Declarations
    private CRServo intakeServo;

    private ElapsedTime time;

    /**
     * NOTE: Before the robot is started, it is assumed that the robot is in the following configuration:
     * LIFT - is set to the "home" position, bottomed out
     * INTAKE ARM - is set to the "home" position, all the way up in starting configuration
     */
    @Override
    public void runOpMode() {
        time = new ElapsedTime();

        // Setup our robot state
        robotState = RobotState.DRIVE;
        robotStateChangeRequested = false;

        // Initialize stall variables
        isLiftStalling = false;
        liftStallTime = 0.0;
        isArmStalling = false;
        armStallTime = 0.0;

        // Map all of our declarations to hardware
        mapHardware();

        // Update our telemetry
        updateTelemetry();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Rest the runtime once the robot has started
        time.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            updateRobotState();
            checkForStallingMotors();
            updateTelemetry();
        }
    }

    private void mapHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        lift = hardwareMap.get(DcMotor.class, "lift");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // For our motors with encoders, reset their positions to 0.
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void updateRobotState() {
        waitForRobotStateChange();
        updateRobotSystemServos();
    }

    private void waitForRobotStateChange() {
        if (gamepad1.a && !robotStateChangeRequested) {
            robotStateChangeRequested = true;
            advanceRobotState(true);
            updateRobotSystemPositions();
        } else if (gamepad1.b && !robotStateChangeRequested) {
            robotStateChangeRequested = true;
            advanceRobotState(false);
            updateRobotSystemPositions();
        } else if ((!gamepad1.a || !gamepad1.b) && robotStateChangeRequested) {
            robotStateChangeRequested = false;
        }
    }

    private void advanceRobotState(boolean advance) {
        switch (robotState) {
            case DRIVE:
                robotState = advance ? RobotState.INTAKE : RobotState.HIGH_GOAL;
                break;
            case INTAKE:
                robotState = advance ? RobotState.HAND_OFF : RobotState.DRIVE;
                break;
            case HAND_OFF:
                robotState = advance ? RobotState.LOW_GOAL : RobotState.INTAKE;
                break;
            case LOW_GOAL:
                robotState = advance ? RobotState.HIGH_GOAL : RobotState.HAND_OFF;
                break;
            case HIGH_GOAL:
                robotState = advance ? RobotState.DRIVE : RobotState.LOW_GOAL;
                break;
            default:
                robotState = RobotState.DRIVE;
        }
    }

    private void updateRobotSystemPositions() {
        // TODO - Incorporate drift values, needs additional testing.
        switch (robotState) {
            case DRIVE:
                intakeArm.setTargetPosition(INTAKE_ARM_HOME_POSITION);
                lift.setTargetPosition(LIFT_HOME_POSITION);
                break;
            case INTAKE:
                intakeArm.setTargetPosition(INTAKE_ARM_DOWN_POSITION);
                lift.setTargetPosition(LIFT_HOME_POSITION);
                break;
            case HAND_OFF:
                intakeArm.setTargetPosition(INTAKE_ARM_HOME_POSITION);
                lift.setTargetPosition(LIFT_HOME_POSITION);
                break;
            case LOW_GOAL:
                intakeArm.setTargetPosition(INTAKE_ARM_HOME_POSITION);
                lift.setTargetPosition(LIFT_LOW_POSITION);
                break;
            case HIGH_GOAL:
                intakeArm.setTargetPosition(INTAKE_ARM_HOME_POSITION);
                lift.setTargetPosition(LIFT_HIGH_POSITION);
                break;
            default:
                intakeArm.setTargetPosition(INTAKE_ARM_HOME_POSITION);
                lift.setTargetPosition(LIFT_HOME_POSITION);
        }
        intakeArm.setPower(INTAKE_ARM_POWER);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(LIFT_POWER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void updateRobotSystemServos() {
        switch (robotState) {
            case DRIVE:
                intakeServo.setPower(0.0);
                break;
            case INTAKE:
                intakeServo.setPower(INTAKE_SERVO_POWER);
                break;
            case HAND_OFF:
                intakeServo.setPower(0.0);
                break;
            case LOW_GOAL:
                intakeServo.setPower(0.0);
                break;
            case HIGH_GOAL:
                intakeServo.setPower(0.0);
                break;
            default:
                intakeServo.setPower(0.0);
        }
    }

    private void checkForStallingMotors() {
        int liftError = Math.abs(lift.getTargetPosition() - lift.getCurrentPosition());
        int armError = Math.abs(intakeArm.getTargetPosition() - intakeArm.getCurrentPosition());

        if (liftError <= 100 && liftStallTime <= 0.0) {
            // Now we've got the moment the lift became within 100 ticks of its target
            liftStallTime = time.time();
        }

        // Now we check if the error is still within 100 ticks, and if the elapsed time has exceeded the boundaries
        if ((time.time() - liftStallTime) >= 2.0 && liftError <= 100) {
            // The motor has stalled, set the target position to the new position
            liftEncoderDrift = lift.getTargetPosition() - lift.getCurrentPosition();
            lift.setTargetPosition(lift.getCurrentPosition());
            liftStallTime = 0.0;
        }

        if (armError <= 100 && armStallTime <= 0.0) {
            // Now we've got the moment the lift became within 100 ticks of its target
            armStallTime = time.time();
        }

        // Now we check if the error is still within 100 ticks, and if the elapsed time has exceeded the boundaries
        if ((time.time() - armStallTime) >= 2.0 && armError <= 100) {
            // The motor has stalled, set the target position to the new position
            armEncoderDrift = intakeArm.getTargetPosition() - lift.getCurrentPosition();
            intakeArm.setTargetPosition(intakeArm.getCurrentPosition());
            armStallTime = 0.0;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Robot State", robotState.toString());
        telemetry.addLine();
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Lift Target", lift.getTargetPosition());
        telemetry.addData("Lift Stall Time", liftStallTime);
        telemetry.addLine();
        telemetry.addData("Intake Position", intakeArm.getCurrentPosition());
        telemetry.addData("Intake Target", intakeArm.getTargetPosition());
        telemetry.addData("Intake Stall Time", armStallTime);
        telemetry.update();
    }
}
