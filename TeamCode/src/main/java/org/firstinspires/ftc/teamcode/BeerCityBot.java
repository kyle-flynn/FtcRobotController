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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name="Beer City Bot", group="Linear OpMode")
public class BeerCityBot extends LinearOpMode {
    // [LIFT] Constant declarations
    private final int LIFT_HOME_POSITION = 0;
    private final int LIFT_LOW_POSITION = 2000;
    private final int LIFT_HIGH_POSITION = 4000;
    private final double LIFT_POWER = 0.5;

    // [INTAKE] Constant declarations
    private final int INTAKE_ARM_HOME_POSITION = -2000;
    private final int INTAKE_ARM_DOWN_POSITION = -10200;
    private final int INTAKE_ARM_HAND_OFF_POSITION = -4000;
    private final int INTAKE_ARM_SCORING_POSITION = -5000;
    private final double INTAKE_ARM_POWER = 1.0;
    private final double INTAKE_SERVO_POWER = -0.5;

    // [BUCKET] Constant declarations
    private final double BUCKET_HOME_POSITION = 0.2;
    private final double BUCKET_SCORING_POSITION = 0.8;
    private final double BUCKET_HAND_OFF_POSITION = 0.75;

    // [CLIMB] Constant declarations
    private final int CLIMB_HOME_POSITION = 0;
    private final int CLIMB_MAX_POSITION = -27500;
    private final double CLIMB_POWER = 1.0;

    // Robot state declarations
    private enum RobotState {
        DRIVE,
        INTAKE,
        HAND_OFF,
        LOW_GOAL,
        HIGH_GOAL
    }
    private RobotState robotState;
    private int lastStateChangeButton;
    private boolean isScoring;
    private boolean isClimbing;
    private boolean isUsingOverride;
    private boolean robotStateOverride;
    private boolean liftLimitReached;

    // Motor declarations
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor lift;
    private DcMotor intakeArm;
    private DcMotor climbArm;

    // Servo declarations
    private CRServo intakeServo;
    private Servo bucketServo;

    // DigitalInput declarations
    private TouchSensor liftLimit;

    // Drive modifiers
    private double speed;
    private double forward;
    private double strafe;
    private double denominator;
    private double turn;

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
        lastStateChangeButton = -1;
        isScoring = false;
        isClimbing = false;
        liftLimitReached = true;

        // Initialize drive modifiers
        speed = 0.7;

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
            readDriverInputs();
            readOperatorInputs();
            drive();
            updateLift();
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

        climbArm = hardwareMap.get(DcMotor.class, "climbArm");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");

        // Invert right side motors since they are mirrored on the chassis
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // For our motors with encoders, reset their positions to 0.
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                intakeArm.setTargetPosition(INTAKE_ARM_HAND_OFF_POSITION);
                lift.setTargetPosition(LIFT_HOME_POSITION);
                break;
            case LOW_GOAL:
                intakeArm.setTargetPosition(INTAKE_ARM_SCORING_POSITION);
                lift.setTargetPosition(LIFT_LOW_POSITION);
                break;
            case HIGH_GOAL:
                intakeArm.setTargetPosition(INTAKE_ARM_SCORING_POSITION);
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
                bucketServo.setPosition(BUCKET_HOME_POSITION);
                break;
            case INTAKE:
                intakeServo.setPower(INTAKE_SERVO_POWER);
                bucketServo.setPosition(BUCKET_HOME_POSITION);
                break;
            case HAND_OFF:
                intakeServo.setPower(0.0);
                bucketServo.setPosition(BUCKET_HAND_OFF_POSITION);
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

    private void readDriverInputs() {
        if (gamepad1.dpad_up) {
            speed = 1;
        } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            speed = 0.7;
        } else if (gamepad1.dpad_down) {
            speed = 0.4;
        }
        forward = -gamepad1.left_stick_y * speed;
        strafe = gamepad1.left_stick_x * speed;
        turn = (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)) * speed;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + strafe + turn));
    }

    private void drive() {
        leftFront.setPower((forward + strafe + turn) / denominator);
        leftBack.setPower((forward - (strafe - turn)) / denominator);
        rightFront.setPower((forward - (strafe + turn)) / denominator);
        rightBack.setPower((forward + (strafe - turn)) / denominator);
    }

    private void readOperatorInputs() {
        if (gamepad2.a && lastStateChangeButton != 0) {
            lastStateChangeButton = 0;
            advanceRobotState(true);
            updateRobotSystemPositions();
            updateRobotSystemServos();
        } else if (gamepad2.b && lastStateChangeButton != 1) {
            lastStateChangeButton = 1;
            advanceRobotState(false);
            updateRobotSystemPositions();
            updateRobotSystemServos();
        } else if (gamepad2.x && lastStateChangeButton != 2) {
            lastStateChangeButton = 2;
            robotState = RobotState.DRIVE;
            updateRobotSystemPositions();
            updateRobotSystemServos();
        } else if (!gamepad2.a && lastStateChangeButton == 0) {
            lastStateChangeButton = -1;
        } else if (!gamepad2.b && lastStateChangeButton == 1) {
            lastStateChangeButton = -1;
        } else if (!gamepad2.x && lastStateChangeButton == 2) {
            lastStateChangeButton = -1;
        }

        if (gamepad2.y && !isScoring) {
            boolean isServoHome = bucketServo.getPosition() == BUCKET_HOME_POSITION;
            bucketServo.setPosition(isServoHome  ? BUCKET_SCORING_POSITION : BUCKET_HOME_POSITION);
            isScoring = true;
        } else if (!gamepad2.y && isScoring) {
            isScoring = false;
        }

        if (gamepad2.right_bumper && !isClimbing) {
            climbArm.setTargetPosition(CLIMB_MAX_POSITION);
            climbArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climbArm.setPower(CLIMB_POWER);
            isClimbing = true;
        } else if (!gamepad2.right_bumper && isClimbing) {
            isClimbing = false;
        }

        if (gamepad2.left_bumper && !isClimbing) {
            climbArm.setTargetPosition(CLIMB_HOME_POSITION);
            climbArm.setPower(CLIMB_POWER);
            climbArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            isClimbing = true;
        } else if (!gamepad2.left_bumper && isClimbing) {
            isClimbing = false;
        }

        if (gamepad2.start && !isUsingOverride) {
            isUsingOverride = true;
            robotStateOverride = !robotStateOverride;
            if (robotStateOverride) {
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climbArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else if (!gamepad2.start && isUsingOverride) {
            isUsingOverride = false;
        }

        if (robotStateOverride) {
            handleOverrides();
        }
    }

    /**
     * Override method for the robot's state. Code in this method should only be executed
     * if the robot is in override mode, otherwise the robot should be controlled by the state machine.
     */
    private void handleOverrides() {
        lift.setPower(gamepad2.left_stick_y * -LIFT_POWER);
        intakeArm.setPower(gamepad2.right_stick_y * -INTAKE_ARM_POWER);
        climbArm.setPower((-gamepad2.left_trigger + gamepad2.right_trigger) * CLIMB_POWER);
    }


    private void updateLift() {
        if (liftLimit.isPressed() && !liftLimitReached) {
            liftLimitReached = true;
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (!liftLimit.isPressed() && liftLimitReached) {
            liftLimitReached = false;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Robot State", robotStateOverride ? "OVERRIDE" : robotState.toString());
        telemetry.addLine();
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Lift Target", lift.getTargetPosition());
        telemetry.addData("Lift Limit", liftLimit.isPressed());
        telemetry.addLine();
        telemetry.addData("Intake Position", intakeArm.getCurrentPosition());
        telemetry.addData("Intake Target", intakeArm.getTargetPosition());
        telemetry.addLine();
        telemetry.addData("Bucket Servo", bucketServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Climb", climbArm.getCurrentPosition());
        telemetry.update();
    }
}
