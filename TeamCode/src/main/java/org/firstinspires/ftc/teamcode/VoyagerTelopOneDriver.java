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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="V1TelopOneDriver", group="VoyagerBot")

public class VoyagerTelopOneDriver extends LinearOpMode {
    final PathSeg[] mAutoParkPath = {
            new PathSeg(-8, 8, 0.4),
            new PathSeg(-30.0, -30.0, 0.4),
            new PathSeg(8.0, 8.0, 0.4)
    };

    /* Declare OpMode members. */
    VoyagerHardware robot           = new VoyagerHardware();
    private StateMachine stateMachineRetract;
    private StateMachine stateMachineAutoPark;
    private StateMachine currentStateMachine = null;
    private RobotHelper  helper = new RobotHelper(robot);

    void setupStatemMachineRetract() {
        if (currentStateMachine == null) {
            StatePivotArm pivotArm = new StatePivotArm();
            StateRetractArm retractArm = new StateRetractArm();
            StateExit noop = new StateExit();
            stateMachineRetract = new StateMachine(pivotArm);
            pivotArm.setNextState(retractArm);
            retractArm.setNextState(noop);
            currentStateMachine = stateMachineRetract;
        }
    }

    void setupStatemMachineAutoPark() {
        if (currentStateMachine == null) {
            StateAutoPark autoPark = new StateAutoPark();
            StateExit noop = new StateExit();
            stateMachineAutoPark = new StateMachine(autoPark);
            autoPark.setNextState(noop);
            currentStateMachine = stateMachineAutoPark;
        }
    }

    abstract class BasePathState extends BaseState {
        abstract PathSeg[] getPath();

        @Override
        public void start() {
            helper.startPath(getPath());
        }

        @Override
        public StateMachine.GotoState update() {
            if (helper.pathComplete()) {
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                        helper.mCurrentSeg, helper.mCurrentPath.length,
                        helper.mLeftEncoderTarget, helper.getLeftPosition(),
                        helper.mRightEncoderTarget, helper.getRightPosition()));
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }
    }

    class StateAutoPark extends BasePathState {
        @Override
        PathSeg[] getPath() { return mAutoParkPath; }
    }

    class StateExit extends BaseState {
        @Override
        public StateMachine.GotoState update() {
            currentStateMachine = null;
            return StateMachine.GotoState.CURRENT_STATE;
        }
    }

    class StatePivotArm extends BaseState {
        @Override
        public void start() {
            robot.pivotArm.setPower(0.6);
        }

        @Override
        public StateMachine.GotoState update() {
            if (stateMachineRetract.getStateTime().milliseconds() > 1000) {
                robot.pivotArm.setPower(0.0);
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("pivotArm", String.format("%d",
                        robot.getPivotArm().getCurrentPosition()));
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }
    }

    class StateRetractArm extends BaseState {
        @Override
        public void start() {
            robot.extendArm.setPower(0.6);
        }

        @Override
        public StateMachine.GotoState update() {
            if (stateMachineRetract.getStateTime().milliseconds() > 1000) {
                robot.extendArm.setPower(0.0);
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("extendArm", String.format("%d",
                        robot.getExtendArm().getCurrentPosition()));
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }
    }

    public boolean twoDriverMode() {
        return false;
    }

    void updateStateMachine() {
        if (currentStateMachine != null) {
            currentStateMachine.update();
        }
    }

    void resetEncoders() {
        robot.getLinearSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getPivotArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getExtendArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLinearSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getPivotArm().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getExtendArm().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower((left*1.2*1.2*0.9)/2);
            robot.rightDrive.setPower((right*1.2*1.2*0.9)/2);

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.a)
                robot.linearSlide.setPower(1);
            else if (gamepad1.y )
                robot.linearSlide.setPower(-1);
            else
                robot.linearSlide.setPower(0.0);

            /*
            if (getGamePadTwo().dpad_right) {
                resetEncoders();
            }
            */

            /*
            if (getGamePadTwo().dpad_left) {
                setupStatemMachineRetract();
            }
            */

            // state machine overrides the other buttons
            if (currentStateMachine == null) {
                // pivot arm
                if (getGamePadTwo().x)
                    robot.getPivotArm().setPower(0.9);
                else if (getGamePadTwo().b)
                    robot.getPivotArm().setPower(-0.9);
                else
                    robot.getPivotArm().setPower(0.0);

                //intake
                if (getGamePadTwo().left_bumper) {
                    robot.getIntakeServo().setPosition(1);
                } else if (getGamePadTwo().right_bumper) {
                    robot.getIntakeServo().setPosition(-1);
                } else {
                    robot.getIntakeServo().setPosition(0.5);
                }

                // extend Arm
                if (getGamePadTwo().dpad_up) {
                    robot.extendArm.setPower(1);
                } else if (getGamePadTwo().dpad_down) {
                    robot.extendArm.setPower(-1);
                } else {
                    robot.extendArm.setPower(0);
                }


            }

            updateStateMachine();

            // Send telemetry message to signify robot running;
            telemetry.addData("left:right",  "%.2f:%.2f", left, right);
            telemetry.addData("linearActuator", "%d", robot.getLinearSlide().getCurrentPosition());
            telemetry.addData("pivot", "%d", robot.getPivotArm().getCurrentPosition());
            telemetry.addData("extendArm", "%d", robot.getExtendArm().getCurrentPosition());
            telemetry.update();

        }
    }

    Gamepad getGamePadTwo() {
        if (twoDriverMode()) {
            return gamepad2;
        } else {
            return gamepad1;
        }
    }
}
