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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;


@Autonomous(name="CraterDon'tUseThis", group="VoyagerBot")

public class StateMachineExample extends OpMode {
    private GoldAlignDetector detector;

    private enum State {
        STATE_INITIAL,
        STATE_LOWER_LIFT,
        STATE_UNLOCK_FROM_LIFT,
        STATE_WIGGLE_FROM_LIFT,
        STATE_AVOID_STUCK,
        STATE_DETECT_GOLD,
        STATE_DRIVE_TO_GOLD,
        STATE_DRIVE_TO_WALL,
        STATE_ALIGN_TO_WALL,
        STATE_DRIVE_TO_ZONE,
        STATE_DEPLOY_TEAM_MARKER,
        STATE_DRIVE_TO_CRATER,
        WAIT_FOR_DEPLOY,
        STATE_STOP
    }

    public enum GoldPosition {
        NONE,
        ONE,
        TWO,
        THREE
    }

    final PathSeg[] mWallPath1 = {
            new PathSeg(3.4, -3.4, 0.3),
            new PathSeg(-1.3, 1.3, 0.3),
            new PathSeg(3.4, -3.4, 0.3),
            new PathSeg(-23.0, -23.0, 0.3),
            new PathSeg(7, -7, 0.3),
            new PathSeg(-50, -50, 0.3)
    };

    final PathSeg[] mWallPath2 = {
            //new PathSeg(13.0, -13, 0.4),
            new PathSeg(-25.0, -20.0, 0.4)
    };

    final PathSeg[] mWallPath3 = {
            //new PathSeg(13.0, -13, 0.4),
            new PathSeg(-4, 4, 0.4),
            new PathSeg(-30.0, -30.0, 0.4),
            new PathSeg(8.0, 8.0, 0.4)
    };

    final PathSeg[] mGoldPath1 = {
            new PathSeg(1.3, -1.3, 0.4),
            new PathSeg(-30, -30, 0.4),
            //new PathSeg(4.0, 4.0, 0.4)
    };

    final PathSeg[] mGoldPath2 = {
            new PathSeg(-20.0, -20.0, 0.4),
            //new PathSeg(6.0, 6.0, 0.4)
    };

    final PathSeg[] mGoldPath3 = {
            new PathSeg(-0.5, 0.5, 0.4),
            new PathSeg(-30, -30, 0.4),

    };

    final PathSeg[] mCraterPath = {
            new PathSeg(0.0, 3.0, 0.2),
            new PathSeg(0.0, 3.0, 0.2)
    };

    final PathSeg[] mWigglePath = {
            new PathSeg(-4.0, 4.0, 0.2),
            new PathSeg(4, -4, 0.2)
    };

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // Gear
    static final double WHEEL_DIAMETER_INCHES = 3.54; // Diameter of wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double WALL_MIN_DISTANCE = 3.0;   // minimal distance to the wall to consider square
    static final double WALL_THRESHOLD = 0;
    public ElapsedTime mRuntimeTime = new ElapsedTime();  // Time overall
    public ElapsedTime mStateTime = new ElapsedTime();    // Time in state

    private int mLeftEncoderTarget = 0;
    private int mRightEncoderTarget = 0;
    private int mLinearSlideTarget = 0;

    private State mCurrentState;         // current state
    private PathSeg[] mCurrentPath;      // current Path
    private int mCurrentSeg;
    public VoyagerHardware robot;
    private int mDetectionTime;
    public GoldPosition goldPosition;

    PathSeg[] getWigglePath() {
        return mWigglePath;
    }

    PathSeg[] getWallPath() {
        if (goldPosition == GoldPosition.ONE) {
            return(mWallPath1);
        } else if (goldPosition == GoldPosition.TWO) {
            return(mWallPath2);
        } else {
            return(mWallPath3);
        }
    }

    PathSeg[] getGoldPath() {
        if (goldPosition == GoldPosition.ONE) {
            return(mGoldPath1);
        } else if (goldPosition == GoldPosition.TWO) {
            return(mGoldPath2);
        } else {
            return(mGoldPath3);
        }
    }

    @Override
    public void init() {
        robot = new VoyagerHardware();
        robot.init(hardwareMap);
        configureDetector();
        setDrivePower(0, 0);
        resetDriveEncoder();
    }

    // Configure the Gold detector using OpenCV
    void configureDetector() {
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    @Override
    public void init_loop() {
        resetDriveEncoder();
        telemetry.addData("ENC", String.format("L:R %d:%d", getLeftPosition(), getRightPosition()));
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        setDriveSpeed(0, 0);
        runToPosition();
        mRuntimeTime.reset();
        newState(State.STATE_INITIAL);
    }


    @Override
    public void loop() {
        double distance;

        telemetry.addData("state ", mCurrentState.toString());
        telemetry.addData("0", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("dectectionTime", String.format("%d", mDetectionTime));
        telemetry.addData("Gold position", goldPosition);
        switch (mCurrentState) {
            case STATE_INITIAL:
                if (encoderAtZero()) {
                    useConstantPower();
                    setDriveSpeed(-0.2, 0.2);  // start rotating left to find gold
                    newState(State.STATE_DETECT_GOLD);
                } else {
                    // Display Telemetry Data
                    telemetry.addData("left:", getLeftPosition()); // Is the bot aligned with the gold mineral
                    telemetry.addData("right:", getRightPosition()); // Gold X pos.
                }
                break;

            case STATE_LOWER_LIFT:
                mLinearSlideTarget = 2600;
                robot.getLinearSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.getLinearSlide().setPower(-0.6);
                newState(State.STATE_UNLOCK_FROM_LIFT);
                break;

            case STATE_UNLOCK_FROM_LIFT:
                if (mStateTime.time() >5.4) {
                    //newState(State.STATE_DETECT_GOLD);
                    //robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.linearSlide.setPower(0.0);
                    newState(State.STATE_WIGGLE_FROM_LIFT);
                    startPath(getWigglePath());
                } else {
                    telemetry.addData("linearSlide", String.format("%d:%d",
                            mLinearSlideTarget, robot.getLinearSlide().getCurrentPosition()));
                }
                break;

            case STATE_WIGGLE_FROM_LIFT:
                if (pathComplete()) {
                    robot.linearSlide.setPower(0.6);
                    newState(State.STATE_AVOID_STUCK);

                                   // start rotating left to find gold
                    //newState(State.STATE_DETECT_GOLD);
                } else {
                    telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            case STATE_AVOID_STUCK:
                if (mStateTime.time() > 5.3)  {
                    useConstantPower();
                    robot.linearSlide.setPower(0);
                    setDriveSpeed(-0.3, 0.2);
                    newState(State.STATE_DETECT_GOLD);
                }

                break;



                case STATE_DETECT_GOLD:
                if (detector.isFound() && detector.getAligned() && mStateTime.time()>1) {

                    mDetectionTime = (int) mStateTime.milliseconds();
                    if (mDetectionTime < 2500) {
                        // gold position 1
                        goldPosition = GoldPosition.ONE;
                    } else if (mDetectionTime < 4300) {
                        // gold position 2
                        goldPosition = GoldPosition.TWO;
                    } else {
                        goldPosition = GoldPosition.THREE;
                    }
                    startPath((getGoldPath()));
                    newState(State.STATE_DRIVE_TO_GOLD);
                } else {
                    telemetry.addData("isFound", detector.isFound()); // Is the gold mineral detected
                    telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
                    telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
                }
                break;

            case STATE_DRIVE_TO_GOLD:
                if (pathComplete()) {
                    startPath(getWallPath());
                    newState(State.STATE_DRIVE_TO_WALL);
                } else {
                    telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            case STATE_DRIVE_TO_WALL:
                if (pathComplete()) {
                    useConstantPower();
                    setDriveSpeed(0, 0);
                    newState(State.STATE_DEPLOY_TEAM_MARKER);
                    // set servo to deploy team marker
                } else {
                    telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            /*case STATE_ALIGN_TO_WALL:
                distance = robot.getDistanceSensor().getDistance(DistanceUnit.CM);
                if (distance < WALL_MIN_DISTANCE) {
                    newState(State.STATE_DRIVE_TO_ZONE);
                    //newState(State.STATE_DEPLOY_TEAM_MARKER);
                    // set servo to deploy team marker
                } else {
                    // steer left or right based on distance
                    if (distance < WALL_THRESHOLD) {
                        setDriveSpeed(0.2, 0.0);
                    } else {
                        setDriveSpeed(0.0, 0.2);
                    }
                    telemetry.addData("Distance", String.format("%f",
                            distance));
                }
                break;*/

            case STATE_DRIVE_TO_ZONE:
                if (pathComplete()) {
                    newState(State.STATE_STOP);
                    //newState(State.STATE_DEPLOY_TEAM_MARKER);
                    // set servo to deploy team marker
                } else {
                    telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                            mCurrentSeg, mCurrentPath.length,
                            mLeftEncoderTarget, getLeftPosition(),
                            mRightEncoderTarget, getRightPosition()));
                }
                break;

            case STATE_DEPLOY_TEAM_MARKER:
                if (mStateTime.time() > 0.5) {
                    newState(State.WAIT_FOR_DEPLOY);
                    robot.getTeamMarkerServo().setPosition(0.6);
                } else {

                }
                break;
            case WAIT_FOR_DEPLOY:
                if (mStateTime.time() > 1) {
                    useConstantPower();
                    setDrivePower(0, 0);
                    newState(State.STATE_STOP);
                }
                break;
            case STATE_DRIVE_TO_CRATER:
                break;

            case STATE_STOP:
                useConstantPower();
                setDrivePower(0, 0);
                break;
        }
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        detector.disable();
        useConstantPower();
        setDrivePower(0, 0);
    }

    void newState(State newState) {
        mStateTime.reset();
        mCurrentState = newState;
    }

    void setEncoderTarget(int leftEncoder, int rightEncoder) {
        robot.getLeftMotor().setTargetPosition(mLeftEncoderTarget = leftEncoder);
        robot.getRightMotor().setTargetPosition(mRightEncoderTarget = rightEncoder);
    }

    void addEncoderTarget(int leftEncoder, int rightEncoder) {
        robot.getLeftMotor().setTargetPosition(mLeftEncoderTarget += leftEncoder);
        robot.getRightMotor().setTargetPosition(mRightEncoderTarget += rightEncoder);
    }

    void setDrivePower(double leftPower, double rightPower) {
        robot.getLeftMotor().setPower(Range.clip(leftPower, -1, 1));
        robot.getRightMotor().setPower(Range.clip(rightPower, -1, 1));
    }

    void setDriveSpeed(double leftSpeed, double rightSpeed) {
        setDrivePower(leftSpeed, rightSpeed);
    }

    public void runToPosition() {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void useConstantPower() {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void useConstantSpeed() {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveEncoder() {
        setEncoderTarget(0, 0);
        setDriveMode(DcMotor.RunMode.RESET_ENCODERS);
    }

    public void syncEncoders() {
        mLeftEncoderTarget = robot.getLeftMotor().getCurrentPosition();
        mRightEncoderTarget = robot.getRightMotor().getCurrentPosition();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        if (robot.getLeftMotor().getMode() != mode) {
            robot.getLeftMotor().setMode(mode);
        }
        if (robot.getRightMotor().getMode() != mode) {
            robot.getRightMotor().setMode(mode);
        }
    }

    int getLeftPosition() {
        return robot.getLeftMotor().getCurrentPosition();
    }

    int getRightPosition() {
        return robot.getRightMotor().getCurrentPosition();
    }

    boolean moveComplete() {
        //return (robot.getLeftMotor().isBusy() && robot.getRightMotor().isBusy());
        return ((Math.abs(getLeftPosition() - mLeftEncoderTarget) < 10) &&
                (Math.abs(getRightPosition() - mRightEncoderTarget) < 10));
    }

    boolean linearSlideMoveComplete() {
        return ((Math.abs(robot.getLinearSlide().getCurrentPosition() - mLinearSlideTarget) < 10));
    }

    boolean encoderAtZero() {
        return ((Math.abs(getLeftPosition())) < 5) && ((Math.abs(getRightPosition())) < 5);
    }

    private void startPath(PathSeg[] path) {
        mCurrentPath = path;
        mCurrentSeg = 0;
        syncEncoders();
        runToPosition();
        startSeg();
    }

    private void startSeg() {
        int left;
        int right;

        if (mCurrentPath != null) {
            left = (int) (mCurrentPath[mCurrentSeg].mLeft * COUNTS_PER_INCH);
            right = (int) (mCurrentPath[mCurrentSeg].mRight * COUNTS_PER_INCH);
            addEncoderTarget(left, right);
            setDriveSpeed(mCurrentPath[mCurrentSeg].mSpeed, mCurrentPath[mCurrentSeg].mSpeed);

            mCurrentSeg++;
        }
    }

    private boolean pathComplete() {
        if (moveComplete()) {
            if (mCurrentSeg < mCurrentPath.length) {
                startSeg();
            } else {
                mCurrentPath = null;
                mCurrentSeg = 0;
                setDriveSpeed(0, 0);
                useConstantSpeed ();
                return true;
            }
        }
        return false;
    }


}


