package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BaseStateMachineOpMode extends BaseOpMode {
    private GoldAlignDetector detector;
    private StateMachine stateMachine;
    private double mDetectionTime;
    public StateMachineExample.GoldPosition goldPosition;

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

    PathSeg[] getWallPath() {
        if (goldPosition == StateMachineExample.GoldPosition.ONE) {
            return(mWallPath1);
        } else if (goldPosition == StateMachineExample.GoldPosition.TWO) {
            return(mWallPath2);
        } else {
            return(mWallPath3);
        }
    }

    PathSeg[] getGoldPath() {
        if (goldPosition == StateMachineExample.GoldPosition.ONE) {
            return(mGoldPath1);
        } else if (goldPosition == StateMachineExample.GoldPosition.TWO) {
            return(mGoldPath2);
        } else {
            return(mGoldPath3);
        }
    }

    PathSeg[] getWigglePath() {
        return mWigglePath;
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
        mRuntimeTime.reset();
        initStateMachine();
    }

    @Override
    public void loop() {

        telemetry.addData("state ", stateMachine.currentState.toString());
        telemetry.addData("0", String.format("%4.1f ", stateMachine.getStateTime().time()));
        telemetry.addData("dectectionTime", String.format("%d", mDetectionTime));
        telemetry.addData("Gold position", goldPosition);
        stateMachine.update();
    }


    void initStateMachine() {
        StateInitial initialState = new StateInitial();
        StateDetectGold detectGold = new StateDetectGold();
        StateDriveToGold driveToGold = new StateDriveToGold();
        StateDriveToWall driveToWall = new StateDriveToWall();
        StateLowerLift lowerLift = new StateLowerLift();
        StateWiggleFromLift wiggleLift = new StateWiggleFromLift();
        StateStop stop = new StateStop();

        initialState.setNextState(detectGold);
        detectGold.setNextState(driveToGold);
        driveToGold.setNextState(driveToWall);
        driveToWall.setNextState(stop);

        //lowerLift.setNextState(wiggleLift);
        stateMachine = new StateMachine(initialState);
    }

    class StateInitial extends BaseState {
        @Override
        public void start() {
            setDriveSpeed(0, 0);
            runToPosition();
        }

        @Override
        public StateMachine.GotoState update() {
            if (encoderAtZero()) {
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                // Display Telemetry Data
                telemetry.addData("left:", getLeftPosition()); // Is the bot aligned with the gold mineral
                telemetry.addData("right:", getRightPosition()); // Gold X pos.
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }

    }

    class StateDetectGold extends BaseState {

        @Override
        public void start() {
            useConstantPower();
            setDriveSpeed(-0.2, 0.2);  // start rotating left to find gold
        }

        @Override
        public StateMachine.GotoState update() {
            if (detector.isFound() && detector.getAligned() && stateMachine.getStateTime().time()>1) {
                mDetectionTime = (int) stateMachine.getStateTime().milliseconds();
                if (mDetectionTime < 2500) {
                    // gold position 1
                    goldPosition = StateMachineExample.GoldPosition.ONE;
                } else if (mDetectionTime < 4300) {
                    // gold position 2
                    goldPosition = StateMachineExample.GoldPosition.TWO;
                } else {
                    goldPosition = StateMachineExample.GoldPosition.THREE;
                }
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("isFound", detector.isFound()); // Is the gold mineral detected
                telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral
                telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }

    }

    abstract class BasePathState extends BaseState {
        abstract PathSeg[] getPath();

        @Override
        public void start() {
            startPath(getPath());
        }

        @Override
        public StateMachine.GotoState update() {
            if (pathComplete()) {
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("1", String.format("%d %d L %5d:%5d - R %5d:5d",
                        mCurrentSeg, mCurrentPath.length,
                        mLeftEncoderTarget, getLeftPosition(),
                        mRightEncoderTarget, getRightPosition()));
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }
    }


    class StateDriveToGold extends BasePathState {
        @Override
        PathSeg[] getPath() { return getGoldPath(); }
    }

    class StateDriveToWall extends BasePathState {
        @Override
        PathSeg[] getPath() { return getWallPath(); }
    }

    class StateStop extends BaseState {
        @Override
        public void start() {
            useConstantPower();
            setDrivePower(0, 0);
        }

        @Override
        public StateMachine.GotoState update() {
            return StateMachine.GotoState.CURRENT_STATE;
        }

        @Override
        public StateMachine.State next() {
            return null;
        }
    }

    class StateLowerLift extends BaseState {
        @Override
        public void start() {
            mLinearSlideTarget = 2600;
            robot.getLinearSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.getLinearSlide().setPower(-0.6);
        }

        @Override
        public StateMachine.GotoState update() {
            if (stateMachine.getStateTime().time() >5.4) {
                robot.linearSlide.setPower(0.0);
                return StateMachine.GotoState.NEXT_STATE;
            } else {
                telemetry.addData("linearSlide", String.format("%d:%d",
                        mLinearSlideTarget, robot.getLinearSlide().getCurrentPosition()));
                return StateMachine.GotoState.CURRENT_STATE;
            }
        }
    }

    class StateWiggleFromLift extends BasePathState {
        PathSeg[] getPath() {return getWigglePath();}
    }

}
