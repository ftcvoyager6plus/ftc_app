package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public abstract class BaseOpMode extends OpMode {
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // Gear
    static final double WHEEL_DIAMETER_INCHES = 3.54; // Diameter of wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public ElapsedTime mRuntimeTime = new ElapsedTime();  // Time overall

    int mLeftEncoderTarget = 0;
    int mRightEncoderTarget = 0;
    int mLinearSlideTarget = 0;
    PathSeg[] mCurrentPath;      // current Path
    int mCurrentSeg;
    public VoyagerHardware robot;

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

    void startPath(PathSeg[] path) {
        mCurrentPath = path;
        mCurrentSeg = 0;
        syncEncoders();
        runToPosition();
        startSeg();
    }

    void startSeg() {
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

    boolean pathComplete() {
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
