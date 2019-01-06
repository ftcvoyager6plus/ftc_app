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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is VoyagerBot
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
public class VoyagerHardware
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  linearSlide = null;
    public DcMotor  pivotArm = null;
    public DcMotor  extendArm = null;

    public Servo    teamMarker    = null;
    public Servo    intakeServo   = null;
    //public ColorSensor colorSensor = null;
    private DistanceSensor sensorRange;


    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public VoyagerHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "lm");
        rightDrive = hwMap.get(DcMotor.class, "rm");
        linearSlide = hwMap.get(DcMotor.class, "am");
        pivotArm = hwMap.get(DcMotor.class, "pivot");
        extendArm = hwMap.get(DcMotor.class, "extend");

        //colorSensor = hwMap.get(ColorSensor.class, "cs");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        linearSlide.setDirection(DcMotor.Direction.FORWARD);
        pivotArm.setDirection(DcMotor.Direction.FORWARD);
        extendArm.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        linearSlide.setPower(0);
        pivotArm.setPower(0);
        extendArm.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        teamMarker  = hwMap.get(Servo.class, "marker");
        intakeServo = hwMap.get(Servo.class, "intake");



    }

    public DcMotor getLeftMotor() {
        return leftDrive;
    }

    public DcMotor getRightMotor() {
        return rightDrive;
    }

    public Servo getTeamMarkerServo() {
        return teamMarker;
    }

    public Servo getIntakeServo() { return intakeServo; }

    public DcMotor getLinearSlide() { return linearSlide; }

    public DcMotor getPivotArm() {return pivotArm; }

    public DcMotor getExtendArm() { return extendArm; }

    public DistanceSensor getDistanceSensor() { return sensorRange;}
}

