package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestGoldDetection", group="VoyagerBot")
public class DiagnosticStateMachine extends BaseStateMachineOpMode {

    private boolean testGoldDetection = true;
    void initStateMachine() {
        StateInitial initialState = new StateInitial();
        StateDetectGold detectGold = new StateDetectGold();
        StateDriveToGold driveToGold = new StateDriveToGold();
        StateDriveToWall driveToWall = new StateDriveToWall();
        StateLowerLift lowerLift = new StateLowerLift();
        StateWiggleFromLift wiggleLift = new StateWiggleFromLift();
        StateDeployTeamMarker deployTeamMarker = new StateDeployTeamMarker();
        StateAvoidStuck avoidStuck = new StateAvoidStuck();
        StateResetLift resetLift = new StateResetLift();
        StateStop stop = new StateStop();

        initialState.setNextState(lowerLift);
        lowerLift.setNextState(wiggleLift);
        wiggleLift.setNextState(avoidStuck);
        avoidStuck.setNextState(detectGold);
        detectGold.setNextState(stop);

        stateMachine = new StateMachine(initialState);
    }
}
