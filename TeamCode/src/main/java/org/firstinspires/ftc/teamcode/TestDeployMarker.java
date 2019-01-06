package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestDeployMarker", group="VoyagerBot")
public class TestDeployMarker extends BaseStateMachineOpMode {

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

        initialState.setNextState(deployTeamMarker);
        deployTeamMarker.setNextState(stop);

        stateMachine = new StateMachine(initialState);
    }
}
