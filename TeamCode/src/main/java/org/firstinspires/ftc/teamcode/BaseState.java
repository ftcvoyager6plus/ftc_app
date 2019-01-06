package org.firstinspires.ftc.teamcode;

public abstract class BaseState implements StateMachine.State {
    StateMachine.State nextState;


    public void start() {
    }

    public void exit() {
    }

    public StateMachine.State next() {return nextState;}

    public void setNextState(StateMachine.State next) {
        nextState = next;
    }
}
