package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class StateMachine {

    State currentState = null;
    State initialState = null;
    ElapsedTime mStateTime = new ElapsedTime();

    public enum GotoState {
        CURRENT_STATE,
        NEXT_STATE
    }

    public interface State {

        public void start();

        public GotoState update();

        public void exit();

        public State next();
    }

    public StateMachine(State state) {
        initialState = state;
    }

    public void update() {
        if (currentState == null) {
            currentState = initialState;
            mStateTime.reset();
            currentState.start();
        }
        GotoState gotoState = currentState.update();
        switch (gotoState) {
            case NEXT_STATE:
                currentState.exit();
                State nextState = currentState.next();
                mStateTime.reset();
                nextState.start();
                currentState = nextState;
                break;
            case CURRENT_STATE:
                break;
        }
    }

    public ElapsedTime getStateTime() {
        return mStateTime;
    }
}