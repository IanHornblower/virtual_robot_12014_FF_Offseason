package org.firstinspires.ftc.teamcode.ActionSystem;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

public class ActionSequenceRunner {

    Robot robotBase;

    ArrayList<Action> actionList;

    int currentState = 0;
    boolean isComplete = false;
    boolean hasStartedAction = false;

    Action currentPersistentAction = null;

    public ActionSequenceRunner(Robot robotBase) {
        this.robotBase = robotBase;
    }

    public void setActionArrayList(ArrayList<Action> actionList) {
        this.actionList = actionList;
    }

    public void setActionSequence(ActionSequence actionSequence){
        this.actionList = actionSequence.getActionList();
    }

    public void update() throws Exception {
        if (!hasStartedAction) {
            actionList.get(currentState).startAction();
            hasStartedAction = true;
        }

        if (actionList.get(currentState).isActionComplete() && currentState < actionList.size() - 1) {
            actionList.get(currentState).stopAction();
            currentState += 1;
            if (actionList.get(currentState).isActionPersistent()) {
                currentPersistentAction = actionList.get(currentState);
            }
            hasStartedAction = false;
        } else if (actionList.get(currentState).isActionComplete() && currentState == actionList.size() - 1) {
            actionList.get(currentState).stopAction();
            isComplete = true;
        } else {
            actionList.get(currentState).runAction();
        }

        if (currentPersistentAction != null
                && (!actionList.get(currentState).isActionPersistent()
                || currentState == actionList.size() - 1) && hasStartedAction
                && !currentPersistentAction.isAMultipleAction()) {
            currentPersistentAction.runAction();
        }

        robotBase.update();
    }

    public boolean isComplete() {
        return isComplete;
    }

}
