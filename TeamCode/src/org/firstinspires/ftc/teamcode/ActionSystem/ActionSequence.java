package org.firstinspires.ftc.teamcode.ActionSystem;

import org.firstinspires.ftc.teamcode.ActionSystem.actions.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

import java.util.ArrayList;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    DriveTrain dt;

    public ActionSequence(Robot robot) {
        dt = robot.driveTrain;
        actionSequence = new ArrayList<>();
    }

    public ActionSequence(ArrayList<Action> actionSequence) {
        this.actionSequence = actionSequence;
    }

    public ArrayList<Action> getActionList() {
        return actionSequence;
    }

    public void addAction(Action action) {
        actionSequence.add(action);
    }

    public void addCustomAction(Runnable runnable) {
        actionSequence.add(new CustomAction(runnable));
    }

    public void addWait(double seconds) {
        actionSequence.add(new Wait(seconds));
    }

}
