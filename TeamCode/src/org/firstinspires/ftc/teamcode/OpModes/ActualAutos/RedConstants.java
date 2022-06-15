package org.firstinspires.ftc.teamcode.OpModes.ActualAutos;

import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequence;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.FollowHolonomicPurePursuitTrajectory;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class RedConstants {

    public static ActionSequence Warehouse(RobotBase robot) {

        Trajectory travelToTeamHub = new Trajectory();
        travelToTeamHub.add(new Pose2D(63, 12, Math.toRadians(90)));
        travelToTeamHub.add(new Pose2D(58, 6, Math.toRadians(30)));
        travelToTeamHub.add(new Pose2D(50, -10, Math.toRadians(-3)));

        Trajectory outerWarehouseTrip1 = new Trajectory();
        outerWarehouseTrip1.add(travelToTeamHub.end());
        outerWarehouseTrip1.add(new Pose2D(62, 10, Math.toRadians(0)));
        outerWarehouseTrip1.add(new Pose2D(62, 18, Math.toRadians(0)));
        outerWarehouseTrip1.add(new Pose2D(62, 35, Math.toRadians(0)));

        ActionSequence arr = new ActionSequence(robot);
        arr.addFollowHolonomicPurePursuitTrajectory(travelToTeamHub, 8);
        arr.addWait(0.3);
        arr.addFollowPurePursuitTrajectory(outerWarehouseTrip1, 10);
        //slow forward

        return arr;
    }





}
