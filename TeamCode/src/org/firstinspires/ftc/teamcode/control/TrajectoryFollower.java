package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.Arrays;

public class TrajectoryFollower extends LinearOpMode {

    DriveTrain dt;
    double timeout;
    double distanceTolerance;

    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    Trajectory currentTrajectory;

    Timer timer = new Timer();

    boolean init = false;
    boolean isCompleted = false;
    int closestPointIndex = 0;
    double runningCount = 1e+6;

    public enum STATE {START, RUNNING, COMPLETED, STOPPED}
    STATE state;

    public TrajectoryFollower(DriveTrain dt, double timeout, double distanceTolerance) {
        this.dt = dt;
        this.timeout = timeout;
        this.distanceTolerance = distanceTolerance;

        state = STATE.START;
    }

    public boolean isCompleted() {
        return isCompleted;
    }

    public Trajectory currentTrajectory() {
        return currentTrajectory;
    }

    public void setTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        state = STATE.START;
        isCompleted = false;
        timer.start();
    }

    public void setState(STATE s) {
        state = s;
    }

    public void HolonomicFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                dt.runToPosition(pointToFollow.x, pointToFollow.y, path.get(closestPointIndex).heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // TODO: Change to after point change, current system does it half way (closest System)

                for(int i = 0; i < path.size(); i++) {
                    double distanceTo = dt.localizer.getPose().getDistanceFrom(path.get(i));

                    if(distanceTo < runningCount) {
                        closestPointIndex = i;
                        runningCount = distanceTo;
                    }
                }

                runningCount = 1e+6;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    // TODO: add ability to back trace it so it doesn't double back
    public void AnglePursuitFollower(double radius) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                double angle = Math.toRadians(270)-Curve.getAngle(pointToFollow, dt.localizer.getPose().toPoint());

                dt.runToPosition(pointToFollow.x, pointToFollow.y, angle);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    public void SetHeadingFollower(double radius, double heading) throws InterruptedException {
        switch (state) {
            case START:
                currentTrajectory.setStart();
                path = currentTrajectory.getPath();
                extendedPath = PurePursuit.extendPath(path, radius);

                lastIndex = path.size()-1;
                lastPoint = path.get(lastIndex);
                state = STATE.RUNNING;
                break;
            case RUNNING:
                pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

                dt.runToPosition(pointToFollow.x, pointToFollow.y, heading);

                double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

                // Trajectory is Finished if the timeout has been reached, or we are within our distance tolerance
                if((timer.currentSeconds() > timeout) || (error < distanceTolerance)) state = STATE.COMPLETED;
                break;
            case COMPLETED:
                currentTrajectory.complete();
                isCompleted = true;
                state = STATE.STOPPED;
                break;
            case STOPPED:
                dt.stopDriveTrain();
                break;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
