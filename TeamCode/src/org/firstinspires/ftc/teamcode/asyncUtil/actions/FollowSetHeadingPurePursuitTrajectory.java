package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;
import java.util.Base64;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.distanceTolerance;

public class FollowSetHeadingPurePursuitTrajectory extends Action {

    // Action Stuff
    DriveTrain dt;
    Trajectory trajectory;
    double heading;
    double radius;

    // Follower Stuff
    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    boolean reversed = false;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    BasicPID xPID = new BasicPID(DriveConstants.xPID);
    BasicPID yPID = new BasicPID(DriveConstants.yPID);
    BasicPID headingPID = new BasicPID(DriveConstants.headingPID);
    AngleController headingController = new AngleController(headingPID);
    BasicPID distanceController = new BasicPID(DriveConstants.trajectoryPID);

    public FollowSetHeadingPurePursuitTrajectory(DriveTrain dt, Trajectory trajectory, double radius, double heading) {
        this.dt = dt;
        this.trajectory = trajectory;
        this.radius = radius;
        this.heading = heading;
    }

    @Override
    public void startAction() {
        trajectory.setStart();
        path = trajectory.getPath();
        extendedPath = PurePursuit.extendPath(path, radius);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);
    }

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = dt.localizer.getPose();
        pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, dt, radius);

        double error = Math.abs(dt.localizer.getPose().getDistanceFrom(lastPoint)) - radius;

        double xP = xPID.calculate(pointToFollow.x, pos.x);
        double yP = yPID.calculate(pointToFollow.y, pos.y);

        double headingP = headingController.calculate(heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);

        if(error < distanceTolerance) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
