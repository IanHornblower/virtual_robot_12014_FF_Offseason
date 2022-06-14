package org.firstinspires.ftc.teamcode.asyncUtil.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.rotationTolerance;

public class IMURotate extends Action {

    DriveTrain dt;
    double heading;

    BasicPID headingPID;
    AngleController headingController;

    public IMURotate(DriveTrain dt, double heading) {
        this.dt = dt;
        this.heading = heading;

        headingPID = new BasicPID(DriveConstants.headingPID);
        headingController = new AngleController(headingPID);

    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = dt.localizer.getPose();

        double headingP = headingController.calculate(heading, pos.heading);
        dt.driveFieldCentric(0, 0, headingP);

        error = Curve.getShortestDistance(heading, dt.localizer.imu.getHeadingInRadians());
        isComplete = error < rotationTolerance;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }



}
