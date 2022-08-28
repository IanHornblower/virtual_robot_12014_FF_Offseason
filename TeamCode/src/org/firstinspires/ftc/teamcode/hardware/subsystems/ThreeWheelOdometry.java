package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.*;

public class ThreeWheelOdometry implements Subsystem {
    Encoder left = null;
    Encoder right = null;
    Encoder lateral = null;

    Pose2D position;
    Pose2D veloPos = new Pose2D(0, 0, 0);

    public double accumulatedDistance = 0;

    double xVelo = 0;
    double yVelo = 0;
    double hVelo = 0;
    double previousLeft = 0;
    double previousRight = 0;
    double previousLateral = 0;

    public ThreeWheelOdometry(Encoder left, Encoder right, Encoder lateral) {
        this.left = left;
        this.right = right;
        this.lateral = lateral;
    }

    public void setStartPosition(Pose2D start) {
        position = start;
    }

    private double convertTicks(double ticks) {
        return ticks * (2.0 * Math.PI * wheelRadius / ticksPerRevolution);
    }

    private void correctAngle() {
        if(position.heading > Math.PI * 2.0) {
            position.heading -= 2.0 * Math.PI;
        }
        else if(position.heading < 0) {
            position.heading += 2.0 * Math.PI;
        }
    }

    @Override
    public void init() throws InterruptedException {
        left.setDirection(Encoder.Direction.REVERSE);
        //right.setDirection(Encoder.Direction.REVERSE);
        //lateral.setDirection(Encoder.Direction.REVERSE);
    }

    @Override
    public void update() throws InterruptedException { // Invert Left, Lateral
        double currentLeft = convertTicks(left.getCurrentPosition());
        double currentRight = convertTicks(right.getCurrentPosition());
        double currentLateral = convertTicks(lateral.getCurrentPosition());

        double currentLeftVelo = convertTicks(left.getRawVelocity());
        double currentRightVelo = convertTicks(right.getRawVelocity());
        double currentLateralVelo = convertTicks(lateral.getRawVelocity());

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;
        double dLateral = currentLateral - previousLateral;

        previousLeft = currentLeft;
        previousRight = currentRight;
        previousLateral = currentLateral;

        // Pose
        double dTheta = (dLeft - dRight) / (trackWidth);
        double dx = dLateral + lateralOffset * dTheta;
        double dy = (dLeft + dRight) / 2.0;

        double newAngle = position.heading + dTheta / 2;

        Point deltaPoint = new Point(dx, dy).rotate(newAngle);

        deltaPoint.invertX(); // Not Inverse Field Axes

        position.addPoint(deltaPoint);
        position.heading += dTheta;

        correctAngle();

        // Velocity
        hVelo = (currentLeftVelo - currentRightVelo) / trackWidth;
        yVelo = (currentLeftVelo + currentRightVelo) / 2.0;
        xVelo = currentLateralVelo - lateralOffset * dTheta;

        veloPos = new Pose2D(xVelo, yVelo, hVelo);
    }

    public Pose2D getPose() {
        return position;
    }

    public Pose2D getRawVelocityPos() {
        return veloPos;
    }

    public Pose2D getRotatedVelocityPos() {
        return veloPos.rotate(position.heading);
    }
}
