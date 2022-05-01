package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Odometry;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.function.DoubleSupplier;

public class GyroIntegratedThreeWheelOdometry extends Odometry {

    // Necessary Variables
    double trackWidth, ticksPerRevolution, wheelRadius, lateralOffset;
    inputMeasurement measurement = null;
    DoubleSupplier left = null, right = null, lateral = null;

    Pose2D position = new Pose2D(0, 0, Math.toRadians(0)), startPosition = new Pose2D(0, 0, Math.toRadians(0));
    boolean hasInitRun = false;

    IMU imu;

    public enum inputMeasurement {
        INCH,
        FEET,
        YARD,
        MM,
        CM,
        M,
    }

    public GyroIntegratedThreeWheelOdometry (DriveTrain driveTrain) {
        imu = new IMU(driveTrain.getHardwareMap());
    }

    public void initDoubleSuppliers(DoubleSupplier left, DoubleSupplier right, DoubleSupplier lateral) {
        this.left = left;
        this.right = right;
        this.lateral = lateral;
    }

    public void setConstants(double trackWidth, double ticksPerRevolution, double wheelRadius, double lateralOffset) {
        this.trackWidth = trackWidth;
        this.ticksPerRevolution = ticksPerRevolution;
        this.wheelRadius = wheelRadius;
        this.lateralOffset = lateralOffset;
    }

    public void setMeasurement(inputMeasurement measurement) {
        this.measurement = measurement;
    }

    public void resetEncoders() {

    }

    public void setStartPosition(Pose2D startPosition) {
        this.startPosition = startPosition;

    }

    /**
     * Take ticks converted to inches
     * @param ticks value of unadjusted tick readings
     * @return ticks converted to inches
     */

    private double convertTicksToInches(double ticks) {
        return ticks * (2.0 * Math.PI * wheelRadius / ticksPerRevolution);
    }

    /**
     * Take ticks converted to inches and change it to the specified measurement
     * @param ticks value of unadjusted tick readings
     * @return ticks converted to measurement
     */
    private double convertTicks(double ticks) {
        switch (measurement) {
            case INCH:
                ticks = convertTicksToInches(ticks);
                break;
            case FEET:
                ticks = convertTicksToInches(ticks) / 12.0;
                break;
            case YARD:
                ticks = convertTicksToInches(ticks) / 36.0;
                break;
            case MM:
                ticks = convertTicksToInches(ticks) * 25.4;
                break;
            case CM:
                ticks = convertTicksToInches(ticks) * 2.54;
                break;
            case M:
                ticks = convertTicksToInches(ticks) / 39.37;
                break;
        }
        return ticks;
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
        if(trackWidth == 0.0) {
            throw new InterruptedException("trackWidth has no value");
        }

        if(ticksPerRevolution == 0.0) {
            throw new InterruptedException("ticksPerRevolution has no value");
        }

        if(wheelRadius == 0.0) {
            throw new InterruptedException("wheelRadius has no value");
        }

        if(lateralOffset == 0.0) {
            throw new InterruptedException("Lateral Offset is Zero");
        }

        if(measurement == null) {
            throw new InterruptedException("type must have a value (ie: INCH)");
        }

        if(left == null) {
            throw new InterruptedException("Left must have a double supplying it");
        }

        if(right == null) {
            throw new InterruptedException("Right must have a double supplying it");
        }

        if(lateral == null) {
            throw new InterruptedException("Lateral must have a double supplying it");
        }

        // Set Position to where we start
        position = startPosition;

        // End init
        hasInitRun = true;
    }

    double previousLeft = 0;
    double previousRight = 0;
    double previousLateral = 0;

    @Override
    public void update() throws InterruptedException {
        if(!hasInitRun) {
            throw new InterruptedException("Something is wrong! Make sure to run the init()");
        }
        double currentLeft = convertTicks(left.getAsDouble());
        double currentRight = convertTicks(right.getAsDouble());
        double currentLateral = convertTicks(lateral.getAsDouble());

        double dLeft = currentLeft - previousLeft;
        double dRight = currentRight - previousRight;
        double dLateral = currentLateral - previousLateral;

        previousLeft = currentLeft;
        previousRight = currentRight;
        previousLateral = currentLateral;

        // theta is flipped (leftD - rightD)
        double dTheta = (dRight - dLeft) / (trackWidth);
        double dx = (dLeft + dRight) / 2.0;
        double dy = dLateral - lateralOffset * dTheta;

        //double theta = pos.heading + (dtheta / 2.0);  // Does same thing as pos.heading | Might remove

        Point deltaPosition = new Point(dx, dy).rotate(position.heading);  // Position heading is what needs to be merged

        position.addPoint(deltaPosition);
        position.heading += dTheta;

        correctAngle();
    }

    public Pose2D getPose() {
        return position;
    }
}
