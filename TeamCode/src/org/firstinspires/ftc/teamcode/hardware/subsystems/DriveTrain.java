package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.function.DoubleSupplier;

public class DriveTrain implements Subsystem {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx left, right, lateral;

    double fl, fr, bl, br;

    DcMotorEx[] motors;
    DcMotorEx[] deadWheels;

    HardwareMap hwMap;

    public GyroIntegratedThreeWheelOdometry localizer;

    public DriveTrain(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        left = hwMap.get(DcMotorEx.class, "enc_left");
        right = hwMap.get(DcMotorEx.class, "enc_right");
        lateral = hwMap.get(DcMotorEx.class, "enc_x");

        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};
        deadWheels = new DcMotorEx[] {left, right, lateral};

        localizer = new GyroIntegratedThreeWheelOdometry(this);

        this.hwMap = hwMap;

    }

    public void setMotorPowers(double x, double y, double turn) {
        double h = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - Math.toRadians(45);

        double[] motorVector = new double[] {
                h * Math.cos(theta) + turn,
                h * Math.sin(theta) - turn,
                h * Math.sin(theta) + turn,
                h * Math.cos(theta) - turn
        };

        setMotorPowers(motorVector[0], motorVector[1],
                       motorVector[2], motorVector[3]);
    }

    public void calculatePosition(double dr, double theta, double turn) {
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        double mx = cos*dr;
        double my = sin*dr;

        setMotorPowers(mx, my, turn);
    }

    public void driveFieldCentric(double x, double y, double turn) {
        Point vector = new Point(x, y);

        calculatePosition(
                vector.hypot(),
                vector.atan2() - localizer.imu.getHeadingInRadians(),
                turn);
    }

    public void driveFieldCentric(double x, double y, double turn, double modHeading) {
        Point vector = new Point(x, y);

        calculatePosition(
                vector.hypot(),
                vector.atan2() - localizer.imu.getHeadingInRadians() + modHeading,
                turn);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void emergencyStop() {
        for(DcMotorEx m : motors) {
            m.setPower(0.0);
            m.setMotorDisable();
        }
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public void setStartPosition(Pose2D pose) {
        localizer.setStartPosition(pose);
    }

    @Override
    public void init() throws InterruptedException {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        localizer.initDoubleSuppliers(
                ()-> left.getCurrentPosition(),
                ()-> right.getCurrentPosition(),
                ()-> lateral.getCurrentPosition()
        );

        localizer.setConstants(12, 1120, 1, -6);
        localizer.setMeasurement(GyroIntegratedThreeWheelOdometry.inputMeasurement.INCH);

        localizer.init();
    }

    @Override
    public void update() throws InterruptedException {
        localizer.update();

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
