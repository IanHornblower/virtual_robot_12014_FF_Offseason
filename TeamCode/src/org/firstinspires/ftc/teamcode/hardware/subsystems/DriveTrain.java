package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import javafx.geometry.Pos;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.*;

public class DriveTrain implements Subsystem {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    Encoder left, right, lateral;

    double fl, fr, bl, br;

    public DcMotorEx[] motors;
    public Encoder[] deadWheels;

    HardwareMap hwMap;

    public DriveTrain(HardwareMap hwMap) {
        this.hwMap = hwMap;

        frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        left = new Encoder(hwMap.get(DcMotorEx.class, "enc_left"));
        right = new Encoder(hwMap.get(DcMotorEx.class, "enc_right"));
        lateral = new Encoder(hwMap.get(DcMotorEx.class, "enc_x"));

        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};
        deadWheels = new Encoder[] {left, right, lateral};
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public void resetEncoders() {
        for(DcMotorEx motors:motors) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setWeightedDrivePower(double x, double y, double heading) {
        Pose2D vel = new Pose2D(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(x)
                    + VY_WEIGHT * Math.abs(y)
                    + TURN_WEIGHT * Math.abs(heading);

            vel = new Pose2D(
                    VX_WEIGHT * x,
                    VY_WEIGHT * y,
                    TURN_WEIGHT * heading
            ).div(denom);
        }

        setMotorPowers(vel.x, vel.y, vel.heading);
    }

    public void setWeightedDrivePower(Pose2D pose) {
        setWeightedDrivePower(pose.x, pose.y, pose.heading);
    }

    public void setMotorPowers(double x, double y, double t) {
        double k = (dt_trackWidth + wheelBase) / 2.0;

        double power = Math.hypot(x, y) * Math.sqrt(2.0); // sqrt(2) == 2 * sin/cos(45)
        double angle = Math.atan2(y, x) - Math.PI / 4.0;

        double fl = power * Math.cos(angle) + Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double bl = power * Math.sin(angle) + Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double fr = power * Math.sin(angle) - Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double br = power * Math.cos(angle) - Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);

        setMotorPowers(fl, fr, bl, br);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void setMotorPowers(Vector vector) {
        setMotorPowers(vector.get(0), vector.get(1));
    }

    public void setMotorPowers(double left, double right) {
        setMotorPowers(left, right, left, right);
    }

    public void stopDriveTrain() {
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public void init() throws InterruptedException {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetEncoders();
    }

    @Override
    public void update() throws InterruptedException {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
