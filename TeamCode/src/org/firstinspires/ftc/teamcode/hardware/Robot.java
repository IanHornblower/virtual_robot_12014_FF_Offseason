package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class Robot extends OpMode {
    public DriveTrain driveTrain;
    public IMU imu;
    public ThreeWheelOdometry localizer;

    Subsystem[] subsystems = {};

    // Gamepad 1 & 2

    public Robot (HardwareMap hwMap, Telemetry telemetry) {
        driveTrain = new DriveTrain(hwMap);
        imu = new IMU(hwMap);
        localizer = new ThreeWheelOdometry(driveTrain.deadWheels[0], driveTrain.deadWheels[1], driveTrain.deadWheels[2]);

        subsystems = new Subsystem[] {driveTrain, imu, localizer};
    }

    public void initHardwareMap() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.init();
        }
    }

    public void resetEncoders() {
        driveTrain.resetEncoders();
    }

    public void setStartPosition(Pose2D start) {
        localizer.setStartPosition(start);
        imu.setStartHeading(start.heading);
    }

    public void update() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
