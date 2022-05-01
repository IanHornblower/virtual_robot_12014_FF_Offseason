package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.QOL.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.subsystems.*;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.interfaces.Subsystem;

public class RobotBase extends OpMode implements Robot {

    public DriveTrain driveTrain;
    public Duck duck;

    Subsystem[] subsystems = {};

    public GamepadEx driverGamepad;
    public GamepadEx operatorGamepad;

    public RobotBase (HardwareMap hwMap) {
        driveTrain = new DriveTrain(hwMap);
        duck = new Duck(hwMap);

        subsystems = new Subsystem[] {driveTrain, duck};

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void initHardwareMap() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.init();
        }
    }

    @Override
    public void update() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
    }

    @Override
    public void resetEncoders() {
        driveTrain.localizer.resetEncoders();
        //lift.resetEncoders();
        duck.resetEncoder();
    }

    @Override
    public void emergencyStop() {
        driveTrain.emergencyStop();
    }

    @Override
    public void init() throws Exception {
    }

    @Override
    public void loop() {

    }

}
