package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Pose2D;

@TeleOp(name = "Ogga Booga TestTeleoP", group = "no")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RobotBase actualRobot = new RobotBase(hardwareMap);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        waitForStart();

        // TODO: Test direction of Encoders and start IMU Fusing
        // TODO: When new ODOM is working use PP for Point following then use Linear Interp for Heading (Create new Trajectory Follower)

        while (opModeIsActive()) {
            actualRobot.update();



            actualRobot.driveTrain.setMotorPowers(1, 1, 0);

            telemetry.addData("Pose Estimate", actualRobot.driveTrain.localizer.getPose().toString());
            telemetry.update();
        }

    }
}
