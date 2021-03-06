package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.Rotate;
import org.firstinspires.ftc.teamcode.control.*;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Ogga Booga TestTeleoP", group = "no")
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotBase actualRobot = new RobotBase(hardwareMap);
        TrajectoryFollower f = new TrajectoryFollower(actualRobot.driveTrain, 99, 0.5);
        TrajectoryFollower x = new TrajectoryFollower(actualRobot.driveTrain, 99, 0.5);

        actualRobot.initHardwareMap();
        //actualRobot.driveTrain.setStartPosition(new Pose2D(63, 12, Math.toRadians(90)));
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        Trajectory joe = new Trajectory();

        joe.add(new Pose2D(0, 0, Math.toRadians(0)));
        joe.add(new Pose2D(24, 24, Math.toRadians(90)));
        joe.add(new Pose2D(24, 48, Math.toRadians(90)));
        joe.right(24, Trajectory.controlType.ROBOT);

        Trajectory oej = new Trajectory();

        oej.add(new Pose2D(38, 0, 0));
        oej.add(new Pose2D(63, 10, 0));
        oej.add(new Pose2D(63, 40, 0));

        ArrayList<Trajectory> s = new ArrayList<>();
        s.add(joe);
        s.add(oej);

        TrajectorySequence trajS = new TrajectorySequence(s);

        int count = 0;
        boolean start = false;


        waitForStart();

        while (opModeIsActive()) {
            actualRobot.driveTrain.setMotorPowers(actualRobot.driverGamepad.leftJoystick.x(), actualRobot.driverGamepad.leftJoystick.y(), actualRobot.driverGamepad.rightJoystick.x());

            actualRobot.update();


            telemetry.addData("Velocity", actualRobot.driveTrain.getCombinedVelocity());
            telemetry.addLine("Front Left = 0, Front Right = 1, Back Left = 2, Back Right = 3");

            for(DcMotorEx motors : actualRobot.driveTrain.motors) {
                telemetry.addLine("Motor: " + motors.getPower());
            }

            double imuValue = actualRobot.driveTrain.localizer.imu.getHeadingInRadians();

            telemetry.addData("Left Enc", actualRobot.driveTrain.deadWheels[0].getCurrentPosition());
            telemetry.addData("Right Enc", actualRobot.driveTrain.deadWheels[1].getCurrentPosition());
            telemetry.addData("X Enc", actualRobot.driveTrain.deadWheels[2].getCurrentPosition());

            telemetry.addData("IMU", Math.toDegrees(imuValue));
            telemetry.addData("Kalman Heading", Math.toDegrees(actualRobot.driveTrain.localizer.getKalmanHeading()));
            telemetry.addData("Pose Estimate", actualRobot.driveTrain.localizer.getPose().toString());
            telemetry.update();
        }



    }

}
