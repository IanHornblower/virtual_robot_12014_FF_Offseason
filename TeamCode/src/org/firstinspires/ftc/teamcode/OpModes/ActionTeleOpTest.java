package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.Action;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.FollowPurePursuitTrajectory;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.Rotate;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.RunToPosition;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

@TeleOp(name = "Action Testing", group = "no")
public class ActionTeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        RobotBase actualRobot = new RobotBase(hardwareMap);
        ActionSequenceRunner runner = new ActionSequenceRunner(actualRobot);

        actualRobot.initHardwareMap();
        //actualRobot.driveTrain.setStartPosition(new Pose2D(63, 12, Math.toRadians(90)));
        actualRobot.driveTrain.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        Trajectory test = new Trajectory();
        test.add(new Pose2D(0, 0, 0));
        test.add(new Pose2D(24, 24, 0));
        test.add(new Pose2D(24, 48, 0));

        ArrayList<Action> actionList = new ArrayList<>();

        actionList.add(new FollowPurePursuitTrajectory(actualRobot.driveTrain, test, 8));
        //actionList.add(new Rotate(actualRobot.driveTrain, Math.toRadians(90)));
        actionList.add(new RunToPosition(actualRobot.driveTrain, new Pose2D(24, 24, Math.toRadians(0))));

        runner.setActionSequence(actionList);

        waitForStart();

        while (opModeIsActive()) {

            // it works :)

            if(!runner.isComplete()) {
                runner.update();
            }
            else {
                stop();
            }

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
