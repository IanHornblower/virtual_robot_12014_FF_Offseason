package org.firstinspires.ftc.teamcode.OpModes.ActualAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequence;
import org.firstinspires.ftc.teamcode.asyncUtil.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.asyncUtil.actions.FollowHolonomicPurePursuitTrajectory;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.math.Pose2D;

@Autonomous(name = "Red Cycle", group = "Actual")
public class RedSideCycleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotBase actualRobot = new RobotBase(hardwareMap);
        ActionSequenceRunner runner = new ActionSequenceRunner(actualRobot);

        actualRobot.initHardwareMap();
        actualRobot.driveTrain.setStartPosition(new Pose2D(63, 12, Math.toRadians(90)));

        runner.setActionSequence(RedConstants.Warehouse(actualRobot));

        waitForStart();

        while (opModeIsActive()) {
            // it works :)

            if(!runner.isComplete()) {
                runner.update();
            }
            else {
                stop();
            }




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
