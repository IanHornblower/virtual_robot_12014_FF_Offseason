package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous()
public class ActionTestingAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws Exception {

        Robot robot = new Robot(hardwareMap, telemetry);
        robot.initHardwareMap();

        robot.setStartPosition(new Pose2D(0, 0, Math.toRadians(0)));

        Timer timer = new Timer();
        timer.start();

        ActionSequence as = new ActionSequence(robot);

        //as.addAction(new BasicTurn(robot, Math.toRadians(0)));

        ActionSequenceRunner runner = new ActionSequenceRunner(robot);

        runner.setActionSequence(as);

        while(!opModeIsActive() & !isStopRequested()) {
            telemetry.addLine("Waiting for Start");
            telemetry.addData("Seconds Since Initialization", (int)timer.currentSeconds());
            telemetry.update();

            if(isStopRequested()) {
                stop();
            }
        }

        telemetry.clear();
        waitForStart();
        timer.reset();
        robot.driveTrain.setMotorPowers(1, 1, 0);

        while(opModeIsActive() & !isStopRequested()) {
            //if(!runner.isComplete()) {
            //    runner.update();
            //}

            double angle = 180;

            if(robot.localizer.getRawVelocityPos().y < 28) {
            }
            else {
                timer.pause();
                robot.driveTrain.stopDriveTrain();
                robot.update();
            }

            robot.update();

            telemetry.addData("Sample Duration", Arrays.toString(as.getActionList().get(0).duration));
            telemetry.addData("Current Duration", timer.currentSeconds());

            telemetry.addData("Pose", robot.localizer.getPose());
            telemetry.addData("Get Raw Velocity", robot.localizer.getRawVelocityPos());
            telemetry.addData("Rotated Velocity", robot.localizer.getRotatedVelocityPos());
            telemetry.update();

            if(isStopRequested()) {
                stop();
            }
        }
    }
}
