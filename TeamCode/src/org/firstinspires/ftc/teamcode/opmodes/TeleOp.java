package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.SocketServer;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp()
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException, IOException {

        Robot robot = new Robot(hardwareMap, telemetry);
        robot.initHardwareMap();

        robot.setStartPosition(new Pose2D(0, 0, Math.toRadians(45)));

        Timer timer = new Timer();
        timer.start();

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

        while(opModeIsActive() & !isStopRequested()) {
            Pose2D power = new Pose2D(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

            if(gamepad1.x) {
                timer.pause();
            }
            if(gamepad1.b) {
                timer.resume();
            }

            robot.driveTrain.setWeightedDrivePower(power);
            robot.driveTrain.setWeightedDrivePower(0.5, 1 ,0);

            telemetry.addData("Pose", robot.localizer.getPose());
            telemetry.addData("Get Raw Velocity", robot.localizer.getRawVelocityPos());
            telemetry.addData("Rotated Velocity", robot.localizer.getRotatedVelocityPos());
            telemetry.addData("Elapsed Time in Seconds", timer.currentSeconds());

            telemetry.update();

            robot.update();

            if(isStopRequested()) {
                stop();
            }
        }
    }
}
