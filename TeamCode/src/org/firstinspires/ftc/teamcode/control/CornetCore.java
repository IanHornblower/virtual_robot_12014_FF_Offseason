package org.firstinspires.ftc.teamcode.control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import sun.awt.image.PNGImageDecoder;

public class CornetCore {

    PIDCoefficients xPIDCoef, yPIDCoef, headingPIDCoef;
    BasicPID xPID, yPID, headingPID;
    AngleController headingController;

    public CornetCore(PIDCoefficients xPIDCoef, PIDCoefficients yPIDCoef, PIDCoefficients headingPIDCoef) {
        this.xPIDCoef = xPIDCoef;
        this.yPIDCoef = yPIDCoef;
        this.headingPIDCoef = headingPIDCoef;

        xPID = new BasicPID(xPIDCoef);
        yPID = new BasicPID(yPIDCoef);
        headingPID = new BasicPID(headingPIDCoef);
        headingController = new AngleController(headingPID);
    }

    // TODO: Create Rotate, Run To Pose (Diffy)
    public void runToPosition(DriveTrain dt, double x, double y, double heading) {
        Pose2D pos = dt.localizer.getPose();

        double xP = xPID.calculate(x, pos.x);
        double yP = yPID.calculate(y, pos.y);
        double headingP = headingController.calculate(heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);
    }

    public void rotate(DriveTrain dt, double heading) {
        Pose2D pos = dt.localizer.getPose();

        double headingP = headingController.calculate(heading, pos.heading);
        dt.driveFieldCentric(0, 0, headingP);
    }


}
