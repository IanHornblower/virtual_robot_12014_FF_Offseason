package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class DriveConstants {

    public static int EncoderTicks = 1120;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double TURN_WEIGHT = 3;

    public static PIDCoefficients xPID = new PIDCoefficients(0.1, 0 ,0);
    public static PIDCoefficients yPID = new PIDCoefficients(0.1, 0 ,0);
    public static PIDCoefficients headingPID = new PIDCoefficients(1.3, 0 ,0);

    public static PIDCoefficients trajectoryPID = new PIDCoefficients(0.07, 0 ,0);

    public static PIDCoefficients encoderHeadingPID = new PIDCoefficients(2, 0 ,0);

    public static PIDCoefficients forwardPID = new PIDCoefficients(0.06, 0 ,0);
    public static PIDCoefficients turnPID = new PIDCoefficients(2, 0 ,0);

    public static PIDCoefficients forwardBackwardPID = new PIDCoefficients(0.002, 0, 0);

    public static double rotationTolerance = Math.toRadians(0.5);
    public static double distanceTolerance = 0.5; // Inches
}
