package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Configurables {

    //Telemetry
    public static boolean showPoseTelemetry = false;
    public static boolean showEncoderTelemetry = false;
    public static boolean showMotorTelemetry = false;
    public static boolean showDebugTelemetry = false;
    public static boolean showIntakeTelemetry = false;

    //Simulation
    public static boolean doSimulation = false; // was true

    // shooter values

    public static double shooterKp = 0.01;
    public static double shooterKi = 0;
    public static double shooterKd = 0;

    public static double shooterKf = .95;

    public static double shooterKs = 0;
    public static double shooterKv = 0.2;
    public static double shooterKa = 0;
    public static boolean changeVelocityCoefficents = false;
    public static boolean changeFeedForwardCoefficents = false;


}
