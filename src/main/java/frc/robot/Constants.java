package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    //TODO vérifier la longueur du robot
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.63); //A verifier changer argument kinematic
    public static final double kSRamsete = 0.6;
    public static final double kVRamsete = 4.48;
    public static final double kPRamsete = 2;
    public static final double maxVitesse = 1;
    public static final double maxAcceleration = 0.5;
    public static final double rampTeleop = 0.2;
}
