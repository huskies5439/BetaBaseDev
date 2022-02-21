// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
