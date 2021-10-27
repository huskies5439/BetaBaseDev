// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasePilotable extends SubsystemBase {
  /** Creates a new BasePilotable. */

private WPI_TalonSRX moteurGauche = new WPI_TalonSRX(16);
private WPI_TalonSRX moteurDroit = new WPI_TalonSRX(15);
private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);

  public BasePilotable() {
    moteurDroit.setInverted(true);
    moteurGauche.setInverted(true);
    setBrake(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void conduire(double vx,double vz) {
  // Le vx en négatif car joystick en mode avion pis le vz capped a 70% de vitesse :D -Steven

  drive.arcadeDrive(-vx, 0.7 * vz);
}
public void setBrake(boolean isBrake) {
  if (isBrake) {
    moteurDroit.setNeutralMode(NeutralMode.Brake);
    moteurGauche.setNeutralMode(NeutralMode.Brake);
  }
  else {
    moteurDroit.setNeutralMode(NeutralMode.Coast);
    moteurGauche.setNeutralMode(NeutralMode.Coast);
  }
}

public double getAngle() {

  return gyro.getAngle();
}

public void resetGyro() {

  gyro.reset();
}

}
