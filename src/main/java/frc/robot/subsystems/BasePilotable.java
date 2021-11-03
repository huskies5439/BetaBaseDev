// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BasePilotable extends SubsystemBase {
  /** Creates a new BasePilotable. */

private WPI_TalonFX moteurGauche = new WPI_TalonFX(1);
private WPI_TalonFX moteurDroit = new WPI_TalonFX(2);
private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);


  public BasePilotable() {
    moteurDroit.setInverted(true);
    moteurGauche.setInverted(true);
    setBrake(true);
    moteurGauche.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    moteurDroit.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Position Droite", getPositionD());
    SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
    // This method will be called once per scheduler run
  }

public void conduire(double vx,double vz) {
  // Le vx en négatif car joystick en mode avion pis le vz capped a 70% de vitesse :D -Steven

  drive.arcadeDrive(-vx, 0.7 * vz);
}

public void autoConduire(double vx, double vz) {

  drive.arcadeDrive(vx, vz, false);
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

public double getPositionG() {

  return moteurGauche.getSelectedSensorPosition();
}
public double getPositionD() {

  return moteurDroit.getSelectedSensorPosition();
}

public double getPosition() {

  return (getPositionG() + getPositionD() ) / 2.0;
}

public void stop() {
}


}
