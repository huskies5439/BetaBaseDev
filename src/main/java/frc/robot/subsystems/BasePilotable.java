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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

  public class BasePilotable extends SubsystemBase {
  /** Creates a new BasePilotable. */

private WPI_TalonFX moteurGauche = new WPI_TalonFX(1);
private WPI_TalonFX moteurDroit = new WPI_TalonFX(2);
private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);

private final double conversionMoteur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);

  public BasePilotable() {

    // On inverse les moteurs pour avancer quand la vittesse est à 1
    moteurDroit.setInverted(true);
    moteurGauche.setInverted(true);
    setBrake(false);

    // Configure les capteurs internes des moteurs
    moteurGauche.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    moteurDroit.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    resetEncoder();
    resetGyro();
    moteurGauche.configSelectedFeedbackCoefficient((14/72)*(16/72)*Math.PI*Units.inchesToMeters(4));
    
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Position Droite", getPositionD());
    SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
    // Insère ces informations dans le dashboard

    // This method will be called once per scheduler run

  }

public void conduire(double vx,double vz) {
  // Le vx en négatif car joystick en mode avion pis le vz capped a 70% de vitesse :D -Steven

  drive.arcadeDrive(-vx, 0.7 * vz);
}

public void autoConduire(double vx, double vz) {
  // Fonction conduire utiliser en Autonomous 

  drive.arcadeDrive(vx, vz, false);
}

public void stop() {
  // Stop les moteurs

  autoConduire(0, 0);
}

public void setBrake(boolean isBrake) {
  // Détermine si le robot brake ou non quand il n'avance pas

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
  // Angle du robot par le gyro

  return gyro.getAngle();
}

public void resetGyro() {
  // Gyro à 0

  gyro.reset();
}

public double getPositionG() {
  // Degrès fait par le moteur gauche

  return moteurGauche.getSelectedSensorPosition()*conversionMoteur;
}



public double getPositionD() {
  // Degrès fait par le moteur droit

  return -moteurDroit.getSelectedSensorPosition()*conversionMoteur;
}

public double getPosition() {
  // Degrès fait par la roue droite et gauche

  return (getPositionG() + getPositionD() ) / 2.0;
}


public void resetEncoder(){
  // Encodeur à 0

  moteurDroit.setSelectedSensorPosition(0);
  moteurGauche.setSelectedSensorPosition(0);
}
 
}