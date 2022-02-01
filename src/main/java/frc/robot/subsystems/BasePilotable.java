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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

  public class BasePilotable extends SubsystemBase {
  /** Creates a new BasePilotable. */

    private WPI_TalonFX moteurGauche = new WPI_TalonFX(3);
    private WPI_TalonFX moteurDroit = new WPI_TalonFX(4);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);
    private double conversionEncodeur;
    private DifferentialDriveOdometry odometry;

private final double conversionMoteur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);

  public BasePilotable() {

    // On inverse les moteurs pour avancer quand la vittesse est à 1
    conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4); 
    setRamp(0.1);
    moteurDroit.setInverted(true);
    moteurGauche.setInverted(true);
    setBrake(true);

    // Configure les capteurs internes des moteurs
    moteurGauche.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    moteurDroit.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    resetEncoder();
    resetGyro();
    

  }


  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD());
    //SmartDashboard.putNumber("Position Droite", getPositionD());
    //SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
    SmartDashboard.putNumber("Angle", getAngle());
    SmartDashboard.putNumber("Vitesse", getVitesse());
    SmartDashboard.putNumber("Vitesse Droite", getVitesseD());
    SmartDashboard.putNumber("Vitesse Gauche", getVitesseG());

    // Insère ces informations dans le dashboard

    // This method will be called once per scheduler run

  }

public void conduire(double vx,double vz) {
  // Le vx en négatif car joystick en mode avion pis le vz capped a 70% de vitesse :D -Steven

  drive.arcadeDrive(-vx, 0.7 * vz);
}

public void autoConduire(double voltGauche, double voltDroit) {
  // Fonction conduire utiliser en Autonomous 

  moteurDroit.setVoltage(voltDroit);
  moteurGauche.setVoltage(voltGauche);
  drive.feed();
}



public void stop() {
  // Stop les moteurs.

  autoConduire(0, 0);
}


public void setRamp(double ramp) {

  moteurDroit.configOpenloopRamp(ramp);
  moteurGauche.configOpenloopRamp(ramp);
}

public void setBrake(boolean isBrake) {
  // Détermine si le robot brake ou non quand il n'avance pas.

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
  // Degrés fait par le moteur gauche

  return moteurGauche.getSelectedSensorPosition()*conversionMoteur;
}



public double getPositionD() {
  // Degrés fait par le moteur droit

  return -moteurDroit.getSelectedSensorPosition()*conversionMoteur;
}

public double getPosition() {
  // Degrés fait par la roue droite et gauche

  return (getPositionG() + getPositionD() ) / 2.0;
}

public double getVitesseD() {

  return -moteurDroit.getSelectedSensorVelocity()*conversionEncodeur*10;//x10 car les encodeurs des Falcon donne des clics par 100 ms.
}

public double getVitesseG() {

  return moteurGauche.getSelectedSensorVelocity()*conversionEncodeur*10;
}

public double getVitesse() {

  return (getVitesseD() + getVitesseG()) / 2;
}


public void resetEncoder(){
  // Encodeur à 0

  moteurDroit.setSelectedSensorPosition(0);
  moteurGauche.setSelectedSensorPosition(0);
}
 
public double[] getOdometry(){
  double[] position = new double[3];
  double x = getPose().getTranslation().getX();
  double y = getPose().getTranslation().getY();
  double theta = getPose().getRotation().getDegrees();
  position[0] = x;
  position[1] = y;
  position[2] = theta;
  return position;
}

private Pose2d getPose() {
  return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose){
  resetEncoder();
  resetGyro();
  odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
}
}
