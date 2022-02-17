// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.lang.invoke.VolatileCallSite;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

  public class BasePilotable extends SubsystemBase {
  /** Creates a new BasePilotable. */

    private WPI_TalonFX moteurGauche = new WPI_TalonFX(3);
    private WPI_TalonFX moteurDroit = new WPI_TalonFX(4);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);
    private double conversionEncodeur;
    private DifferentialDriveOdometry odometry;
    private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
    private NetworkTableEntry voltage = calibration.add("voltage",0).getEntry();;
  


private final double conversionMoteur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);

  public BasePilotable() {

    // On inverse les moteurs pour avancer quand la vittesse est à 1
    conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4); 
    setRamp(0);
    moteurDroit.setInverted(true);
    moteurGauche.setInverted(true);
    setBrake(false);

    // Configure les capteurs internes des moteurs
    moteurGauche.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    moteurDroit.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    resetEncoder();
    resetGyro();
    
      odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
  }


  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD());
    SmartDashboard.putNumber("Position Droite", getPositionD());
    SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
    SmartDashboard.putNumber("Angle", getAngle());
    SmartDashboard.putNumber("Angle Speed", getAngleSpeed());
    SmartDashboard.putNumber("Vitesse", getVitesse());
    SmartDashboard.putNumber("Vitesse Droite", getVitesseD());
    SmartDashboard.putNumber("Vitesse Gauche", getVitesseG());
    SmartDashboard.putNumberArray("Pose", getOdometry());

    // Insère ces informations dans le dashboard

    // This method will be called once per scheduler run

  }

public void conduire(double vx,double vz) {
  // Le vx en négatif car joystick en mode avion pis le vz capped a 70% de vitesse :D -Steven

  drive.arcadeDrive(-0.7*vx, 0.5 * vz);
}

public void autoConduire(double voltGauche, double voltDroit) {
  // Fonction conduire utiliser en Autonomous 
  moteurGauche.setVoltage(voltGauche);
  moteurDroit.setVoltage(-voltDroit);
  drive.feed();
}


public double getVitesseShuffleBoard()
{
   return voltage.getDouble(0.0); 
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

  return -gyro.getAngle();
}

public void resetGyro() {
  // Gyro à 0

  gyro.reset();
}

public double getAngleSpeed() {
  return -gyro.getRate();
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

public Pose2d getPose() {
  return odometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose){
  resetEncoder();
  resetGyro();
  odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getVitesseG(), getVitesseD());
  }

  //Trajectory
  public Trajectory creerTrajectoire(String trajet){
    String trajetJSON = "output/"+trajet+".wpilib.json";
    try{
      var path = Filesystem.getDeployDirectory().toPath().resolve(trajetJSON);
      return TrajectoryUtil.fromPathweaverJson(path);
    }
    catch(IOException e){
      DriverStation.reportError("Unable to open trajectory : " + trajetJSON, e.getStackTrace());
      return null;
    }
    
  }
  public Command ramseteSimple(Trajectory trajectoire){
    //                                                                          //?? Maybe ajouter intialisation de la pose
    RamseteCommand ramseteCommand = new RamseteCommand(                         //On crée notre Ramsete Command
      trajectoire,                                                              //On passe notre trajectoire a la RamseteCommand afin qu'il sache quoi faire
      this::getPose,                                                            //On passe notre pose, afin qu'il connaisse son emplacement
      new RamseteController(2, 0.7),                                            //Ce sont des arguments réputés comme idéale pour des robots de FRC dans un Ramsete Controller
      new SimpleMotorFeedforward(Constants.kSRamsete, Constants.kVRamsete, 0),  //On donnes nos arguments de FeedForward
      Constants.kinematics,                                                     //Notre kinematics (Afin que le robot conaisse ses mesures)
      this::getWheelSpeeds,                                                     //Donne nos vitesses de roues
      new PIDController(Constants.kPRamsete, 0, 0),                             //On donne un PID Controller a chacune des roues (SpeedController Group)
      new PIDController(Constants.kPRamsete, 0, 0),                             //"                                                                     "
      this::autoConduire,                                                       //On lui donne une commande pour conduire
      this);                                                                    //Tous les "this" sont la pour spécifier qu'on parle de la BasePilotable
      return ramseteCommand.andThen(()->stop());                                //On demande au robot de s'arrêter à la fin de la trajectoire
  }
}
