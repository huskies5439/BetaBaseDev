package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

  public class BasePilotable extends SubsystemBase {
    //Initalisation des Capteurs, Encodeurs et Moteurs
    private WPI_TalonFX moteurGauche = new WPI_TalonFX(3);
    private WPI_TalonFX moteurDroit = new WPI_TalonFX(4);
    private ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);
    //Création pour dans le DashBoard
    private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
    private NetworkTableEntry voltageDrive = calibration.add("voltageDrive",0).getEntry();
    //Autre
    private double conversionEncodeur;
    private final double conversionMoteur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);
    private DifferentialDriveOdometry odometry;
    private SimpleMotorFeedforward tournerFF = new SimpleMotorFeedforward(0.496, 0.0287);
    private ProfiledPIDController tournerPID = new ProfiledPIDController(0.20, 0, 0, new TrapezoidProfile.Constraints(90, 90));
    private MedianFilter filter = new MedianFilter(5);

  public BasePilotable() {
    //Configure les moteurs
    conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4); 
    setRamp(0);setBrake(false);
    moteurGauche.setInverted(true);
    moteurDroit.setInverted(false);

    //Configure les capteurs internes des moteurs
    moteurGauche.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    moteurDroit.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,0);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()));
    tournerPID.enableContinuousInput(-180, 180);
    tournerPID.setTolerance(1);
    resetEncoder();
    resetGyro();
  }

  @Override
  public void periodic() {
    //Insertion d'information dans le DashBoard
    odometry.update(Rotation2d.fromDegrees(getAngle()), getPositionG(), getPositionD());
    //SmartDashboard.putNumber("Position Droite", getPositionD());
    //SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
    SmartDashboard.putNumber("Angle", getAngle());
    //SmartDashboard.putNumber("Angle Speed", getAngleSpeed());
    SmartDashboard.putNumber("Vitesse Moyenne", getVitesse());
    //SmartDashboard.putNumber("Vitesse Droite", getVitesseD());
    //SmartDashboard.putNumber("Vitesse Gauche", getVitesseG());
    //SmartDashboard.putNumberArray("Pose", getOdometry());
  }

public void conduire(double vx,double vz) {
  //vx la vitesse pour avancer et vz la vitesse pour tourner
  drive.arcadeDrive(-0.7*vx, 0.5*vz);
}

public void autoConduire(double voltGauche, double voltDroit) {
  //Fonction conduire pour en Autonomous 
  moteurGauche.setVoltage(voltGauche);
  moteurDroit.setVoltage(voltDroit);
  drive.feed();
}


public double getVitesseShuffleBoard() {
  //Vitesse à Implémenter dans le DashBoard
  return voltageDrive.getDouble(0.0); 
}
public void stop() {
  //Bein Le Robot Bein y s'arrête
  autoConduire(0, 0);
}


public void setRamp(double ramp) {
//Création de la ramp
  moteurDroit.configOpenloopRamp(ramp);
  moteurGauche.configOpenloopRamp(ramp);
}

public void setBrake(boolean isBrake) {
  //Détermine si le robot brake ou non quand il n'avance pas.
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
  //Angle à Implémenter dans le DashBoard
  return -gyro.getAngle();
}

public void resetGyro() {
  //Gyro à 0
  gyro.reset();
}

public double getAngleSpeed() {
  //Vitesse de l'angle à Implémenter dans le DashBoard
  return filter.calculate(-gyro.getRate());
} 

public double getPositionG() {
  //Position du Moteur Gauche à Implémenter dans le DashBoard
  return moteurGauche.getSelectedSensorPosition()*conversionMoteur;
}



public double getPositionD() {
  //Position du Moteur Droit à Implémenter dans le DashBoard
  return moteurDroit.getSelectedSensorPosition()*conversionMoteur;
}

public double getPosition() {
  //Position Moyenne des Moteurs à Implémenter dans le DashBoard
  return (getPositionG() + getPositionD() ) / 2.0;
}

public double getVitesseD() {
  //Vitesse du Moteur Droit à Implémenter dans le DashBoard
  return moteurDroit.getSelectedSensorVelocity()*conversionEncodeur*10;//x10 car les encodeurs des Falcon donne des clics par 100 ms.
}

public double getVitesseG() {
  //Vitesse du Moteur Gauche à Implémenter dans le DashBoard
  return moteurGauche.getSelectedSensorVelocity()*conversionEncodeur*10;
}

public double getVitesse() {
  //Vitesse du Moteur Moyenne à Implémenter dans le DashBoard
  return (getVitesseD() + getVitesseG()) / 2;
}


public void resetEncoder(){
  //Reset Encodeur
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
  //Reset Odometry
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
      return ramseteCommand.andThen(()->stop());                              //On demande au robot de s'arrêter à la fin de la trajectoire
  }

  public double getVoltagePIDF(double angleCible, DoubleSupplier mesure){
    return tournerPID.calculate(mesure.getAsDouble(), angleCible) + tournerFF.calculate(tournerPID.getSetpoint().velocity);
  }

  public boolean atAngleCible(){
   return tournerPID.atGoal();
  }
}
