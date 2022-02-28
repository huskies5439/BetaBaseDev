package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable limelight = networkTableInstance.getTable("limelight-huskies");
  private NetworkTableEntry tv = limelight.getEntry("tv");
  private NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry ta = limelight.getEntry("ta");
  private NetworkTableEntry ty = limelight.getEntry("ty");
  private NetworkTableEntry ledMode = limelight.getEntry("ledMode");
  private NetworkTableEntry camMode = limelight.getEntry("camMode");


  //Estimateur de distance, voir documentation Limelight ce sont les chiffres de IR
  double hLimelight = Units.inchesToMeters(20.5);
  double hCible = Units.inchesToMeters(89.5); // double angleLimelight=22.0;//degres
  double angleLimelight = Units.inchesToMeters(22.0);
  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    //ledOff();
    //camHumain();
    ledOn();
    camDetection();
  }
  public double getDistance(){
    return(hCible-hLimelight)/Units.inchesToMeters(Math.tan(Math.toRadians(angleLimelight+getTy())));
  }

  public double getTa() {
    return ta.getDouble(0);
  }

  public boolean getTv() {
    return tv.getDouble(0)==1;
  }

  public double getTx() {
    return tx.getDouble(0);
  }

  public double getTy() {
    return ty.getDouble(0);
  }

  public void ledOn() {
    ledMode.setNumber(3);
  }
  public void ledOff() {
    ledMode.setNumber(1);
  }
  public void camHumain(){
    camMode.setNumber(1);
  }
  public void camDetection(){
    camMode.setNumber(0);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ta", getTa());
    SmartDashboard.putNumber("tx", getTx());
    SmartDashboard.putNumber("ty", getTy());
    SmartDashboard.putNumber("distance limelight", getDistance());

    
      

  }

}