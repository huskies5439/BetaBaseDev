package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable limelight = networkTableInstance.getTable("limelight-huskies");
  private NetworkTableEntry tv = limelight.getEntry("tv");
  private NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry ta = limelight.getEntry("ta");
  private NetworkTableEntry ty = limelight.getEntry("ty");
  private NetworkTableEntry ledMode = limelight.getEntry("ledMode");
  private NetworkTableEntry camMode = limelight.getEntry("camMode");
  private NetworkTableEntry stream = limelight.getEntry("stream");

  double hLimelight = 0.73;
  double hCible = 2.4; // double angleLimelight=22.0;//degres
  double angleLimelight = 22.77756315;

  public Limelight() {
    ledOff();
    //camHumain();
    camDetection();
    stream.setNumber(2);
  }

  public double getDistance(){
    return(hCible-hLimelight)/Math.tan(Math.toRadians(angleLimelight+getTy()));
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
    SmartDashboard.putNumber("tx", getTx());
    SmartDashboard.putNumber("ty", getTy());
    SmartDashboard.putNumber("distance limelight", getDistance());
  }
}