package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pince extends SubsystemBase {

  private Spark bras = new Spark(0);
  private DoubleSolenoid pince = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  public Pince() {
    ouvrirPince();
  }

  @Override
  public void periodic() {
  }

  public void ouvrirPince() {
    pince.set(Value.kForward);
  }

  public void fermerPince() {
    pince.set(Value.kReverse);
  }

  public void brasIn() {
    bras.set(1);
  }

  public void brasOut() {
    bras.set(0);
  }
}