package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class TournerAuto extends CommandBase {

  double angleCible;
  BasePilotable basePilotable;  
  double voltage;
  public TournerAuto(double angleCible, BasePilotable basePilotable) {

    this.basePilotable = basePilotable;
    this.angleCible = angleCible;
    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage = basePilotable.getVoltagePIDF(angleCible, basePilotable::getAngle);
    basePilotable.autoConduire(-voltage, voltage);
  }

  @Override
  public void end(boolean interrupted) {
    basePilotable.stop();
  }

  @Override
  public boolean isFinished() {
    return basePilotable.atAngleCible();
  }
}
