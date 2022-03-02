package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Limelight;

public class TournerLimelight extends CommandBase {
  Limelight limelight;
  BasePilotable basePilotable;
  double voltage;
  public TournerLimelight(BasePilotable basePilotable, Limelight limelight) {
    this.limelight = limelight;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    voltage = basePilotable.getVoltagePIDF(0, limelight::getTx);
    basePilotable.autoConduire(-voltage, voltage);
  }

  @Override
  public void end(boolean interrupted) {
    basePilotable.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
