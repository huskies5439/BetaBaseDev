package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class CaracteriserDrive extends CommandBase {
  BasePilotable basePilotable;

  public CaracteriserDrive(BasePilotable basePilotable) {
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);

  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    basePilotable.autoConduire(basePilotable.getVitesseShuffleBoard(),-basePilotable.getVitesseShuffleBoard());//2 vitesses ++ == drive linéaire / 2 vitesses +- == drive rotation
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
