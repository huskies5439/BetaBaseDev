package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class CaracteriserDrive extends CommandBase {
  BasePilotable basePilotable;
  private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
  private NetworkTableEntry voltage = calibration.add("voltage",0).getEntry();

  public CaracteriserDrive(BasePilotable basePilotable) {
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);


  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    basePilotable.autoConduire(voltage.getDouble(0),voltage.getDouble(0));
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
