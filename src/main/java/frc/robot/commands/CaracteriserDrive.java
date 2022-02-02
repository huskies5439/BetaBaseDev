package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class CaracteriserDrive extends CommandBase {
  BasePilotable basePilotable;

  public CaracteriserDrive(BasePilotable basePilotable) {
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);

    private ShuffleboardTab calibration = Shuffleboard.getTab("calibration");
    private NetworkTableEntry voltage = calibration.add("voltage",0).getEntry();
  }
  @Override
  public void initialize() {
    basePilotable.autoconduire(,);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
