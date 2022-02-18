// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;

public class TournerAuto extends CommandBase {
  /** Creates a new TournerAuto. */
  double angleCible;
  BasePilotable basePilotable;  
  public TournerAuto(double angleCible, BasePilotable basePilotable) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.basePilotable = basePilotable;
    this.angleCible = angleCible;
    addRequirements(basePilotable);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    basePilotable.autoConduire(-basePilotable.getVoltagePIDF(angleCible), basePilotable.getVoltagePIDF(angleCible));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    basePilotable.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return basePilotable.atAngleCible();
  }
}
