// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Limelight;

public class TournerLimelight extends CommandBase {
  /** Creates a new TournerLimelight. */
  Limelight limelight;
  BasePilotable basePilotable;
  double voltage;
  public TournerLimelight(BasePilotable basePilotable, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.basePilotable = basePilotable;
    addRequirements(basePilotable);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // limelight.ledOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltage = basePilotable.getVoltagePIDF(0, limelight::getTx);
    basePilotable.autoConduire(-voltage, voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    basePilotable.stop();
    //limelight.ledOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return basePilotable.atAngleCible(); //Tester la reconaisance
    return false;
  }
}