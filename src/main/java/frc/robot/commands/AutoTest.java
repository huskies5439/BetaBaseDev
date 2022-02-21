// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Pince;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  /** Creates a new Test. */
  public AutoTest(BasePilotable basePilotable, Pince pince) {
    Trajectory senretour = basePilotable.creerTrajectoire("test-senretour");
    Trajectory versballon = basePilotable.creerTrajectoire("test-versballon");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> basePilotable.resetOdometry(senretour.getInitialPose())),
      new InstantCommand(() -> basePilotable.setRamp(0)),
      new InstantCommand(() -> basePilotable.setBrake(true)),
      basePilotable.ramseteSimple(versballon),
      basePilotable.ramseteSimple(senretour),
      new InstantCommand(() -> basePilotable.setBrake(false))
    );
  }
}

//supprimer cette commande pour programme final
