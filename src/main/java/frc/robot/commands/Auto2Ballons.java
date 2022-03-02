package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Pince;

public class Auto2Ballons extends SequentialCommandGroup {

  public Auto2Ballons(BasePilotable basePilotable, Pince pince) {
    
    Trajectory one = basePilotable.creerTrajectoire("2bV2-1");
    Trajectory two = basePilotable.creerTrajectoire("2bV2-2");
    addCommands(
      //initialisation
      new InstantCommand(() -> basePilotable.resetOdometry(one.getInitialPose())),
      new InstantCommand(() -> basePilotable.setRamp(0)),
      new InstantCommand(() -> basePilotable.setBrake(true)),
      //trajet
      basePilotable.ramseteSimple(one),
      basePilotable.ramseteSimple(two)

    );
  }
}
