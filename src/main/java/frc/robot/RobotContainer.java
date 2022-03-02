package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auto1Ballon;
import frc.robot.commands.Auto2Ballons;
import frc.robot.commands.Auto3Ballons;
import frc.robot.commands.CaracteriserDrive;
import frc.robot.commands.TournerAuto;
import frc.robot.commands.TournerLimelight;
import frc.robot.commands.TrajetAuto;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pince;

public class RobotContainer {
  BasePilotable basePilotable = new BasePilotable();
  Pince pince = new Pince();
  Limelight limelight = new Limelight();

  XboxController joystick = new XboxController(0);

  private final SendableChooser<Command> chooser = new SendableChooser<>();
  //trajets
  private final Command Auto1Ballon = new Auto1Ballon(basePilotable, pince);
  private final Command Auto2Ballons = new Auto2Ballons(basePilotable, pince);
  private final Command Auto3Ballons = new Auto3Ballons(basePilotable, pince);
  private final Command trajetVide = new WaitCommand(14);

  public RobotContainer() {
    configureButtonBindings();

    chooser.setDefaultOption("Trajet Vide", trajetVide);
    chooser.addOption("Trajet 1 Ballon", Auto1Ballon);
    chooser.addOption("Tajet 2 Ballons", Auto2Ballons);
    chooser.addOption("Trajet 3 Ballons", Auto3Ballons);
  

    SmartDashboard.putData(chooser);

    basePilotable.setDefaultCommand(
    new RunCommand(() -> basePilotable.conduire(joystick.getLeftY(), joystick.getRightX()), basePilotable));
  }

  private void configureButtonBindings() {
    new JoystickButton(joystick, Button.kA.value).whenPressed(new InstantCommand(pince::ouvrirPince));
    new JoystickButton(joystick, Button.kB.value).whenPressed(new InstantCommand(pince::fermerPince));
    new JoystickButton(joystick, Button.kX.value).toggleWhenPressed(new TournerLimelight(basePilotable, limelight));
  }

  public Command getAutonomousCommand() {
    /*return chooser.getSelected()
    //prepare Teleop
    .andThen (new InstantCommand(() -> basePilotable.setBrake(false)))
    .andThen (new InstantCommand(() -> basePilotable.setRamp(0)));*/
    return null;
  }
}
