// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of basePilotable project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TrajetAuto;
import frc.robot.subsystems.BasePilotable;


/**
 * basePilotable class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 BasePilotable basePilotable = new BasePilotable();

 XboxController joystick = new XboxController(0);

 private final SendableChooser <Command> chooser = new SendableChooser<>();
 
  private final Command testDroit = new TrajetAuto("testDroit", basePilotable);
  private final Command testGauche = new TrajetAuto("testGauche", basePilotable);
  private final Command trajetVide = new WaitCommand(14);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    chooser.addOption("test Gauche", testGauche);
    chooser.addOption("test Droit", testDroit);
    chooser.setDefaultOption("Trajet Vide", trajetVide);


    SmartDashboard.putData(chooser);

basePilotable.setDefaultCommand(new RunCommand(() -> basePilotable.conduire(joystick.getY(Hand.kLeft), joystick.getX(Hand.kRight)), basePilotable));

    
  }

  /**
   * Use basePilotable method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use basePilotable to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
//return new TrajetAuto("test", basePilotable);

return chooser.getSelected();


}
  }
