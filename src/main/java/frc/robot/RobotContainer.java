// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.List;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.CaracteriserDrive;
import frc.robot.commands.TrajetAuto;
import frc.robot.subsystems.BasePilotable;
import jdk.vm.ci.meta.Constant;
import frc.robot.Constants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 BasePilotable basePilotable = new BasePilotable();

 XboxController joystick = new XboxController(0);

 private final SendableChooser <Command> chooser = new SendableChooser<>();
 


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

  
    SmartDashboard.putData(chooser);

basePilotable.setDefaultCommand(new RunCommand(() -> basePilotable.conduire(joystick.getY(Hand.kLeft), joystick.getX(Hand.kRight)), basePilotable));

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.kSRamsete,
        Constants.kVRamsete, 
        0), 
        Constants.kinematics,
        10);

    TrajectoryConfig config =
      new TrajectoryConfig(
        Constants.maxVitesse, 
        Constants.maxAcceleration).setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint); 
    // An ExampleCommand will run in autonomous
    return new TrajetAuto("Test", basePilotable);

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    RamseteCommand ramseteCommand =
      new RamseteCommand(
        exampleTrajectory, 
        RamseteCommand::getPose, 
        new RamseteController(Constants.kBRamsete, Constants.kZRamsete), 
        new SimpleMotorFeedforward(
          Constants.kSRamsete,
          Constants.kVRamsete, 
          0), 
          Constants.kinematics, 
          BasePilotable::getWheelSpeeds, 
          new PIDController(Constants.kPRamsete, 0, 0), 
          new PIDController(Constants.kPRamsete, 0, 0), 
          BasePilotable::autoConduire, 
          );

          BasePilotable.resetOdometry(exampleTrajectory.getInitialPose());

          return ramseteCommand.andThen(() -> BasePilotable.autoConduire(0, 0));

  }
}
