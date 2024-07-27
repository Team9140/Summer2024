// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private final CommandXboxController joystick = new CommandXboxController(Constants.Ports.DRIVER_CONTROLLER);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private Launcher launcher;

  private Auto auto;
  private Command autonomousCommand;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drivetrain.MAX_SPEED * 0.1)
      .withRotationalDeadband(Constants.Drivetrain.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @Override
  public void robotInit() {
    this.launcher = Launcher.getInstance();
    this.auto = new Auto(drivetrain);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-joystick.getLeftY() * Constants.Drivetrain.MAX_SPEED)
            .withVelocityY(-joystick.getLeftX() * Constants.Drivetrain.MAX_SPEED)
            .withRotationalRate(-joystick.getRightX() * Constants.Drivetrain.MAX_ANGULAR_RATE)
        ).ignoringDisable(true)
    );

    joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    this.joystick.x().onTrue(this.launcher.prepareLaunch());

    this.joystick.rightTrigger()
      .onTrue(this.launcher.launch()).
      onFalse(new WaitCommand(1).andThen(this.launcher.off()));

    this.joystick.rightBumper()
      .onTrue(this.launcher.intake()).
      onFalse(this.launcher.off());

    this.joystick.b()
      .onTrue(this.launcher.amp()).
      onFalse(new WaitCommand(1).andThen(this.launcher.off()));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = auto.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }
  
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

}