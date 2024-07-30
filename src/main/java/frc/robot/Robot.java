// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Cantdle;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private final CommandXboxController joystick = new CommandXboxController(Constants.Ports.DRIVER_CONTROLLER);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private Launcher launcher;
  private Cantdle vegetable;

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
    this.vegetable = Cantdle.getInstance();
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    switch (Constants.SYSID_MODE) {
      case Teleop:
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * Constants.Drivetrain.MAX_SPEED)
                .withVelocityY(-joystick.getLeftX() * Constants.Drivetrain.MAX_SPEED)
                .withRotationalRate(-joystick.getRightX() * Constants.Drivetrain.MAX_ANGULAR_RATE))
                .ignoringDisable(true));

        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

        this.joystick.x().onTrue(this.launcher.prepareLaunch());

        this.launcher.isGood().onTrue(this.vegetable.flashGreen());

        this.joystick.rightTrigger().and(this.launcher.isGood())
            .onTrue(this.launcher.launch());

        this.joystick.rightBumper()
            .onTrue(this.launcher.intake()).onFalse(this.launcher.off());

        this.joystick.b()
            .onTrue(this.launcher.amp()).onFalse(new WaitCommand(1).andThen(this.launcher.off()));
        break;

      case SteerSysId:
        this.joystick.x().whileTrue(drivetrain.steerSysIdQuasistatic(Direction.kForward));
        this.joystick.y().whileTrue(drivetrain.steerSysIdQuasistatic(Direction.kReverse));
        this.joystick.a().whileTrue(drivetrain.steerSysIdDynamic(Direction.kForward));
        this.joystick.b().whileTrue(drivetrain.steerSysIdDynamic(Direction.kReverse));
        break;

      case DriveSysId:
        this.joystick.x().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        this.joystick.y().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        this.joystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        this.joystick.b().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        break;
    }

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) SignalLogger.stop();
    this.vegetable.setRed(DriverStation.getAlliance().get().equals(Alliance.Red));
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = auto.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (Constants.SYSID_MODE != Constants.SYSID.Teleop) SignalLogger.start();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

}