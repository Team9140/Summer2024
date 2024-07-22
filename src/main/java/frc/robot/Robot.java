// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import frc.robot.generated.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Launcher;

public class Robot extends TimedRobot {
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandXboxController joystick = new CommandXboxController(Constants.Ports.DRIVER_CONTROLLER);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final Telemetry logger = new Telemetry(Constants.Drivetrain.MAX_SPEED);

  // Field-centric drive request
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(Constants.Drivetrain.MAX_SPEED * 0.1)
          .withRotationalDeadband(Constants.Drivetrain.MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Brake request to stop the robot
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // Request to point wheels in a specific direction
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private Launcher launcher;

  @Override
  public void robotInit() {
    drivetrain.registerTelemetry(logger::telemeterize);
    this.launcher = Launcher.getInstance();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                    .withVelocityX(-joystick.getLeftY() * Constants.Drivetrain.MAX_SPEED)  // Negative Y is forward
                    .withVelocityY(-joystick.getLeftX() * Constants.Drivetrain.MAX_SPEED)  // Negative X is left
                    .withRotationalRate(-joystick.getRightX() * Constants.Drivetrain.MAX_ANGULAR_RATE)  // Negative X is counter-clockwise
            ).ignoringDisable(true)
    );

    // A button will brake the robot
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // B button will point the wheels in the direction of the left stick
    joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    ));

    // D-pad Up: Move forward at 50% speed (robot-centric)
    joystick.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

    // D-pad Down: Move backward at 50% speed (robot-centric)
    joystick.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // Left bumper will reset the field-centric heading
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));

    this.joystick.y()
            .onTrue(this.launcher.prepareLaunch())
            .onFalse(this.launcher.off());

    this.joystick.rightTrigger()
            .onTrue(this.launcher.launch()).
            onFalse(this.launcher.off());

    this.joystick.rightBumper()
            .onTrue(this.launcher.intake()).
            onFalse(this.launcher.off());
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
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit(){}

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