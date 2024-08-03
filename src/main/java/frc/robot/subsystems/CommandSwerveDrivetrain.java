package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;


public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // Rotation for blue alliance perspective (forward is 0 degrees)
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    // Rotation for red alliance perspective (forward is 180 degrees)
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    // Flag to track if operator perspective has been applied
    private boolean hasAppliedOperatorPerspective = false;
    // Autobuilder request to move robot
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final PIDController rotationController;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    /**
     * Constructor for CommandSwerveDrivetrain.
     * @param driveTrainConstants Constants defining the drivetrain's behavior
     * @param modules Individual swerve module constants
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        rotationController = new PIDController(0.1, 0, 0);
        rotationController.enableContinuousInput(-180, 180);
        rotationController.setTolerance(Constants.ROTATION_TOLERANCE);
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Creates a command that applies a given SwerveRequest.
     * @param requestSupplier A supplier that provides a SwerveRequest
     * @return A command that applies the provided SwerveRequest
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Gets the current robot chassis speeds.
     * @return ChassisSpeeds object representing the robot's current velocity
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public SwerveRequest.ApplyChassisSpeeds getAutoRequest() {
        return AutoRequest;
    }

    public Translation2d[] getModuleLocations() {
        return m_moduleLocations;
    }
    private void updateFaceSourceControl() {
        double targetAngle = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue 
            ? Constants.BLUE_AMP_POS_ROTATION 
            : Constants.RED_AMP_POS_ROTATION;
        double currentAngle = getPose().getRotation().getDegrees();
        
        double rotationSpeed = rotationController.calculate(currentAngle, targetAngle);
        
        double maxRotationSpeed = Constants.Drivetrain.MAX_ANGULAR_RATE;
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotationSpeed, maxRotationSpeed);

        setControl(driveRequest.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(rotationSpeed));
    }
    
    public Command faceSource() {
        return Commands.run(this::updateFaceSourceControl, this)
        .until(() -> Math.abs(rotationController.getPositionError()) < rotationController.getPositionTolerance());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Velocity_X", getCurrentRobotChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Velocity_Y", getCurrentRobotChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Angular_Velocity", getCurrentRobotChassisSpeeds().omegaRadiansPerSecond);

        SmartDashboard.putNumber("Pose_X", getPose().getX());
        SmartDashboard.putNumber("Pose_Y", getPose().getY());
        SmartDashboard.putNumber("Pose_Rotation", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("pigeon yaw", this.m_pigeon2.getYaw().getValueAsDouble());

        var moduleStates = getState().ModuleStates;
        if (moduleStates != null) {
            for (int i = 0; i < moduleStates.length; i++) {
                SmartDashboard.putNumber("Module_" + i + "_Angle", moduleStates[i].angle.getDegrees());
                SmartDashboard.putNumber("Module_" + i + "_Velocity", moduleStates[i].speedMetersPerSecond);
            }
        }

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation : BlueAlliancePerspectiveRotation
                );
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    private final SwerveRequest.SysIdSwerveTranslation SysIDTranslate = new SwerveRequest.SysIdSwerveTranslation();

    private SysIdRoutine translateRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SysIDTranslate.withVolts(volts)),
                    null,
                    this));

    private final SwerveRequest.SysIdSwerveSteerGains SysIDSteer = new SwerveRequest.SysIdSwerveSteerGains();

    private SysIdRoutine steerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Units.Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SysIDSteer.withVolts(volts)),
                    null,
                    this));


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return translateRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return translateRoutine.dynamic(direction);
    }

    public Command steerSysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return steerRoutine.quasistatic(direction);
    }

    public Command steerSysIdDynamic(SysIdRoutine.Direction direction) {
        return steerRoutine.dynamic(direction);
    }
}