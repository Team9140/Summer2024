package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // Rotation for blue alliance perspective (forward is 0 degrees)
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    // Rotation for red alliance perspective (forward is 180 degrees)
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    // Flag to track if operator perspective has been applied
    private boolean hasAppliedOperatorPerspective = false;
    
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    /**
     * Constructor for CommandSwerveDrivetrain.
     * @param driveTrainConstants Constants defining the drivetrain's behavior
     * @param modules Individual swerve module constants
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
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
    
    /**
     * This method is called periodically by the CommandScheduler.
     * It handles applying the correct operator perspective based on alliance color.
     */
    @Override
    public void periodic() {

        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        SmartDashboard.putNumber("Drivetrain/Velocity_X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Velocity_Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Angular_Velocity", speeds.omegaRadiansPerSecond);

        Pose2d pose = getPose();
        SmartDashboard.putNumber("Drivetrain/Pose_X", pose.getX());
        SmartDashboard.putNumber("Drivetrain/Pose_Y", pose.getY());
        SmartDashboard.putNumber("Drivetrain/Pose_Rotation", pose.getRotation().getDegrees());

        var moduleStates = getState().ModuleStates;
        if (moduleStates != null) {
            for (int i = 0; i < moduleStates.length; i++) {
                SmartDashboard.putNumber("Drivetrain/Module_" + i + "_Angle", moduleStates[i].angle.getDegrees());
                SmartDashboard.putNumber("Drivetrain/Module_" + i + "_Velocity", moduleStates[i].speedMetersPerSecond);
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
}