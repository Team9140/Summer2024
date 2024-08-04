package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {

    private static Launcher instance;

    private final WPI_TalonSRX topLauncher; // Need to keep reminding it of voltage
    private final TalonFX bottomLauncher; // Set it and forget it

    private final VoltageOut bottomLauncherController;

    private Timer motorOnTimer = new Timer();

    private Launcher() {
        this.topLauncher = new WPI_TalonSRX(Constants.Ports.TOP_LAUNCHER);
        this.bottomLauncher = new TalonFX(Constants.Ports.BOTTOM_LAUNCHER);

        CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.Launcher.BOTTOM_LAUNCHER_MAX_CURRENT)
                .withStatorCurrentLimitEnable(true);
        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration()
                .withCurrentLimits(launcherCurrentLimits);

        this.bottomLauncher.getConfigurator().apply(launcherConfiguration
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        this.bottomLauncherController = new VoltageOut(0.0).withEnableFOC(true)
                .withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

        this.topLauncher.configContinuousCurrentLimit(Constants.Launcher.TOP_LAUNCHER_MAX_CURRENT);
    }

    public static Launcher getInstance() {
        return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("skibidi timer", this.motorOnTimer.get());
    }

    public Command setTopVolts(double volts) {
        return this.runOnce(() -> this.topLauncher.setVoltage(volts));
    }

    public Command setBottomVolts(double volts) {
        return this.runOnce(() -> this.bottomLauncher.setControl(this.bottomLauncherController.withOutput(volts)));
    }

    public Command prepareLaunch() {
        return this.runOnce(() -> this.motorOnTimer.restart())
                .andThen(this.setTopVolts(Constants.Launcher.TOP_LAUNCHER_VOLTS));
    }

    public Trigger isGood() {
        return new Trigger(() -> this.motorOnTimer.hasElapsed(Constants.Launcher.LAUNCH_GAP));
    }

    public Trigger hasNote() {
        return new Trigger(() -> (this.bottomLauncher.getStatorCurrent().getValueAsDouble() <= -1.0));
    }

    public Command launch() {
        return this.setBottomVolts(Constants.Launcher.BOTTOM_LAUNCHER_VOLTS)
                .andThen(new WaitCommand(Constants.Launcher.LAUNCH_GAP))
                .andThen(this.runOnce(() -> {
                    this.motorOnTimer.stop();
                    this.motorOnTimer.reset();
                }))
                .andThen(this.off());
    }

    public Command intake() {
        return this.setBottomVolts(-Constants.Launcher.INTAKE_VOLTS)
                .andThen(this.setTopVolts(-Constants.Launcher.INTAKE_VOLTS));
    }

    public Command amp() {
        return this.setBottomVolts(Constants.Launcher.AMP_VOLTS_BOTTOM)
                .andThen(this.setTopVolts(Constants.Launcher.AMP_VOLTS_TOP));
    }

    public Command off() {
        return this.setBottomVolts(0).andThen(this.setTopVolts(0));
    }
}