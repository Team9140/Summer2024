package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Cantdle extends SubsystemBase {
    private CANdle candle;
    private static Cantdle instance;
    private boolean isRed;

    private Cantdle() {
        this.candle = new CANdle(Constants.Ports.CANDLE, "moe");
        this.isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
    }

    public void setRed(boolean isRed) {
        this.isRed = isRed;
    }

    public static Cantdle getInstance() {
        return Cantdle.instance == null ? new Cantdle() : Cantdle.instance;
    }

    public Command flashGreen() {
        return this.runOnce(() -> this.candle.setLEDs(0, 255, 0))
        .andThen(new WaitCommand(0.5))
        .andThen(this.solidAllianceColor());
    }

    public Command solidAllianceColor() {
        return this.runOnce(() -> this.candle.setLEDs(this.isRed ? 255 : 0, 0, this.isRed ? 0 : 255));
    }
}