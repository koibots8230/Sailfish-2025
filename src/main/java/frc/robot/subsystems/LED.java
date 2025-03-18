package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilo;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private Phase phase;
  
  public enum State {
    off,
    normal,
    auto,
    teleop,
    intake,
    hasCoral,
    cDeploying,
    cDeployed,
    climbing,
    tada,
    xanderMode
    
  }

  public enum Phase {
    preGame,
    auto,
    teleop,
  
  }

  public SerialPort uart;

  public LED() {
    uart = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kMXP);
  }

  private void setLED(State state) {
    if (state == State.off) {
      uart.writeString("0");
    }
    if (state == State.normal) {
      uart.writeString("1");
    }
    if (state == State.auto) {
      uart.writeString("2");
    }
    if (state == State.teleop) {
      uart.writeString("3");
    }
    if (state == State.intake) {
      uart.writeString("4");
    }
    if (state == State.hasCoral) {
      uart.writeString("5");
    }
    if (state == State.cDeploying) {
      uart.writeString("6");
    }
    if (state == State.cDeployed) {
      uart.writeString("7");
    }
    if (state == State.climbing) {
      uart.writeString("8");
    }
    if (state == State.tada) {
      uart.writeString("9");
    }
    if (state == State.xanderMode) {
      uart.writeString("10");
    }
  }

  private void resetLEDsToPhase() {
    if (phase == Phase.preGame) {
      setLED(State.normal);
    }
    if (phase == Phase.auto) {
      setLED(State.auto);
    }
    if (phase == Phase.teleop) {
      setLED(State.teleop);
    }
  }

  public void robotInit() {
    phase = Phase.preGame;
    phaseCommand();
  }

  public void autoInit() {
    phase = Phase.auto;
    phaseCommand();
  }

  public void teleopInit() {
    phase = Phase.teleop;
    phaseCommand();
  }

  public Command LEDCommand(State state) {
    return Commands.runOnce(() -> this.setLED(state), this);
  }

  public Command phaseCommand() {
    return Commands.runOnce(() -> this.resetLEDsToPhase(), this);
  }
}
