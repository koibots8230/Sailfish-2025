package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.util.function.BooleanSupplier;

@Logged
public class LED extends SubsystemBase {
  private Phase phase;
  private BooleanSupplier doesRobotHaveCoral;
  public State currentState;

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

  public LED(BooleanSupplier hasCoralFunc) {
    doesRobotHaveCoral = hasCoralFunc;
    uart = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kMXP);

    currentState = State.normal;
    uart.writeString("1");
    phase = Phase.preGame;
  }

  private void setLED(State state) {
    System.out.println(state);
    if (currentState == state) {
      return;
    }

    if (state == State.off) {
      uart.writeString("0");
    } else if (state == State.normal) {
      uart.writeString("1");
    } else if (state == State.auto) {
      uart.writeString("2");
    } else if (state == State.teleop) {
      uart.writeString("3");
    } else if (state == State.intake) {
      uart.writeString("4");
    } else if (state == State.hasCoral) {
      uart.writeString("5");
    } else if (state == State.cDeploying) {
      uart.writeString("6");
    } else if (state == State.cDeployed) {
      uart.writeString("7");
    } else if (state == State.climbing) {
      uart.writeString("8");
    } else if (state == State.tada) {
      uart.writeString("a");
    } else if (state == State.xanderMode) {
      uart.writeString("9");
    }
    currentState = state;
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

  private void activateXanderMode() {
    if (currentState == State.xanderMode) {
      phaseCommand();
    } else {
      LEDCommand(State.xanderMode);
    }
  }

  public void periodic() {
    if (doesRobotHaveCoral.getAsBoolean() && currentState != State.hasCoral) {
      LEDCommand(State.hasCoral);
    } else if (currentState == State.hasCoral) {
      phaseCommand();
    }
    System.out.println(currentState);
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

  public Command XModeCommand() {
    return Commands.runOnce(() -> this.activateXanderMode(), this);
  }

  public Command setAutoCommand() {
    return Commands.runOnce(() -> this.autoInit(), this);
  }

  public Command setTeleopCommand() {
    return Commands.runOnce(() -> this.teleopInit(), this);
  }
}
