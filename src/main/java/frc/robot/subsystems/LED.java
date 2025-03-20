package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  public enum State {
    teleop,
    endgame,
    auto,
    p1,
    p2,
    p3,
    off
  }

  public SerialPort uart;

  public LED() {
    uart = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kMXP);
  }

  private void setLED(State state) {
    if (state == State.teleop) {
      uart.writeString("0"); // purple and orange pattern
    }
    if (state == State.endgame) {
      uart.writeString("1"); // rainbow flash
    }
    if (state == State.auto) {
      uart.writeString("2"); // rainbow gradient
    }
    if (state == State.p1) {
      uart.writeString("3"); // purple
    }
    if (state == State.p2) {
      uart.writeString("4"); // teal
    }
    if (state == State.p3) {
      uart.writeString("5"); // orange
    }
    if (state == State.off) {
      uart.writeString("5"); // all LEDs off
    }
  }

  public Command LEDCommand(State state) {
    return Commands.runOnce(() -> this.setLED(state), this);
  }
}
