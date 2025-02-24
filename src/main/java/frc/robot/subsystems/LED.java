package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  public enum State {
    S1,
    S2,
    S3
  }

  public SerialPort uart;

  public LED() {
    uart = new SerialPort(LEDConstants.BAUD_RATE, SerialPort.Port.kMXP);
  }

  private void SetLED(State state) {
    if (state == State.S1) {
      uart.writeString("1");
    }
    if (state == State.S2) {
      uart.writeString("2");
    }
    if (state == State.S3) {
      uart.writeString("3");
    }
  }

  public Command LEDCommand(State state) {
    return Commands.runOnce(() -> this.SetLED(state), this);
  }
}
