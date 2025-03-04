package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.StatusConstants;

public class StatusSubsystem extends SubsystemBase {
  // LED 硬體
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;

  // 定義 LED 狀態
  public enum LEDState {
    OFF,             // 兩側 LED 全部關閉
    RED_LEFT,        // 左側紅燈，右側關閉
    GREEN_RIGHT,     // 右側綠燈，左側關閉
    BLUE_BOTH             
  }

  private LEDState currentState = LEDState.OFF; // 預設狀態為關閉

  /** 建立 StateSubsystem */
  public StatusSubsystem(int pwmPort, int length) {
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(length);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  /** 設定 LED 狀態 */
  public void setLEDState(LEDState state) {
    this.currentState = state;
    updateLED();
  }

  /** 更新 LED 顏色 */
  private void updateLED() {
    switch (currentState) {
      case RED_LEFT:
          setLeftLED(StatusConstants.LED_RED);
          setRightLED(StatusConstants.LED_OFF);
          break;
      case GREEN_RIGHT:
          setLeftLED(StatusConstants.LED_OFF);
          setRightLED(StatusConstants.LED_GREEN);
          break;
      case BLUE_BOTH:
          setLeftLED(StatusConstants.LED_BLUE);
          setRightLED(StatusConstants.LED_BLUE);
          break;
      case OFF:
      default:
          setLeftLED(StatusConstants.LED_OFF);
          setRightLED(StatusConstants.LED_OFF);
          break;
  }
  ledStrip.setData(ledBuffer);
  }

  /** 設定左側 LED 顏色 */
  private void setLeftLED(Color color) {
    for (int i = StatusConstants.LED_LEFT_START; i <= StatusConstants.LED_LEFT_END; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  /** 設定右側 LED 顏色 */
  private void setRightLED(Color color) {
    for (int i = StatusConstants.LED_RIGHT_START; i <= StatusConstants.LED_RIGHT_END; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  @Override
  public void periodic() {
    
    updateLED();
  }
}
