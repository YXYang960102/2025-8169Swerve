package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.StatusConstants;

public class StatusSubsystem extends SubsystemBase {
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private LEDState currentState = LEDState.RAINBOW; // 開機自動啟動彩虹模式
  private boolean isOverridden = false; // 用來判斷 LED 是否被手動控制
  private int rainbowFirstPixelHue = 0; // 彩虹動畫變化索引

  public enum LEDState {
    OFF,             
    RED_LEFT,        
    GREEN_RIGHT,    
    BLUE_BOTH,     
    BLINK_RED_GREEN,
    CHASE_RED,     
    CHASE_GREEN,    
    CHASE_BLUE,     
    RAINBOW        
  }

  public StatusSubsystem(int pwmPort, int length) {
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(length);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void setLEDState(LEDState state) {
    this.currentState = state;
    this.isOverridden = (state != LEDState.RAINBOW); // 只要不是 RAINBOW，就標記為手動控制
  }

  public void resetToRainbow() {
    isOverridden = false; // 讓 LED 可以回到彩虹模式
  }

  public LEDState getCurrentState() {
    return currentState;
  }

  @Override
  public void periodic() {
    if (!isOverridden) {
      currentState = LEDState.RAINBOW; // 如果沒有手動控制，回到 RAINBOW
    }

    switch (currentState) {
      case RAINBOW:
        updateRainbow();
        break;
      default:
        updateStaticLED();
        break;
    }
  }

  /** 更新彩虹 LED 效果 */
  private void updateRainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3; 
    rainbowFirstPixelHue %= 180; 
    ledStrip.setData(ledBuffer);
  }

  /** 更新靜態 LED 顏色 */
  private void updateStaticLED() {
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
      default:
        setLeftLED(StatusConstants.LED_OFF);
        setRightLED(StatusConstants.LED_OFF);
        break;
    }
    ledStrip.setData(ledBuffer);
  }

  private void setLeftLED(Color color) {
    for (int i = StatusConstants.LED_LEFT_START; i <= StatusConstants.LED_LEFT_END; i++) {
      ledBuffer.setLED(i, color);
    }
  }

  private void setRightLED(Color color) {
    for (int i = StatusConstants.LED_RIGHT_START; i <= StatusConstants.LED_RIGHT_END; i++) {
      ledBuffer.setLED(i, color);
    }
  }
}
