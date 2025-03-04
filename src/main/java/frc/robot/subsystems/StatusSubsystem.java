package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.StatusConstants;

public class StatusSubsystem extends SubsystemBase {
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private LEDState currentState = LEDState.OFF;
  private boolean isBlinking = false;
  private boolean blinkToggle = false;
  private int blinkCounter = 0;

  public enum LEDState {
    OFF,             
    RED_LEFT,        
    GREEN_RIGHT,    
    BLUE_BOTH,     
    BLINK_RED_GREEN  
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
    this.isBlinking = (state == LEDState.BLINK_RED_GREEN); // 設定閃爍模式
    updateLED();
  }

  public LEDState getCurrentState() {
    return currentState;
  }

  private void updateLED() {
    if (isBlinking) return; // 閃爍模式時不執行靜態顏色更新

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

  @Override
  public void periodic() {
    if (isBlinking) {
      blinkCounter++;
      if (blinkCounter % 10 == 0) { // 每 10 個周期閃爍一次
        blinkToggle = !blinkToggle;
        if (blinkToggle) {
          setLeftLED(StatusConstants.LED_RED);
          setRightLED(StatusConstants.LED_GREEN);
        } else {
          setLeftLED(StatusConstants.LED_GREEN);
          setRightLED(StatusConstants.LED_RED);
        }
        ledStrip.setData(ledBuffer);
      }
    } else {
      updateLED();
    }
  }
}
