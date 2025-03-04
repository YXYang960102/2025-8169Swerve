package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.StatusConstants;

public class StatusSubsystem extends SubsystemBase {

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;


  public enum LEDState {
    OFF,             
    RED_LEFT,        
    GREEN_RIGHT,    
    BLUE_BOTH             
  }

  private LEDState currentState = LEDState.BLUE_BOTH; 


  public StatusSubsystem(int pwmPort, int length) {
    ledStrip = new AddressableLED(pwmPort);
    ledBuffer = new AddressableLEDBuffer(length);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }


  public void setLEDState(LEDState state) {
    this.currentState = state;
    updateLED();
  }


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
    
    updateLED();
  }
}
