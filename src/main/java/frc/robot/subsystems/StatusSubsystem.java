// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.StatusConstants;

// public class StatusSubsystem extends SubsystemBase {
//   private final AddressableLED m_led;
//   private final AddressableLEDBuffer m_ledBuffer;
//   private int m_rainbowFirstPixelHue = 0; 

//   /** Create StatusSubsystem */
//   public StatusSubsystem(int pwmPort, int ledCount) {
//     m_led = new AddressableLED(pwmPort);
//     m_ledBuffer = new AddressableLEDBuffer(ledCount);
//     m_led.setLength(m_ledBuffer.getLength());
//     m_led.setData(m_ledBuffer);
//     m_led.start();
//   }

//   /**
//    * 
//    */
//   public void lightLeft(int hue, int saturation, int value) {
//     for (int i = StatusConstants.LED_LEFT_START; i <= StatusConstants.LED_LEFT_END; i++) {
//       m_ledBuffer.setHSV(i, hue, saturation, value);
//     }
//   }

//   /**
//    * 
//    */
//   public void lightRight(int hue, int saturation, int value) {
//     for (int i = StatusConstants.LED_RIGHT_START; i <= StatusConstants.LED_RIGHT_END; i++) {
//       m_ledBuffer.setHSV(i, hue, saturation, value);
//     }
//   }

//   /**
//    * 
//    */
//   private void driveRainbow() {
//     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
//       final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
//       m_ledBuffer.setHSV(i, hue, 255, 128); 
//     }

    
//     m_rainbowFirstPixelHue += 3;
//     m_rainbowFirstPixelHue %= 180;
//   }

//   /**
//    * 
//    */
//   @Override
//   public void periodic() {
//     driveRainbow(); 
//     m_led.setData(m_ledBuffer); 
//   }
// }