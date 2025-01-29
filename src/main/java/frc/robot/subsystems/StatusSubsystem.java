// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.StatusConstants;

// public class StatusSubsystem extends SubsystemBase {
//   private final AddressableLED m_led;
//   private final AddressableLEDBuffer m_ledBuffer;

//   private boolean isLeftOn = false;
//   private boolean isRightOn = false;

//   /** Creates a new StatusSubsystem. */
//   public StatusSubsystem(int pwmPort, int ledCount) {
//     m_led = new AddressableLED(pwmPort);
//     m_ledBuffer = new AddressableLEDBuffer(ledCount);
//     m_led.setLength(m_ledBuffer.getLength());
//     m_led.setData(m_ledBuffer);
//     m_led.start();
//   }


//   /**
//    * Left Start
//    * 
//    * @param hue        顏色的色相值 (0-180)
//    * @param saturation 飽和度 (0-255)
//    * @param value      亮度 (0-255)
//    */
//   public void lightLeft(int hue, int saturation, int value) {
//     // Left Start
//     for (int i = StatusConstants.LED_LEFT_START; i <= StatusConstants.LED_LEFT_END; i++) {
//       m_ledBuffer.setHSV(i, hue, saturation, value);
//     }
//     // Right End
//     for (int i = StatusConstants.LED_RIGHT_START; i <= StatusConstants.LED_RIGHT_END; i++) {
//       m_ledBuffer.setHSV(i, 0, 0, 0); // default
//     }
//     m_led.setData(m_ledBuffer); // LED Update

//     isLeftOn = true;
//     isRightOn = false;
//     updateSmartDashboard();
//   }

//   /**
//    * Right Start
//    * 
//    * @param hue        
//    * @param saturation 
//    * @param value      
//    */
//   public void lightRight(int hue, int saturation, int value) {
//     // Right Lighting
//     for (int i = StatusConstants.LED_RIGHT_START; i <= StatusConstants.LED_RIGHT_END; i++) {
//       m_ledBuffer.setHSV(i, hue, saturation, value);
//     }
//     // Left End
//     for (int i = StatusConstants.LED_LEFT_START; i <= StatusConstants.LED_LEFT_END; i++) {
//       m_ledBuffer.setHSV(i, 0, 0, 0); // default
//     }
//     m_led.setData(m_ledBuffer); // LED Updae

//     isLeftOn = false;
//     isRightOn = true;
//     updateSmartDashboard();
//   }

//   public void turnOffLEDs() {
//     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
//       m_ledBuffer.setHSV(i, 0, 0, 0);
//     }
//     m_led.setData(m_ledBuffer);

    
//     isLeftOn = false;
//     isRightOn = false;
//     updateSmartDashboard();
//   }

//   private void updateSmartDashboard() {
//     SmartDashboard.putBoolean("LED Left On", isLeftOn);
//     SmartDashboard.putBoolean("LED Right On", isRightOn);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     updateSmartDashboard();
//   }
// }
