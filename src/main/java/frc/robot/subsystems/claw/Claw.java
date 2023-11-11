package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  PneumaticsControlModule pcm;
  Compressor airCompressor;
  boolean compressorEnabled;
  boolean pressureSwitch;
  double current;
  DoubleSolenoid test;
  DoubleSolenoid test2;

  /** Creates a new Pneumatics. */
  public Claw() {
    pcm = new PneumaticsControlModule(0);
    airCompressor = new Compressor(pcm.getModuleNumber(), PneumaticsModuleType.CTREPCM);
    test = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    airCompressor.enableDigital();
  }

  public void moveSolenoid() {
    test.set(Value.kForward);
  }
  public void resetSolenoid() {
    test.set(Value.kReverse);
  }



  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
