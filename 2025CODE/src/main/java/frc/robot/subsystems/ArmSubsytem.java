public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armMotor; // <- [1]
  
  public ArmSubsystem () { // <- [2]
      armMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushed);

      SparkMaxConfig armConfig = new SparkMaxConfig();
      armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
      armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void runArm(double speed){ // <- [3]
      armMotor.set(speed);
  }
}