package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object IntakeSubsystem : SubsystemBase("Intake") {
	private val bottomMotor = TalonFX(RobotMap.IntakeMap.BOTTOM_MOTOR_ID)
	private val topMotor = TalonFX(RobotMap.IntakeMap.TOP_MOTOR_ID)

	fun runMotors() {
		bottomMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
		topMotor.setVoltage(IntakeConstants.MOTOR_VOLTAGE)
	}

	fun stopMotors() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
	}
}