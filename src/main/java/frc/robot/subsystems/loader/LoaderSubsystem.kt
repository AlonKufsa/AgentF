package frc.robot.subsystems.loader

import com.ctre.phoenix6.hardware.TalonFX
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import kotlin.math.absoluteValue

object LoaderSubsystem : SubsystemBase("Loader") {
	private val motor = TalonFX(RobotMap.LoaderMap.MOTOR_ID)
	private val beamBreak = AnalogInput(RobotMap.LoaderMap.BEAMBREAK_CHANNEL)

	val isNoteDetected: Boolean
		get() = beamBreak.voltage < LoaderConstants.BEAMBREAK_TURNPOINT

	fun runMotor(voltage: Volts) {
		motor.setVoltage(voltage)
	}

	fun stopMotor() {
		motor.stopMotor()
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("isNoteDetected", { isNoteDetected }, null)
		builder.addBooleanProperty("isMotorRunning", { motor.get().absoluteValue > 0.0 }, null)
		builder.addDoubleProperty("Beambreak voltage", { beamBreak.voltage }, null)
	}
}