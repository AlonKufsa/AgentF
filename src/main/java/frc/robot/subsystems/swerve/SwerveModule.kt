package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.IdleMode.kBrake
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.subsystems.swerve.SwerveConstants as Constants

class SwerveModule(
	private val driveMotorID: Int,
	private val steerMotorID: Int,
	private val canCoderID: Int,
	val moduleName: String,
) : Sendable {

	//Motor for controlling the velocity of the wheel
	val driveMotor = HaTalonFX(driveMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.DRIVE_MOTOR_CONFIGS)
	}

	//Motor for controlling the angle of the wheel
	val steerMotor = HaTalonFX(steerMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.steerMotorConfigs(canCoderID, moduleName))
	}


	val canCoder = CANcoder(canCoderID, Constants.SWERVE_CANBUS)


	/** Change this to change the driving motor's idle state. kBrake by default */
	var driveIdleMode: IdleMode = kBrake
		set(value) {
			driveMotor.idleMode = value
			field = value
		}

	/** Change this to change the steering motor's idle state. kBrake by default */
	var steerIdleMode: IdleMode = kBrake
		set(value) {
			steerMotor.idleMode = value
			field = value
		}


	//The angle setpoint of the swerve module from 0.0 to 360.0 from the right side of the x axis counter clockwise
	private var angleSetpoint: Rotation2d = Rotation2d(0.0)

	//The angular velocity setpoint of the wheel in rps
	private var wheelAngularVelocitySetpoint: AngularVelocity = AngularVelocity.fromRps(0.0)

	/** Function for setting the module state externally */
	fun setModuleState(swerveModuleState: SwerveModuleState) {
		//TODO: continuous wrap the angle
		angleSetpoint = swerveModuleState.angle
		wheelAngularVelocitySetpoint = AngularVelocity.fromRps(
			swerveModuleState.speedMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE_METERS)


	}


	//Shortcuts for getting speed and rotation, and ways of accessing them outside of the class
	val moduleRotationDeg: Double
		get() = canCoder.position.value * 360.0
	val moduleSpeedMPS: Double
		get() = driveMotor.velocity.value * Constants.WHEEL_CIRCUMFERENCE_METERS


	//Logging
	override fun initSendable(builder: SendableBuilder) {
		builder.addDoubleProperty("$moduleName rotation deg", { moduleRotationDeg }, null)
		builder.addDoubleProperty("$moduleName speed MPS", { moduleSpeedMPS }, null)

		builder.addDoubleProperty("$moduleName rotation setpoint deg", { angleSetpoint.degrees }, null)
		builder.addDoubleProperty("$moduleName speed setpoint MPS",
			{ wheelAngularVelocitySetpoint.asRps * Constants.WHEEL_CIRCUMFERENCE_METERS },
			null)
	}
}