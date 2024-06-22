package frc.robot.subsystems.swerve

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
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
	private val moduleName: String,
) : Sendable {

	//Motor for controlling the velocity of the wheel
	private val driveMotor = HaTalonFX(driveMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.DRIVE_MOTOR_CONFIGS)
	}

	//Motor for controlling the angle of the wheel
	private val steerMotor = HaTalonFX(steerMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.steerMotorConfigs(canCoderID))
		positionWrapEnabled = true
	}


	private val canCoder = CANcoder(canCoderID, Constants.SWERVE_CANBUS).apply {
		configurator.apply(Constants.canCoderConfigs(moduleName))
	}


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


	/**The angle setpoint of the swerve module from 0.0 to 360.0 from the right side of the x-axis, counterclockwise*/
	private var angleSetpoint: Rotation2d = Rotation2d(0.0)
		set(value) {
			val controlRequestSteerAngle: PositionVoltage = PositionVoltage(value.rotations)
			steerMotor.setControl(controlRequestSteerAngle)
			field = value
		}

	//The angular velocity setpoint of the wheel in rps
	private var wheelAngularVelocitySetpoint: AngularVelocity = AngularVelocity.fromRps(0.0)
		set(value) {
			val controlRequestDriveSpeed: VelocityVoltage = VelocityVoltage(value.asRps)

			driveMotor.setControl(controlRequestDriveSpeed)
			field = value
		}

	/** Function for setting the module state externally.
	 * Angle is from 0.0 to 360.0 with the right side of the x-axis, counterclockwise
	 * Speed is in meters per second, in the direction the wheel is facing
	 */
	fun setModuleState(swerveModuleState: SwerveModuleState) {
		angleSetpoint = Rotation2d.fromDegrees(swerveModuleState.angle.degrees % 360)

		wheelAngularVelocitySetpoint = AngularVelocity.fromRps(swerveModuleState.speedMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE_METERS)
	}



	
	/** Current rotation of the module in degrees from 0 to 360 from the right side of the x-axis, counterclockwise */
	val moduleRotationDeg: Double
		get() = canCoder.position.value * 360.0

	/** Current speed of the module in meters per second */
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