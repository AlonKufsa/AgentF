package frc.robot.subsystems.swerve

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage

import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.IdleMode.kBrake
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.subsystems.swerve.SwerveConstants as Constants

class SwerveModule(
	private val driveMotorID: Int,
	private val steerMotorID: Int,
	private val canCoderID: Int,
	private val moduleName: String,
) : Sendable {

	/** Motor for controlling the velocity of the wheel */
	private val driveMotor = HaTalonFX(driveMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.DRIVE_MOTOR_CONFIGS)
	}

	/** Motor for controlling the angle of the wheel */
	private val steerMotor = HaTalonFX(steerMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.steerMotorConfigs(canCoderID))
		positionWrapEnabled = true
	}


	/** CANCoder measuring the angle of the module. Sits on the wheel. Measures from 0 to 1 counterclockwise */
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


	private var controlRequestSteerAngle: MotionMagicVoltage = MotionMagicVoltage(0.0)
	/** The angle setpoint of the swerve module from 0.0 to 360.0 from the right side of the x-axis, counterclockwise*/
	private var angleSetpoint: Rotation2d = Rotation2d(0.0)
		set(value) {
			controlRequestSteerAngle.Position = value.rotations
			steerMotor.setControl(controlRequestSteerAngle)
			field = value
		}


	private var controlRequestDriveVelocity: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0.0)
	/** The angular velocity setpoint of the wheel in rps */
	private var wheelAngularVelocitySetpoint: AngularVelocity = AngularVelocity.fromRps(0.0)
		set(value) {
			controlRequestDriveVelocity.Velocity = value.asRps

			driveMotor.setControl(controlRequestDriveVelocity)
			field = value
		}

	/** Function for setting the module state externally.
	 * Angle is from 0.0 to 360.0 degrees with the right side of the x-axis, counterclockwise.
	 * Speed is in meters per second, in the direction the wheel is facing.
	 */
	fun setModuleState(swerveModuleState: SwerveModuleState) {
		setModuleRotation(swerveModuleState.angle)
		setModuleSpeed(swerveModuleState.speedMetersPerSecond)
	}

	fun setModuleSpeed(speedMPS: Double) {
		wheelAngularVelocitySetpoint = AngularVelocity.fromRps(
			(speedMPS / Constants.WHEEL_CIRCUMFERENCE_METERS) * Constants.DRIVE_TRANSMISSION)
	}

	/** Set the angle the module will be in from 0.0 to 360.0 degrees with the right side of the x-axis, counterclockwise */
	fun setModuleRotation(rotation: Rotation2d) {
		angleSetpoint = Rotation2d.fromDegrees(MathUtil.inputModulus(rotation.degrees, 0.0, 360.0))
	}



	
	/** Current rotation of the module in degrees from 0 to 360 from the right side of the x-axis, counterclockwise */
	val moduleRotation: Rotation2d
		get() = Rotation2d.fromRotations(canCoder.position.value)

	/** Current speed of the module in meters per second */
	val moduleSpeedMPS: Double
		get() = (driveMotor.velocity.value / Constants.DRIVE_TRANSMISSION) * Constants.WHEEL_CIRCUMFERENCE_METERS

	/** Current measured module state */
	val currentModuleState: SwerveModuleState
		get() = SwerveModuleState(moduleSpeedMPS, moduleRotation)


	/** Current setpoint of the module */
	val currentModuleStateSetpoint: SwerveModuleState
		get() = SwerveModuleState(
			wheelAngularVelocitySetpoint.asRps * Constants.WHEEL_CIRCUMFERENCE_METERS,
			angleSetpoint
		)




	// Logging
	override fun initSendable(builder: SendableBuilder) {
		builder.addDoubleProperty("$moduleName rotation deg", { moduleRotation.degrees }, null)
		builder.addDoubleProperty("$moduleName speed MPS", { moduleSpeedMPS }, null)

		builder.addDoubleProperty("$moduleName rotation setpoint deg", { currentModuleStateSetpoint.angle.degrees }, null)
		builder.addDoubleProperty("$moduleName speed setpoint MPS", { currentModuleStateSetpoint.speedMetersPerSecond }, null)
	}
}