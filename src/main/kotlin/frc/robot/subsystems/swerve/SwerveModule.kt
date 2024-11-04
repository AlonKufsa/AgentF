package frc.robot.subsystems.swerve

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.IdleMode.kBrake
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO
import frc.robot.subsystems.swerve.SwerveConstants.WHEEL_CIRCUMFERENCE_METERS
import frc.robot.subsystems.swerve.SwerveConstants as Constants

class SwerveModule(
	private val driveMotorID: Int,
	private val steerMotorID: Int,
	private val canCoderID: Int,
	val moduleName: String,
	private val invertedDrive: Boolean = false,
	private val invertedSteer: Boolean = false,
) {

	/** Motor for controlling the velocity of the wheel */
	private val driveMotor = HaTalonFX(driveMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.DRIVE_MOTOR_CONFIGS)
		inverted = invertedDrive
	}

	/** Motor for controlling the angle of the wheel */
	private val steerMotor = HaTalonFX(steerMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.steerMotorConfigs(canCoderID))
		inverted = invertedSteer
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


	private var controlRequestSteerAngle: MotionMagicVoltage = MotionMagicVoltage(0.0).apply {
		EnableFOC = false
		Slot = 0
		LimitForwardMotion = false
		LimitReverseMotion = false
	}

	/** The angle setpoint of the swerve module in wpilib standards*/
	private var angleSetpoint: Rotation2d = Rotation2d(0.0)
		set(value) {
			controlRequestSteerAngle.Position = value.rotations
			steerMotor.setControl(controlRequestSteerAngle)
			field = value
		}

	private var controlRequestDriveVelocity: MotionMagicVelocityVoltage = MotionMagicVelocityVoltage(0.0)

	/** The angular velocity setpoint of the wheel in rps */
	private var driveMotorAngularVelocitySetpoint: AngularVelocity = AngularVelocity.fromRps(0.0)
		set(value) {
			controlRequestDriveVelocity.Velocity = value.asRps

			driveMotor.setControl(controlRequestDriveVelocity)
			field = value
		}

	/** Function for setting the module state externally.
	 * Angle is in WPILib standards for swerve.
	 * Speed is in meters per second, in the direction the wheel is facing.
	 */
	fun setModuleState(swerveModuleState: SwerveModuleState) {
		if (swerveModuleState.speedMetersPerSecond != 0.0) setModuleRotation(swerveModuleState.angle)
		setModuleSpeed(swerveModuleState.speedMetersPerSecond)
	}

	/** Set only the speed of the module, in MPS*/
	fun setModuleSpeed(speedMPS: Double) {
		driveMotorAngularVelocitySetpoint = AngularVelocity.fromRps(
			(speedMPS / WHEEL_CIRCUMFERENCE_METERS) * DRIVE_GEAR_RATIO)
	}

	/** Set only the angle the module will be in, using WPILib standards for swerve */
	fun setModuleRotation(rotation: Rotation2d) {
		angleSetpoint = rotation
	}

	// *Logging and module statistics*
	private fun convertDriveMotorRotationsToPosition(rotations: Double): Double {
		return rotations * WHEEL_CIRCUMFERENCE_METERS
	}

	val position: SwerveModulePosition
		get() {
			return SwerveModulePosition(convertDriveMotorRotationsToPosition(driveMotor.position.value),
				currentRotation)
		}

	/** Current rotation of the module in WPLib standards */
	val currentRotation: Rotation2d
		get() = Rotation2d.fromRotations(canCoder.absolutePosition.value)

	val currentRotationNotWrapped: Rotation2d
		get() = Rotation2d.fromRotations(canCoder.position.value)

	/** Current angular velocity of the module (across its steering axis) */
	val currentAngularVelocity: AngularVelocity
		get() = AngularVelocity.fromRps(canCoder.velocity.value)

	/** Current speed of the module in meters per second */
	val currentSpeedMPS: Double
		get() = (driveMotor.velocity.value / DRIVE_GEAR_RATIO) * WHEEL_CIRCUMFERENCE_METERS

	/** Current measured module state */
	val currentModuleState: SwerveModuleState
		get() = SwerveModuleState(currentSpeedMPS, currentRotation)

	val currentAppliedSteerVoltage: Volts
		get() = steerMotor.motorVoltage.value


	/** Current setpoint of the module */
	val currentModuleStateSetpoint: SwerveModuleState
		get() = SwerveModuleState(
			driveMotorAngularVelocitySetpoint.asRps / Constants.DRIVE_GEAR_RATIO * Constants.WHEEL_CIRCUMFERENCE_METERS,
			angleSetpoint
		)

	/** Use for testing */
	fun setDriveVoltage(voltage: Volts) {
		driveMotor.setVoltage(voltage)
	}

	/** Use for testing */
	fun setSteerVoltage(voltage: Volts) {
		steerMotor.setVoltage(voltage)
	}


	// Logging
	fun addModuleInfo(builder: SendableBuilder) {
		builder.addDoubleProperty("$moduleName rotation deg", { currentRotation.degrees }, null)
		builder.addDoubleProperty("$moduleName speed MPS", { currentSpeedMPS }, null)

		builder.addDoubleProperty("$moduleName rotation setpoint deg",
			{ currentModuleStateSetpoint.angle.degrees },
			null)
		builder.addDoubleProperty("$moduleName speed setpoint MPS",
			{ currentModuleStateSetpoint.speedMetersPerSecond },
			null)
		builder.addDoubleProperty("$moduleName steer motor setpoint",
			{ angleSetpoint.degrees },
			null)
		builder.addDoubleProperty("$moduleName steer error deg",
			{ angleSetpoint.degrees - currentRotation.degrees },
			null)

		builder.addDoubleProperty("$moduleName drive voltage", { driveMotor.motorVoltage.value }, null)
	}
}