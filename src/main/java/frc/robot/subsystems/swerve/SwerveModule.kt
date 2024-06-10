package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import frc.robot.subsystems.swerve.SwerveConstants as Constants

class SwerveModule(
	val posFromCenterMeters: Translation2d,
	private val driveMotorID: Int,
	private val steerMotorID: Int,
	private val canCoderID: Int,
	private val canCoderOffset: Double,
) {

	val driveMotor = TalonFX(driveMotorID, Constants.SWERVE_CANBUS)
	val steerMotor = TalonFX(steerMotorID, Constants.SWERVE_CANBUS)
	val canCoder = CANcoder(canCoderID, Constants.SWERVE_CANBUS)

	val driveController = PIDController(
		Constants.DRIVE_PID_GAINS.kP,
		Constants.DRIVE_PID_GAINS.kI,
		Constants.DRIVE_PID_GAINS.kD
	)
	val steerController = PIDController(
		Constants.STEER_PID_GAINS.kP,
		Constants.STEER_PID_GAINS.kI,
		Constants.STEER_PID_GAINS.kD
	).apply {
		enableContinuousInput(0.0, 360.0)
	}

	//The angle setpoint of the swerve module from 0.0 to 360.0 from the right side of the x axis
	private var angleSetpointDeg: Rotation2d = Rotation2d(0.0)
		private set(value) {
			field = Rotation2d.fromDegrees((450.0 + value.degrees) % 360)
		}

	//The angular velocity setpoint of the wheel in rps
	private var wheelAngularVelocitySetpointRPS: AngularVelocity = AngularVelocity.fromRps(0.0)

	//Converts from WPILib's 0 to 180 and then -180 to 0 with the positive side of the y axis rotation system
	//to the standard 0 to 360 with the positive side of the x axis rotation system
	private fun convertRotationFromWPILibToStandard(angle: Rotation2d): Rotation2d =
		Rotation2d.fromDegrees((450.0 + angle.degrees) % 360)

	fun setModuleState(swerveModuleState: SwerveModuleState) {
		angleSetpointDeg = convertRotationFromWPILibToStandard(swerveModuleState.angle)
		wheelAngularVelocitySetpointRPS = AngularVelocity.fromRps(
			swerveModuleState.speedMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE_METERS)
	}


}