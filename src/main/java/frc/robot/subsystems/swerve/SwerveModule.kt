package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.CANcoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkBase.IdleMode.kBrake
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
	val moduleName: String,
) {

	//Motor for controlling the velocity of the wheel
	val driveMotor = HaTalonFX(driveMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configPID(Constants.DRIVE_PID_GAINS)
		
	}

	//Motor for controlling the angle of the wheel
	val steerMotor = HaTalonFX(steerMotorID, Constants.SWERVE_CANBUS).apply {
		restoreFactoryDefaults()
		configurator.apply(Constants.steerMotorFeedbackConfigs(canCoderID, moduleName))
		configPID(Constants.STEER_PID_GAINS)
	}


	val canCoder = CANcoder(canCoderID, Constants.SWERVE_CANBUS)


	//Make the user able to change the idle mode of the motors in the module. Brake by default
	var driveIdleMode: IdleMode = kBrake
		set(value) {
			driveMotor.idleMode = value
			field = value
		}
	var steerIdleMode: IdleMode = kBrake
		set(value) {
			steerMotor.idleMode = value
			field = value
		}


	//The angle setpoint of the swerve module from 0.0 to 360.0 from the right side of the x axis counter clockwise
	private var angleSetpointDeg: Rotation2d = Rotation2d(0.0)

	//The angular velocity setpoint of the wheel in rps
	private var wheelAngularVelocitySetpointRPS: AngularVelocity = AngularVelocity.fromRps(0.0)

	fun setModuleState(swerveModuleState: SwerveModuleState) {
		angleSetpointDeg = swerveModuleState.angle
		wheelAngularVelocitySetpointRPS = AngularVelocity.fromRps(
			swerveModuleState.speedMetersPerSecond / Constants.WHEEL_CIRCUMFERENCE_METERS)
	}


}