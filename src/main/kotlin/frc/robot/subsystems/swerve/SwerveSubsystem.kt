package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.SwerveMap
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase("Swerve subsystem") {
	private val frontRight = SwerveModule(
		SwerveMap.FrontRight.DRIVE_MOTOR_ID,
		SwerveMap.FrontRight.STEER_MOTOR_ID,
		SwerveMap.FrontRight.CANCODER_ID,
		"FrontRight",
		invertedDrive = true,
		invertedSteer = false
	)
	private val frontLeft = SwerveModule(
		SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
		SwerveMap.FrontLeft.STEER_MOTOR_ID,
		SwerveMap.FrontLeft.CANCODER_ID,
		"FrontLeft",
		invertedDrive = true,
		invertedSteer = false
	)
	private val backLeft = SwerveModule(
		SwerveMap.BackLeft.DRIVE_MOTOR_ID,
		SwerveMap.BackLeft.STEER_MOTOR_ID,
		SwerveMap.BackLeft.CANCODER_ID,
		"BackLeft",
		invertedDrive = true,
		invertedSteer = false
	)
	private val backRight = SwerveModule(
		SwerveMap.BackRight.DRIVE_MOTOR_ID,
		SwerveMap.BackRight.STEER_MOTOR_ID,
		SwerveMap.BackRight.CANCODER_ID,
		"BackRight",
		invertedDrive = true,
		invertedSteer = false
	)

	/** FR, FL, BL, BR*/
	private val modules = listOf(frontRight, frontLeft, backLeft, backRight)

	/** Use externally only for testing */
	fun setModuleStates(moduleStates: ModuleStates) {
		frontRight.setModuleState(moduleStates.frontRight)
		frontLeft.setModuleState(moduleStates.frontLeft)
		backLeft.setModuleState(moduleStates.backLeft)
		backRight.setModuleState(moduleStates.backRight)
	}

	fun setRotation(rotation: Rotation2d) {
		for (module in modules) {
			module.setModuleRotation(rotation)
		}
	}

	fun setSpeed(speedMPS: Double) {
		for (module in modules) {
			module.setModuleSpeed(speedMPS)
		}
	}

	fun spinCounterClockwise(velocity: AngularVelocity) {
		val moduleStates =
			SwerveKinematics.angularVelocityToModuleStates(velocity)

		setModuleStates(moduleStates)
	}

	fun robotRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(chassisSpeeds, Constants.MAX_SPEED_MPS)

		setModuleStates(moduleStates)
	}


	// Gyro and IMU

	private val pigeon = Pigeon2(SwerveMap.PIGEON_2_ID, Constants.SWERVE_CANBUS)

	private val angle: Rotation2d
		get() = pigeon.rotation2d

	fun resetPigeon() {
		pigeon.reset()
	}


	fun fieldRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			SwerveKinematics.fieldRelativeChassisSpeedsToModuleStates(chassisSpeeds,
				Constants.MAX_SPEED_MPS,
				angle)

		setModuleStates(moduleStates)
	}


	// Odometry

	// FR, FL, BL, BR
	private fun getCurrentSwervePositionsArray(): Array<SwerveModulePosition> {
		return arrayOf(
			frontRight.position,
			frontLeft.position,
			backLeft.position,
			backRight.position
		)
	}


	private val swerveDriveKinematics = SwerveDriveKinematics(
		Translation2d(Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET),
		Translation2d(Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET)
	)
	private val swerveDriveOdometry = SwerveDriveOdometry(
		swerveDriveKinematics,
		angle,
		getCurrentSwervePositionsArray(),
	)

	fun resetOdometry(newPose: Pose2d) {
		swerveDriveOdometry.resetPosition(angle, getCurrentSwervePositionsArray(), newPose)
	}


	// Pose estimation
	var pose = Pose2d()
		private set


	private val poseEstimator =
		SwerveDrivePoseEstimator(swerveDriveKinematics, angle, getCurrentSwervePositionsArray(), Pose2d())


	// Periodic method

	override fun periodic() {
		pose = poseEstimator.update(angle, getCurrentSwervePositionsArray())
		field.robotPose = pose
	}


	// Testing

	fun setSteerVoltage(voltage: Volts) {
		for (module in modules) {
			module.setSteerVoltage(voltage)
		}
	}

	fun setDriveVoltage(voltage: Volts) {
		for (module in modules) {
			module.setDriveVoltage(voltage)
		}
	}

	fun resetAllModules() {
		for (module in modules) {
			module.setModuleState(SwerveModuleState(0.0, Rotation2d(0.0)))
		}
	}

	// Logging
	override fun initSendable(builder: SendableBuilder) {
		for (module in modules) {
			module.addModuleInfo(builder)
		}
		builder.addDoubleProperty("Robot yaw deg", { pigeon.angle }, null)
	}

	val field = Field2d()

	init {
		SmartDashboard.putData("Robot pose", field)
	}
}