package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap.SwerveMap
import frc.robot.vision.AprilTagVision
import frc.robot.vision.VisionConstants
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
	private val pigeon = Pigeon2(SwerveMap.PIGEON_2_ID, Constants.SWERVE_CANBUS)

	/** FR, FL, BL, BR*/
	private val modules = arrayOf(frontRight, frontLeft, backLeft, backRight)

	private val swerveDriveKinematics = SwerveDriveKinematics(
		Translation2d(Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, Constants.MODULE_OFFSET),
		Translation2d(-Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET),
		Translation2d(Constants.MODULE_OFFSET, -Constants.MODULE_OFFSET)
	)

	private val poseEstimator =
		SwerveDrivePoseEstimator(swerveDriveKinematics,
			angle,
			currentSwervePositionsArray,
			Constants.STARTING_POS
		)

	private val angle: Rotation2d
		get() = pigeon.rotation2d
	private val currentSwervePositionsArray: Array<SwerveModulePosition>
		get() {
			return arrayOf(
				frontRight.position,
				frontLeft.position,
				backLeft.position,
				backRight.position
			)
		}
	private val field = Field2d()

	private fun getRobotRelativeSpeeds() = SwerveKinematics.currentRobotRelativeChassisSpeeds
	private fun getFieldRelativeSpeeds() = SwerveKinematics.currentFieldRelativeChassisSpeeds
	private fun getEstimatedPose() = poseEstimator.estimatedPosition


	// Driving methods
	/** Use externally only for testing */
	fun setModuleStates(moduleStates: ModuleStates) {
		for (i in 0..3) {
			modules[i].setModuleState(moduleStates.array[i])
		}
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

	fun fieldRelativeDrive(chassisSpeeds: ChassisSpeeds) {
		val moduleStates =
			SwerveKinematics.fieldRelativeChassisSpeedsToModuleStates(chassisSpeeds,
				Constants.MAX_SPEED_MPS,
				-angle)

		setModuleStates(moduleStates)
	}

	// Gyro
	fun resetGyro() {
		val pose = getEstimatedPose()
		pigeon.reset()
		poseEstimator.resetPosition(Rotation2d(),
			currentSwervePositionsArray,
			Pose2d(pose.x, pose.y, Rotation2d(0.0)))
	}

	fun setGyro(newAngle: Rotation2d) {
		val pose = getEstimatedPose()
		pigeon.setYaw(newAngle.degrees)
		poseEstimator.resetPosition(newAngle, currentSwervePositionsArray, Pose2d(pose.x, pose.y, newAngle))
	}

	// Odometry
	// FR, FL, BL, BR
	private fun resetOdometry(pose2d: Pose2d) {
		poseEstimator.resetPosition(angle, currentSwervePositionsArray, pose2d)
	}


	// Pose estimation
	private fun applyVisionMeasurement() {
		if (VisionConstants.useVisionPoseEstimation) {
			val pose = AprilTagVision.estimatedGlobalPose
			field.getObject("vision-pos").pose = pose?.estimatedPose?.toPose2d()
			if (pose != null && AprilTagVision.isInRange) {
				val pose2d = pose.estimatedPose.toPose2d().let { Pose2d(it.x, it.y, angle) }
				poseEstimator.addVisionMeasurement(pose2d, pose.timestampSeconds, AprilTagVision.poseEstimationStdDevs)
			}
		}
	}


	// Periodic method
	override fun periodic() {
		if (AprilTagVision.isConnected) {
			applyVisionMeasurement()
		}
		field.robotPose = poseEstimator.update(angle, currentSwervePositionsArray)
		val visionPose = AprilTagVision.estimatedGlobalPose
	}

	// Autonomous
	init {
		AutoBuilder.configureHolonomic(
			this::getEstimatedPose,
			this::resetOdometry,
			this::getRobotRelativeSpeeds,
			this::robotRelativeDrive,
			Constants.PP_CONFIGS,
			{
				Robot.getAlliance() == Alliance.Red
			},
			this
		)
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

	init {
		SmartDashboard.putData("Robot pose", field)
	}
}