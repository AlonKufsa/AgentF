package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.PI
import kotlin.math.max
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveKinematics {

	/**
	 * Converts between angular velocity of the robot to the module states needed to achieve it.
	 * Positive angular velocity is counterclockwise, negative is clockwise.
	 */
	fun angularVelocityToModuleStates(angularVelocity: AngularVelocity): ModuleStates {
		val wheelSpeedMPS: Double = angularVelocity.asRps * 2 * PI * Constants.DRIVEBASE_RADIUS.asMeters

		val moduleStates = ModuleStates()

		moduleStates.setStates(
			SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(135.0)),
			SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(-135.0)),
			SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(-45.0)),
			SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(45.0)),
		)
		return moduleStates
	}

	/** Converts between a velocity in some direction to the module states needed to achieve it. */
	fun robotRelativeVelocityToModuleStates(velocity: Translation2d): ModuleStates {
		return ModuleStates(
			SwerveModuleState(velocity.norm, velocity.angle)
		)
	}

	private fun moduleStateToTranslation2d(moduleState: SwerveModuleState): Translation2d {
		return Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
	}

	/** Converts between chassis speeds and module states.
	 * Positive chassis speeds omega results in counterclockwise rotation.*/
	fun robotRelativeChassisSpeedsToModuleStates(chassisSpeeds: ChassisSpeeds, maxSpeedMPS: Double): ModuleStates {
		val velocity = Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)

		val velocityModuleStates = robotRelativeVelocityToModuleStates(velocity)
		val rotationModuleStates =
			angularVelocityToModuleStates(AngularVelocity.fromRadPs(chassisSpeeds.omegaRadiansPerSecond))

		val frontRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.frontRight) + moduleStateToTranslation2d(
				rotationModuleStates.frontRight)
		val frontLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.frontLeft) + moduleStateToTranslation2d(rotationModuleStates.frontLeft)
		val backLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.backLeft) + moduleStateToTranslation2d(rotationModuleStates.backLeft)
		val backRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.backRight) + moduleStateToTranslation2d(rotationModuleStates.backRight)

		SmartDashboard.putNumberArray("Wanted module states", arrayOf(
			frontRightCombined.angle.degrees, frontRightCombined.norm,
			frontLeftCombined.angle.degrees, frontLeftCombined.norm,
			backLeftCombined.angle.degrees, backLeftCombined.norm,
			backRightCombined.angle.degrees, backRightCombined.norm
		))

		val moduleStates = ModuleStates(
			SwerveModuleState(frontRightCombined.norm, frontRightCombined.angle),
			SwerveModuleState(frontLeftCombined.norm, frontLeftCombined.angle),
			SwerveModuleState(backLeftCombined.norm, backLeftCombined.angle),
			SwerveModuleState(backRightCombined.norm, backRightCombined.angle),
		)
		return factorModuleStates(maxSpeedMPS, moduleStates)
	}

	fun factorModuleStates(maxSpeedMPS: Double, moduleStates: ModuleStates): ModuleStates {
		if (max(max(moduleStates.frontRight.speedMetersPerSecond, moduleStates.frontLeft.speedMetersPerSecond),
				max(moduleStates.backRight.speedMetersPerSecond,
					moduleStates.backLeft.speedMetersPerSecond)) > maxSpeedMPS && maxSpeedMPS != 0.0
		) {
			val highestModuleSpeed =
				max(max(moduleStates.frontRight.speedMetersPerSecond, moduleStates.frontLeft.speedMetersPerSecond),
					max(moduleStates.backRight.speedMetersPerSecond, moduleStates.backLeft.speedMetersPerSecond))
			val factor = maxSpeedMPS / highestModuleSpeed

			moduleStates.frontRight.speedMetersPerSecond *= factor
			moduleStates.frontLeft.speedMetersPerSecond *= factor
			moduleStates.backLeft.speedMetersPerSecond *= factor
			moduleStates.backRight.speedMetersPerSecond *= factor
		}
		return moduleStates
	}
}