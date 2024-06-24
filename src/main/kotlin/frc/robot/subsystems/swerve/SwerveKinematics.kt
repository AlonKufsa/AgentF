package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.AngularVelocity.Companion
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import kotlin.math.PI
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveKinematics {
	/** Converts between angular velocity of the robot to the module states needed to achieve it.
	 * Positive angular velocity is counterclockwise, negative is clockwise */
	fun angularVelocityToModuleStates(angularVelocity: AngularVelocity): ModuleStates {
		val wheelSpeedMPS: Double = angularVelocity.asRps * 2 * PI * Constants.DRIVEBASE_RADIUS.asMeters

		val moduleStates = ModuleStates()
		if (angularVelocity.asRps < 0) {
			moduleStates.setStates(
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(315.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(45.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(135.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(225.0))
			)
		} else {
			moduleStates.setStates(
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(135.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(225.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(315.0)),
				SwerveModuleState(wheelSpeedMPS, Rotation2d.fromDegrees(45.0))
			)
		}
		return moduleStates
	}

	/** Converts between a velocity in some direction to the module states needed to achieve it. */
	fun velocityToModuleStates(velocity: Translation2d): ModuleStates {
		//The Translation2d.angle sometimes gives negative angles.
		val modulatedAngle = Rotation2d.fromDegrees(MathUtil.inputModulus(velocity.angle.degrees, 0.0, 360.0))
		return ModuleStates(
			SwerveModuleState(velocity.norm, modulatedAngle),
			SwerveModuleState(velocity.norm, modulatedAngle),
			SwerveModuleState(velocity.norm, modulatedAngle),
			SwerveModuleState(velocity.norm, modulatedAngle)
		)
	}

	private fun moduleStateToTranslation2d(moduleState: SwerveModuleState): Translation2d {
		return Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
	}

	/** Converts between chassis speeds and module states.
	 * Positive chassis speeds omega results in counterclockwise rotation.*/
	fun chassisSpeedsToModuleStates(chassisSpeeds: ChassisSpeeds): ModuleStates {
		val velocity = Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)

		val velocityModuleStates = velocityToModuleStates(velocity)
		val rotationModuleStates = angularVelocityToModuleStates(AngularVelocity.fromRadPs(chassisSpeeds.omegaRadiansPerSecond))

		val frontRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.frontRight) + moduleStateToTranslation2d(rotationModuleStates.frontRight)
		val frontLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.frontLeft) + moduleStateToTranslation2d(rotationModuleStates.frontLeft)
		val backLeftCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.backLeft) + moduleStateToTranslation2d(rotationModuleStates.backLeft)
		val backRightCombined: Translation2d =
			moduleStateToTranslation2d(velocityModuleStates.backRight) + moduleStateToTranslation2d(rotationModuleStates.backRight)

		return ModuleStates(
			SwerveModuleState(frontRightCombined.norm, frontRightCombined.angle),
			SwerveModuleState(frontLeftCombined.norm, frontLeftCombined.angle),
			SwerveModuleState(backLeftCombined.norm, backLeftCombined.angle),
			SwerveModuleState(backRightCombined.norm, backRightCombined.angle)
		)
	}


}