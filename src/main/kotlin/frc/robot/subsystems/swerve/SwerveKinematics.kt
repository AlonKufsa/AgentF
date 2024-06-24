package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import kotlin.math.PI

object SwerveKinematics {
	/** Converts between angular velocity of the robot to the module states needed to achieve it.
	 * Positive angular velocity is counterclockwise, negative is clockwise */
	fun angularVelocityToModuleStates(angularVelocity: AngularVelocity): ModuleStates {
		val wheelSpeedMPS: Double = angularVelocity.asRps * 2 * PI * SwerveConstants.DRIVEBASE_RADIUS.asMeters

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
		return ModuleStates(
			SwerveModuleState(velocity.norm, velocity.angle),
			SwerveModuleState(velocity.norm, velocity.angle),
			SwerveModuleState(velocity.norm, velocity.angle),
			SwerveModuleState(velocity.norm, velocity.angle)
		)
	}


}