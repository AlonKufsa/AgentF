package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.subsystems.swerve.SwerveKinematics
import frc.robot.subsystems.swerve.SwerveSubsystem

class SwerveTestCommand(val controller: CommandPS4Controller) : Command() {
	init {
		name = "Swerve test command"
		requirements.add(SwerveSubsystem)
	}


	override fun execute() {
		SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(ChassisSpeeds(controller.leftY,
			controller.leftX,
			controller.rightX))
	}
}

fun sysIdQuasistatic(direction: Direction): Command = SwerveSubsystem.routine.quasistatic(direction)

fun sysIdDynamic(direction: Direction): Command = SwerveSubsystem.routine.dynamic(direction)

fun SwerveSubsystem.resetSwerveSetpoints(): Command {
	return run {
		//setModuleStates(
		//ModuleStates(SwerveModuleState(0.0, Rotation2d())))
		setRotation()
	}
}