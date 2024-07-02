package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.subsystems.swerve.SwerveKinematics
import frc.robot.subsystems.swerve.SwerveSubsystem

class SwerveTestCommand(val controller: CommandPS4Controller): Command() {
	init {
		name = "Swerve test command"
		requirements.add(SwerveSubsystem)
	}


	override fun execute() {
		SwerveKinematics.robotRelativeChassisSpeedsToModuleStates(ChassisSpeeds(controller.leftY, controller.leftX, controller.rightX))
	}
}