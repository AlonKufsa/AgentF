package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingSubsystem

class DefaultClimbingCommand(val leftY: () -> Double, val rightY: () -> Double) : Command() {
	init {
		name = "Climbing default command"
		addRequirements(ClimbingSubsystem)
	}

	override fun execute() {
		val leftYVal = leftY()
		val rightYVal = rightY()
		if (leftYVal > 0.5) {
			ClimbingSubsystem.openLeft()
		} else if (leftYVal < -0.5) {
			ClimbingSubsystem.closeLeft()
		} else {
			ClimbingSubsystem.stopLeft()
		}

		if (rightYVal > 0.5) {
			ClimbingSubsystem.openRight()
		} else if (rightYVal < -0.5) {
			ClimbingSubsystem.closeRight()
		} else {
			ClimbingSubsystem.stopRight()
		}
	}
}