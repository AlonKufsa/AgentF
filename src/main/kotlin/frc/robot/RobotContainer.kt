package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import frc.robot.commands.*
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
	private val mainController = CommandPS4Controller(1)

	init {
		configureBindings()
		setDefaultCommands()
		sendSubsystemInfo()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {

		mainController.L1().whileTrue(MaintainShooterStateCommand(ShooterState.TO_AMP, true))
		mainController.R1().whileTrue(MaintainShooterStateCommand(ShooterState.AT_SPEAKER, true))

		mainController.cross().toggleOnTrue(CollectAndLoadCommand())
		mainController.square()
			.toggleOnTrue(TransferToShooterCommand().withTimeout(LoaderConstants.TRANSFER_TO_SHOOTER_DURATION))
		mainController.triangle()
			.toggleOnTrue(LoaderEjectToAmpCommand().withTimeout(LoaderConstants.AMP_EJECT_DURATION))

		mainController.L3()
			.toggleOnTrue(ManualShootingAngleControl({ mainController.leftX }, { mainController.leftY }))

		// Testing
//		mainController.povUp().onTrue(SwerveSubsystem.setSwerveRotation { Rotation2d.fromDegrees(0.0) })
//		mainController.povLeft().onTrue(SwerveSubsystem.setSwerveRotation { Rotation2d.fromDegrees(90.0) })
//		mainController.povDown().onTrue(SwerveSubsystem.setSwerveRotation { Rotation2d.fromDegrees(180.0) })
//		mainController.povRight().onTrue(SwerveSubsystem.setSwerveRotation { Rotation2d.fromDegrees(-90.0) })
		mainController.povUp().whileTrue(SwerveSubsystem.povDriveCommand("Up"))
		mainController.povLeft().whileTrue(SwerveSubsystem.povDriveCommand("Left"))
		mainController.povDown().whileTrue(SwerveSubsystem.povDriveCommand("Down"))
		mainController.povRight().whileTrue(SwerveSubsystem.povDriveCommand("Right"))


		//mainController.L2().whileTrue(SwerveSubsystem.setSwerveDriveVoltage { -mainController.l2Axis * 12.0 })
	}

	private fun setDefaultCommands() {
		IntakeSubsystem.defaultCommand = DefaultIntakeCommand()
		LoaderSubsystem.defaultCommand = DefaultLoaderCommand()
		//SwerveSubsystem.defaultCommand =
		//RobotRelativeSwerveDrive({ mainController.leftX }, { -mainController.leftY }, { mainController.rightX })
		//SwerveSubsystem.defaultCommand = SwerveSubsystem.setSwerveSpeedMPS { mainController.leftY * 3.0 }
		//SwerveSubsystem.defaultCommand = SwerveSubsystem.setSwerveDriveVoltage { mainController.leftY * 6.0 }
		ShooterSubsystem.defaultCommand = DefaultShooterCommand()
//		ClimbingSubsystem.defaultCommand =
//			DefaultClimbingCommand({ mainController.leftY }, { mainController.rightX })
	}

	private fun sendSubsystemInfo() {
		SmartDashboard.putData(ShooterSubsystem)
		SmartDashboard.putData(LoaderSubsystem)
		SmartDashboard.putData(IntakeSubsystem)
		SmartDashboard.putData(LedSubsystem)
		SmartDashboard.putData(SwerveSubsystem)
	}

	fun getAutonomousCommand(): Command? {
		return null
	}
}