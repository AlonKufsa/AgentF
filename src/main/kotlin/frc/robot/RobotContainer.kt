package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse
import frc.robot.commands.*
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedSubsystem
import frc.robot.subsystems.loader.LoaderSubsystem
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
	private val mainMotor = CommandPS5Controller(0)
	private val secondaryController = CommandPS4Controller(1)
	private val testController = CommandPS4Controller(5)

	init {
		configureBindings()
		setDefaultCommands()
		sendSubsystemInfo()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {

//		secondaryController.L1().whileTrue(MaintainShooterStateCommand(ShooterState.TO_AMP, true))
//		secondaryController.R1().whileTrue(MaintainShooterStateCommand(ShooterState.AT_SPEAKER, true))
//
//		secondaryController.cross().toggleOnTrue(CollectAndLoadCommand())
//		secondaryController.square()
//			.toggleOnTrue(TransferToShooterCommand().withTimeout(LoaderConstants.TRANSFER_TO_SHOOTER_DURATION))
//		secondaryController.triangle()
//			.toggleOnTrue(LoaderEjectToAmpCommand().withTimeout(LoaderConstants.AMP_EJECT_DURATION))
//
//		secondaryController.L3()
//			.toggleOnTrue(ManualShootingAngleControl({ secondaryController.leftX }, { secondaryController.leftY }))

		secondaryController.circle().whileTrue(sysIdQuasistatic(kForward))
		secondaryController.square().whileTrue(sysIdQuasistatic(kReverse))
		secondaryController.cross().whileTrue(sysIdDynamic(kForward))
		secondaryController.triangle().whileTrue(sysIdDynamic(kReverse))
	}

	private fun setDefaultCommands() {
		IntakeSubsystem.defaultCommand = DefaultIntakeCommand()
		LoaderSubsystem.defaultCommand = DefaultLoaderCommand()
		//ShooterSubsystem.defaultCommand = DefaultShooterCommand()
		ClimbingSubsystem.defaultCommand =
			DefaultClimbingCommand({ secondaryController.leftY }, { secondaryController.leftX })
		SwerveSubsystem.defaultCommand = SwerveSubsystem.resetSwerveSetpoints()
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