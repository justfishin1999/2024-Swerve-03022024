package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton runIndex = new JoystickButton(driver, XboxController.Button.kA.value); //A
    private final JoystickButton shootSpeaker = new JoystickButton(driver, XboxController.Button.kX.value); //LB
    private final JoystickButton shootAmp = new JoystickButton(driver, XboxController.Button.kB.value); //RB
    private final JoystickButton ShootREV = new JoystickButton(driver, XboxController.Button.kY.value); //Y
    private final JoystickButton swerveLowSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);; //Shoulder Left
    //private final JoystickButton swerveHighSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);//Shoulder Right
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kBack.value); 
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value); 
    
    /* Operator Controls */
    private final int climberRight = 1;
    private final int climberLeft = 5;

    /* Operator Buttons */
    private final JoystickButton runIndexFWD = new JoystickButton(operator,XboxController.Button.kB.value); //B
    private final JoystickButton runIndexREV = new JoystickButton(operator,XboxController.Button.kX.value); //X

    /*Swerve Speed Multipliers */
    private  double translationMultiplier = 1;
    private  double strafeMultiplier = 1;
    private  double rotateMultiplier = .5;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Indexer s_Indexer = new Indexer();
    private final Climber s_Climber = new Climber();

    /*Auto Builder Auto Chooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        /* Sets the Default command for TeleopSwerve Subsystem */
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)*translationMultiplier, 
                () -> -driver.getRawAxis(strafeAxis)*strafeMultiplier, 
                () -> -driver.getRawAxis(rotationAxis)*rotateMultiplier, 
                () -> zeroGyro.getAsBoolean()

            )
        );
        /* Sets the Default Command for Climber Subsystem */
        s_Climber.setDefaultCommand(
            new Climb(
                s_Climber,
                () -> operator.getRawAxis (climberRight)*1,
                () -> operator.getRawAxis (climberLeft)*-1
            )
        );
  /*Auto Builder needs the named commands to be registered before the creation of any Planner autos or paths */
    NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(s_Shooter,s_Indexer,Constants.ShooterConstants.combined_shooterVelo,Constants.IndexerConstants.indexVeloAmp).withTimeout(1));
    NamedCommands.registerCommand("RunIndex", new RunIndexer(s_Indexer,Constants.IndexerConstants.IndexVeloFWD));
    NamedCommands.registerCommand("StopShooter", new StopShooter(s_Shooter,s_Indexer));
    NamedCommands.registerCommand("StopIndex",new RunIndexer(s_Indexer, 0));
    NamedCommands.registerCommand("RunIndexRev",new RunIndexer(s_Indexer, Constants.IndexerConstants.indexVeloREV));

    /*Auto Builder This builds an auto chooser uses none put name of default auto in () in wanted */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    // Configure the button bindings
    configureButtonBindings();
    }

    private void configureBindings(){
        /*SmartDashboard.putData("Left Auto",new PathPlannerAuto("A1"));
        SmartDashboard.putData("Middle Auto",new PathPlannerAuto("B1"));
        SmartDashboard.putData("Right Auto",new PathPlannerAuto("C1"));*/
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        robotCentric.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        /*Create binding for shooting speaker */
        shootSpeaker.whileTrue(new ShootSpeaker(s_Shooter, s_Indexer,Constants.ShooterConstants.combined_shooterVelo,Constants.IndexerConstants.indexVeloSpeaker));
        shootSpeaker.whileFalse(new ShootSpeaker(s_Shooter,s_Indexer, 0,0));
        shootSpeaker.whileFalse(new StopShooter(s_Shooter, s_Indexer)); 

        /*Create binding for shooting amp */
        shootAmp.whileTrue(new ShootAmp(s_Shooter, s_Indexer,Constants.ShooterConstants.top_shooterVelo,Constants.ShooterConstants.bottom_shooterVelo, Constants.IndexerConstants.indexVeloAmp));
        shootAmp.whileFalse(new ShootAmp(s_Shooter,s_Indexer, 0,0,0));
        shootAmp.whileFalse(new StopShooter(s_Shooter, s_Indexer)); 

        /*Create binding for shooter running in reverse */
        ShootREV.whileTrue(new ShootRev(s_Shooter,Constants.ShooterConstants.rev_shooterVelo));
        ShootREV.whileFalse(new ShootRev(s_Shooter,0));
        
        /*Create binding for running indexer */
        runIndex.whileTrue(new RunIndexer(s_Indexer, Constants.IndexerConstants.indexVelo));
        runIndex.whileFalse(new RunIndexer(s_Indexer, 0));
        
        runIndexFWD.whileTrue(new RunIndexerFWD(s_Indexer, Constants.IndexerConstants.IndexVeloFWD));
        runIndexFWD.whileFalse(new RunIndexerFWD(s_Indexer, 0));
        
        runIndexREV.whileTrue(new RunIndexerREV(s_Indexer, Constants.IndexerConstants.indexVeloREV));
        runIndexREV.whileFalse(new RunIndexerREV(s_Indexer, 0));

        /*Create binding for swerve speed */

        swerveLowSpeed.whileTrue(new InstantCommand(() -> translationMultiplier = .5));
        swerveLowSpeed.whileFalse(new InstantCommand(() -> translationMultiplier = 1));

        swerveLowSpeed.whileTrue(new InstantCommand(() -> strafeMultiplier = .5));
        swerveLowSpeed.whileFalse(new InstantCommand(() -> strafeMultiplier = 1));

        swerveLowSpeed.whileTrue(new InstantCommand(() -> rotateMultiplier = .5));
        swerveLowSpeed.whileFalse(new InstantCommand(() -> rotateMultiplier = .5));
    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /* Auto Builder gets the selected autochooser */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
