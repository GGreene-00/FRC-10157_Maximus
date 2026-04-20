// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

     // Coral game variables
     private final SparkMax CoralGame;
     private static final int CAN_ID = 15;
     private static final double CORAL_MOTOR_SPEED = -0.3; // Change this value to adjust the motor speed
     private static final double BACK_CORAL = .125;
     private static final double CORAL_AUTO = -0.25;
     private static final double CORAL_STOP = 0;
 
     // endgame variables
     private final SparkMax endGame;
     private static final int CAN_ID2 = 16;
     private static final double ENDGAME_MOTOR_SPEED = 0.55; // Change this value to adjust the motor speed
 
     // slowMode variables
     private boolean slowMode = false;
     private boolean fastMode = false;
 

    public RobotContainer() {

        CoralGame = new SparkMax(CAN_ID, MotorType.kBrushless);
        SparkMaxConfig neoConfig = new SparkMaxConfig();
        neoConfig.inverted(false).idleMode(IdleMode.kCoast);

        endGame = new SparkMax(CAN_ID2, MotorType.kBrushless);
        SparkMaxConfig neoConfig2 = new SparkMaxConfig();
        neoConfig2.inverted(false).idleMode(IdleMode.kBrake);

        NamedCommands.registerCommand("Score", Commands.runOnce(()->{
            CoralGame.set(CORAL_AUTO);
        }));
        
        NamedCommands.registerCommand("Stop", Commands.runOnce(()->{
            CoralGame.set(CORAL_STOP);
        }));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        joystick.povLeft().onTrue(Commands.runOnce(() -> {  // Use onTrue() to toggle slowMode
            slowMode = !slowMode; // Toggle slow mode
            fastMode = false;
            updateDriveModeDashboard();
        }, drivetrain));

        joystick.povRight().onTrue(Commands.runOnce(() -> {
            fastMode = !fastMode; // toggle fast mode
            slowMode = false;
            updateDriveModeDashboard();
        }, drivetrain));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double speedFactor;
                if(slowMode){
                    speedFactor = 0.2;
                }
                else if(fastMode){
                    speedFactor = 1.0;
                }
                else{
                    speedFactor = 0.75;
                }
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed* 0.5 * speedFactor) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5 * speedFactor) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5 * speedFactor); // Drive counterclockwise with negative X (left)
            })
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(Commands.run(() ->{
            CoralGame.set(BACK_CORAL);
    }, drivetrain)).whileFalse(Commands.run(()->{
        CoralGame.set(0);
    }, drivetrain));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        // CoralGame is a motor controller that is controlled by the right bumper
        joystick.rightBumper().onTrue(Commands.run(() -> {
            CoralGame.set(CORAL_MOTOR_SPEED);
        }, drivetrain)).whileFalse(Commands.run(() -> {
            CoralGame.set(0);
        }, drivetrain));

        // Climber goes up
        joystick.povDown().whileTrue(Commands.run(() -> {
           endGame.set(ENDGAME_MOTOR_SPEED);
        }, drivetrain)).whileFalse(Commands.run(() -> {
            endGame.set(0);
        }, drivetrain));
        // Climber goes down
        joystick.povUp().whileTrue(Commands.run(() -> {
            endGame.set(-ENDGAME_MOTOR_SPEED);
        }, drivetrain)).whileFalse(Commands.run(() -> {
            endGame.set(0);
        }, drivetrain));
    }

    private void updateDriveModeDashboard() {
        if (slowMode) {
        SmartDashboard.putString("Drive Mode", "Slow");
        } else if (fastMode) {
        SmartDashboard.putString("Drive Mode", "Fast");
        } else {
        SmartDashboard.putString("Drive Mode", "Normal"); // Or "Default", or whatever you want to call it
        }
     }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}