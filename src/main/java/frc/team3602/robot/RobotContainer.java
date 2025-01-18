package frc.team3602.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.generated.TunerConstants;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.GripperSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.Camera;

import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Operator interfaces */
    private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
    // private final CommandXboxController controlPanel = new CommandXboxController(kControlPanelPort);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);

    /* Subsystems */
    public final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
    private final PivotSubsystem pivotSubsys = new PivotSubsystem();
    private final GripperSubsystem gripperSubsys = new GripperSubsystem(elevatorSubsys, () -> pivotSubsys.pivotMotor.getMotorVoltage().getValueAsDouble());

    private final Camera mod0Camera = new Camera(kMod0CameraName, kRobotToMod0CameraTransform);
    private final Camera mod1Camera = new Camera(kMod1CameraName, kRobotToMod1CameraTransform);
    private final Camera mod2Camera = new Camera(kMod2CameraName, kRobotToMod2CameraTransform);
    private final Camera mod3Camera = new Camera(kMod1CameraName, kRobotToMod3CameraTransform);

    /* Autonomous */
    private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

    public RobotContainer() {
        configDefaultCommands();
        configButtonBindings();
        configAutonomous();
    }

    /**
     * Function that is called in the constructor where we configure default
     * commands for the subsytems.
     */
    private void configDefaultCommands() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrainSubsys.setDefaultCommand(
                drivetrainSubsys.applyRequest(() ->
                    drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    }

    /**
     * Function that is called in the constructor where we configure operator
     * interface button bindings.
     */
    private void configButtonBindings() {

        if (Utils.isSimulation()){
            drivetrainSubsys.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrainSubsys.applyRequest(() ->
                    drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            
            joystick.button(1).whileTrue(gripperSubsys.testGripperWheel(12.0)); 
            joystick.button(2).whileTrue(gripperSubsys.testGripperWheel(-12.0));
            joystick.button(3).whileTrue(elevatorSubsys.testElevator(5.0));
            joystick.button(4).whileTrue(elevatorSubsys.testElevator(-5.0));
            joystick2.button(1).onTrue(pivotSubsys.testMotionMagic(-90));
            joystick2.button(2).onTrue(pivotSubsys.testMotionMagic(0));
            joystick2.button(3).onTrue(pivotSubsys.testMotionMagic(90));
            joystick2.button(4).onTrue(pivotSubsys.testMotionMagic(150));
        } else {

        xboxController.a().whileTrue(drivetrainSubsys.applyRequest(() -> brake));
        xboxController.b().whileTrue(drivetrainSubsys.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xboxController.back().and(xboxController.y()).whileTrue(drivetrainSubsys.sysIdDynamic(Direction.kForward));
        xboxController.back().and(xboxController.x()).whileTrue(drivetrainSubsys.sysIdDynamic(Direction.kReverse));
        xboxController.start().and(xboxController.y()).whileTrue(drivetrainSubsys.sysIdQuasistatic(Direction.kForward));
        xboxController.start().and(xboxController.x()).whileTrue(drivetrainSubsys.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xboxController.leftBumper().onTrue(drivetrainSubsys.runOnce(() -> drivetrainSubsys.seedFieldCentric()));

        drivetrainSubsys.registerTelemetry(logger::telemeterize);
        }
    }

    /**
     * Function that returns the currently selected autonomous routine in the
     * SendableChooser.
     * 
     * @return Currently selected autonomous routine.
     */
    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();
    }

    /**
     * Function that is called in the constructor where we configure anything
     * relating to autonomous.
     */
    private void configAutonomous() {
        SmartDashboard.putData(sendableChooser);
    }
}
