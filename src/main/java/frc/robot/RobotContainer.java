package frc.robot;



import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import static frc.robot.Constants.OperatorInterfaceConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Camera;

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

    private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
    // private final CommandXboxController controlPanel = new CommandXboxController(kControlPanelPort);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandJoystick joystick2 = new CommandJoystick(1);
    public final DrivetrainSubsystem drivetrainSubsystem = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
    private final GripperSubsystem gripperSubsys = new GripperSubsystem(elevatorSubsys);

    private final Camera mod0Camera = new Camera(kMod0CameraName, kRobotToMod0CameraTransform);
    private final Camera mod1Camera = new Camera(kMod1CameraName, kRobotToMod1CameraTransform);
    private final Camera mod2Camera = new Camera(kMod2CameraName, kRobotToMod2CameraTransform);
    private final Camera mod3Camera = new Camera(kMod1CameraName, kRobotToMod3CameraTransform);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        if (Utils.isSimulation()){
            drivetrainSubsystem.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrainSubsystem.applyRequest(() ->
                    drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            
            joystick.button(1).whileTrue(gripperSubsys.testGripperWheel()); 
            joystick.button(2).whileTrue(gripperSubsys.testGripperWheelNegative());
            joystick.button(3).whileTrue(elevatorSubsys.testElevator());
            joystick.button(4).whileTrue(elevatorSubsys.testElevatorNegative());
            joystick2.button(1).onTrue(gripperSubsys.testMotionMagic(-90));
            joystick2.button(2).onTrue(gripperSubsys.testMotionMagic(0));
            joystick2.button(3).onTrue(gripperSubsys.testMotionMagic(90));
            joystick2.button(4).onTrue(gripperSubsys.testMotionMagic(150));
        } else {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrainSubsystem.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrainSubsystem.applyRequest(() ->
                drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        xboxController.a().whileTrue(drivetrainSubsystem.applyRequest(() -> brake));
        xboxController.b().whileTrue(drivetrainSubsystem.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        xboxController.back().and(xboxController.y()).whileTrue(drivetrainSubsystem.sysIdDynamic(Direction.kForward));
        xboxController.back().and(xboxController.x()).whileTrue(drivetrainSubsystem.sysIdDynamic(Direction.kReverse));
        xboxController.start().and(xboxController.y()).whileTrue(drivetrainSubsystem.sysIdQuasistatic(Direction.kForward));
        xboxController.start().and(xboxController.x()).whileTrue(drivetrainSubsystem.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        xboxController.leftBumper().onTrue(drivetrainSubsystem.runOnce(() -> drivetrainSubsystem.seedFieldCentric()));

        drivetrainSubsystem.registerTelemetry(logger::telemeterize);
        }
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
