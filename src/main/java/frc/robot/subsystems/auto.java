package frc.robot.subsystems;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class auto extends TimedRobot {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final AutoFactory autoFactory;

    public auto(){
        autoFactory = new AutoFactory(
            swerveSubsystem::getPose, // A function that returns the current robot pose
            swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
        );
    }
    public Command testauto() {
    return Commands.sequence(
        autoFactory.resetOdometry("reef(1)"),
        Commands.deadline(
            autoFactory.trajectoryCmd("reef(1)")
        )
    );
}

}
