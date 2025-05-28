// package frc.robot.subsystems;

// import com.ctre.phoenix6.SignalLogger;
// import com.ctre.phoenix6.controls.VoltageOut;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.lib.helpers.IDashboardProvider;
// import frc.robot.lib.motor.TalonModule;

// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Volts;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// public class sysidtest extends SubsystemBase {
//     private final VoltageOut voltagRequire = new VoltageOut(0.0);
//     private final SysIdRoutine sysIdRoutine;

//     private TalonModule frontLeft;
//     private TalonModule frontRight;
//     private TalonModule backLeft;
//     private TalonModule backRight;

//     public sysidtest() {
//         // this.frontLeft = new TalonModule(
//         //         2, false);
//         // this.frontRight = new TalonModule(
//         //         4, true);
//         // this.backLeft = new TalonModule(
//         //         6, false);
//         // this.backRight = new TalonModule(
//         //         8, true);

//         sysIdRoutine = new SysIdRoutine(
//                 new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
//                         null, (state) -> SignalLogger.writeString("state", state.toString())),
//                 new SysIdRoutine.Mechanism(
//                         (volts) -> {
//                             this.frontLeft.setControl(voltagRequire.withOutput(volts.in(Volts)));
//                             this.frontRight.setControl(voltagRequire.withOutput(volts.in(Volts)));
//                             this.backLeft.setControl(voltagRequire.withOutput(volts.in(Volts)));
//                             this.backRight.setControl(voltagRequire.withOutput(volts.in(Volts)));
//                         },
//                         null,
//                         this));
//     }

//     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//         return this.sysIdRoutine.quasistatic(direction);
//     }

//     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//         return this.sysIdRoutine.dynamic(direction);
//     }

//     public Command sysIdTest() {
//         return Commands.sequence(
//                 this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
//                         .raceWith(new WaitUntilCommand(2)),
//                 new WaitCommand(2),

//                 this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
//                         .raceWith(new WaitUntilCommand(2)),
//                 new WaitCommand(2),

//                 this.sysIdDynamic(SysIdRoutine.Direction.kForward)
//                         .raceWith(new WaitUntilCommand(2)),
//                 new WaitCommand(2),

//                 this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
//                         .raceWith(new WaitUntilCommand(2)));
//     }
    
//     public Command startCommand() {
//         return Commands.runOnce(SignalLogger::start);
//     }

//     public Command stopCommand() {
//         return Commands.runOnce(SignalLogger::stop);
//     }
// }
