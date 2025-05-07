// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */

public class Autos {
    static Timer timer = new Timer();

    public static Command driveForward(Drivetrain drive, double speed) {
             timer.reset();
            return Commands.run(() -> {
                
                timer.start();
                if(timer.get() < 2.5){
                drive.driveMecanum(speed, 0, 0);
                } else {
                    drive.driveMecanum(0, 0, 0);
                }
            }, drive);
    }

}
