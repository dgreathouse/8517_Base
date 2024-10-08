// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class OdometryThread extends Thread {
    public OdometryThread() {
        super();

    }

    @Override
    public void run() {

        while (true) {


            
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                System.out.println(e.getMessage());
            }
        }
    }

}
