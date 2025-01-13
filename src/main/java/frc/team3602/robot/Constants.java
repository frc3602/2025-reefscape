/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

public final class Constants {
    public final class OperatorInterfaceConstants {
        public final static int kXboxControllerPort = 0;
        public final static int kControlPanelPort = 1;
    }

    public final class GripperConstants {

        //pivot PID constants
        public final static double pivotKP = 1;
        public final static double pivotKI = 0;
        public final static double pivotKD = 0;

        //pivot ffe constants
        public final static double pivotKS = 5;
        public final static double pivotKG = 2;
        public final static double pivotKV = 0.9;
        public final static double pivotKA = 0.1;

         // sim pivot PID constants
         public final static double simPivotKP = 9;
         public final static double simPivotKI = 0;
         public final static double simPivotKD = 0;
 
         // sim pivot ffe constants
         public final static double simPivotKS = 5;
         public final static double simPivotKG = 2;
         public final static double simPivotKV = 0.9;
         public final static double simPivotKA = 0.1;

         //sim constants
         public final static int pivotGearing = 36;
         public final static double pivotLengthMeters = 0.5;
         public final static double pivotMassKg = 3;


    }
}
