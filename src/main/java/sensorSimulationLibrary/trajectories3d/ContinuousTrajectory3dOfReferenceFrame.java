package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.UnitQuaternion;



public interface ContinuousTrajectory3dOfReferenceFrame
    extends ContinuousTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    public UnitQuaternion orientation();  // orientation as a rotation transformation from the reference frame to inertial (qIF)
    
}
