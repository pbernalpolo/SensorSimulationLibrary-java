package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector3;



public interface ContinuousTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    public void setTime( double theTime );  // sets the time at which the trajectory is evaluated
    public double time();  // gets the time in which the trajectory will be evaluated
    public Vector3 position();  // position measured from origin (rOOF)
    
}
