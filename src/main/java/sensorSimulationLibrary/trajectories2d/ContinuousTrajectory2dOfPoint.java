package sensorSimulationLibrary.trajectories2d;


import numericalLibrary.types.Vector2;



public interface ContinuousTrajectory2dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    public void setTime( double theTime );  // sets the time at which the trajectory is evaluated
    public double time();  // gets the time in which the trajectory will be evaluated
    public Vector2 position();  // position measured from origin (rOOF)
    
}
