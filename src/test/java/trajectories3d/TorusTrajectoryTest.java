package trajectories3d;

import sensorSimulationLibrary.trajectories3d.TorusTrajectory;

class TorusTrajectoryTest
    extends DifferentiableTrajectoryOfReferenceFrameTester
{
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public TorusTrajectoryTest()
    {
        this.addTrajectoryForTesting( new TorusTrajectory( 50.0 , 20.0 , 3.0 , 1.0 ) );
    }
    
}
