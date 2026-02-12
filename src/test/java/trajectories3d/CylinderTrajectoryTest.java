package trajectories3d;

import sensorSimulationLibrary.trajectories3d.CylinderTrajectory;

class CylinderTrajectoryTest
    extends DifferentiableTrajectoryOfReferenceFrameTester
{

    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public CylinderTrajectoryTest()
    {
        this.addTrajectoryForTesting( new CylinderTrajectory( 50.0 , 20.0 , 1.0 , 3.0 ) );
    }

}
