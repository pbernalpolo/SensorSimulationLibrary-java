package sensorSimulationLibrary.sensors;


import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;



/**
 * {@link SimulatedSensorWithVector3Output} is a sensor that produces {@link Vector3} measurements.
 */
public interface SimulatedSensorWithVector3Output
{
    ////////////////////////////////////////////////////////////////
    // PUBLIC ABSTRACT METHODS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Returns the {@link Vector3} measurement of the sensor.
     * 
     * @param trf   {@link DifferentiableTrajectory3dOfReferenceFrame} that describes the orientation and position of the navigation frame.
     * @param time  time at which the measurement is taken.
     * @return  measurement of the sensor as a {@link Vector3}.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time );
    
}
