package sensorSimulationLibrary.sensors.triaxials;


import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.sensors.SimulatedSensorWithVector3Output;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;



/**
 * {@link IdealGyroscope} is a sensor that measures angular velocity.
 * <p>
 * The sensor measures the angular velocity with respect to an inertial reference frame expressed in the sensor frame (S): omgSIS
 * The equation that relates the measured angular velocity and the one measured in navigation frame is:
 * omgSIS = R_SN omgNIN
 */
public class IdealGyroscope
	implements SimulatedSensorWithVector3Output
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    
    /**
     * {@link UnitQuaternion} that describes the orientation of the sensor (S frame) relative to the navigation frame (N).
     */
    private UnitQuaternion qNS;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Constructs a {@link IdealGyroscope}.
     */
    public IdealGyroscope()
    {
    	this.qNS = UnitQuaternion.one();
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Sets the orientation of the sensor relative to the navigation frame.
     * 
     * @param unitQuaternionNS	{@link UnitQuaternion} that represents the orientation of the sensor relative to the navigation frame.
     */
    public void setOrientationFromSensorToNavigationFrame( UnitQuaternion unitQuaternionNS )
    {
        this.qNS.setTo( unitQuaternionNS );
    }
    
    
    /**
     * {@inheritDoc}
     * <p>
     * The sensor measures the angular velocity of the sensor with respect to an inertial reference frame expressed in the sensor frame (S): omgSIS
     * The equation that relates the measured angular velocity and the one measured in navigation frame is:
     * omgSIS = R_SN omgNIN
     * 
     * @param trf   {@link ContinuousTrajectory3dOfReferenceFrame} that describes the orientation and position of the navigation frame.
     * @param time  time at which the measurement is taken.
     * @return  measurement of the gyroscope.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time )
    {
        trf.setTime( time );
        return this.qNS.inverseMultiplicative().rotate( trf.angularVelocity() );
    }
    
}
