package sensorSimulationLibrary.sensors.triaxials;


import kalmanFilterLibrary.gravityModels.GravityModel;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.sensors.SimulatedSensorWithVector3Output;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;



/**
 * {@link IdealAccelerometer} is a sensor that measures acceleration.
 * <p>
 * The sensor measures the acceleration with respect to an inertial reference frame expressed in the sensor frame (S): accSIS
 * The equation that relates the measured acceleration and the one measured in navigation frame is:
 * accSIS = R( qSN ) [ R( qNO ) ( accOON - gOS ) + omgNIN x ( omgNIN x rNNS ) ]  (angular acceleration is currently neglected)
 */
public class IdealAccelerometer
	implements SimulatedSensorWithVector3Output
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    
    /**
     * {@link UnitQuaternion} that represents the orientation of the sensor (S frame) relative to the navigation frame (N).
     */
    private UnitQuaternion qNS;
    
    /**
     * {@link Vector3} that describes the position of the sensor (S frame) with respect to the navigation frame (N) expressed in the navigation frame (N).
     */
    private Vector3 rNNS;
    
    /**
     * {@link GravityModel} used to produce IMU measurements.
     */
    private GravityModel gravityModel;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Constructs a {@link IdealAccelerometer}.
     */
    public IdealAccelerometer()
    {
    	this.qNS = UnitQuaternion.one();
    	this.rNNS = Vector3.zero();
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
     * Sets the position of the sensor with respect to the navigation frame expressed in the navigation frame.
     * 
     * @param positionNNS	position of the sensor with respect to the navigation frame expressed in the navigation frame.
     */
    public void setPositionOfSensorFromNavigationFrame( Vector3 positionNNS )
    {
    	this.rNNS.setTo( positionNNS );
    }
    
    
    /**
     * Sets the {@link GravityModel} used to produce IMU measurements.
     * 
     * @param gravityModel	{@link GravityModel} to be used to produce IMU measurements.
     */
    public void setGravityModel( GravityModel gravityModel )
    {
    	this.gravityModel = gravityModel;
    }
    
    
    /**
     * {@inheritDoc}
     * <p>
     * The sensor measures the angular velocity of the sensor with respect to an inertial reference frame expressed in the sensor frame (S): omgSIS
     * The equation that relates the measured angular velocity and the one measured in navigation frame is:
     * accSIS = R( qSN ) [ R( qNO ) ( accOON - gOS ) + omgNIN x ( omgNIN x rNNS ) ]  (angular acceleration is currently neglected)
     * 
     * groundTruthTrajectory.orientation().rotateWithInverse( groundTruthTrajectory.acceleration().subtract( gravityModel.acceleration() ) )
     * 
     * @param trf   {@link ContinuousTrajectory3dOfReferenceFrame} that describes the orientation and position of the navigation frame.
     * @param time  time at which the measurement is taken.
     * @return  measurement of the gyroscope.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time )
    {
        trf.setTime( time );
        // rOOS = rOON + R( qON ) rNNS
        Vector3 rOOS = trf.position().add( trf.orientation().rotate( this.rNNS ) );
        this.gravityModel.setPosition( rOOS );
        UnitQuaternion qON = trf.orientation();
        Vector3 accNIN = trf.acceleration();
        Vector3 omgNIN = trf.angularVelocity();
        Vector3 accCentripetal = omgNIN.crossProduct( omgNIN.crossProduct( this.rNNS ) );
        // accSIS = R( qSN ) [ R( qNO ) ( accOON - gOS ) + omgNIN x ( omgNIN x rNNS ) ]  (angular acceleration is currently neglected)
        return this.qNS.rotateWithInverse(
        		qON.rotateWithInverse( accNIN.subtract( this.gravityModel.acceleration() ) )
        		.addInplace( accCentripetal ) );
    }
    
}
