package sensorSimulationLibrary.sensors.triaxials;


import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.sensors.SimulatedSensorWithVector3Output;
import sensorSimulationLibrary.trajectories3d.DifferentiableTrajectory3dOfReferenceFrame;



/**
 * {@link IdealMagnetometer} is a sensor that measures the magnetic field.
 * <p>
 * The sensor measures the magnetic field at the sensor position expressed in the sensor frame (S): mSS
 * The magnetic field value is known at the sensor position and is expressed in the origin frame (O): mOS
 * The equation that relates both quantities is:
 * mSS = R_SN R_NO mOS
 */
public class IdealMagnetometer
	implements SimulatedSensorWithVector3Output
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    
    /**
     * {@link UnitQuaternion} that describes the orientation of the sensor (S frame) relative to the navigation frame (N).
     */
    private UnitQuaternion qNS;
    
    /**
     * Magnetic field at the sensor position (S frame) expressed in the origin (O frame).
     */
    private Vector3 mOS;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    /**
     * Constructs a {@link IdealMagnetometer}.
     */
    public IdealMagnetometer()
    {
        this.qNS = UnitQuaternion.one();
        this.mOS = Vector3.zero();
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
     * Sets the magnetic field vector at the sensor position (S frame) expressed in origin frame (O).
     * 
     * @param magneticField     magnetic field vector at the sensor position (S frame) expressed in origin frame (O).
     */
    public void setMagneticFieldExpressedInOrigin( Vector3 magneticField )
    {
        this.mOS.setTo( magneticField );
    }
    
    
    /**
     * {@inheritDoc}
     * <p>
     * The sensor measures the magnetic field at the sensor position expressed in the sensor frame (S): mSS
     * The magnetic field value is known at the sensor position and is expressed in the origin frame (O): mOS
     * The equation that relates both quantities is:
     * mSS = R_SN R_NO mOS
     *     = R^T( qNS ) R^T( qON ) mOS
     *     = R^T( qON qNS ) mOS
     * 
     * @param trf   {@link DifferentiableTrajectory3dOfReferenceFrame} that describes the orientation and position of the navigation frame.
     * @param time  time at which the measurement is taken.
     * @return  measurement of the magnetometer.
     */
    public Vector3 measure( DifferentiableTrajectory3dOfReferenceFrame trf , double time )
    {
        trf.setTime( time );
        UnitQuaternion qOS = trf.orientation().multiply( this.qNS );
        return qOS.inverseMultiplicativeInplace().rotate( this.mOS );
    }
    
}
