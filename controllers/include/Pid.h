#pragma once


/***************************************************/
/*! \brief A basic pid class.
    
    This class implements a generic structure that
    can be used to create a wide range of pid 
    controllers. It can function independently or 
    be subclassed to provide more specific controls 
    based on a particular control loop.

    In particular, this class implements the standard
    pid equation:

    command  = Pterm + Iterm + Dterm

    where: <br> 
    <UL TYPE="none">
    <LI>  Pterm  = pGain * pError
    <LI>  Iterm  = iGain * iError
    <LI>  Dterm  = dGain * dError
    <LI>  iError = iError + pError * dT
    <LI>  dError = dError + pError / dT
    </UL> 
      
    given:<br> 
    <UL TYPE="none"> 
    <LI>  pError = pTarget - pState.
    </UL>
    
    If the fixedTime input of UpdatePid is set to alpha, 
    dT = alpha. Otherwise the time step is computed when 
    the function is called.

*/
/***************************************************/
class Pid
{
  public:
  
    /*!
      * \brief Constructor, zeros out Pid values when created and 
      * initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
      *
      * \param P  The proportional gain.
      * \param I  The integral gain. 
      * \param D  The derivative gain.
      * \param I1 The integral upper limit.
      * \param I2 The integral lower limit.
      */
    Pid(double P = 0.8,double I = 0.5, double D = 0.0, double I1 = 1.0, double I2 =-1.0);
    
    /*!
      * \brief Destructor of Pid class.
      */       
    ~Pid( );

    /*!
      * \brief Update the Pid loop with nonuniform time step size.
      *
      * \param pState  This is the current measured state or position of the object 
      * being controlled.
      * \param pTarget This is the set point the controller is trying to reach.
      * \param fixedTime Set to a value for fixed time step of that value
      */
    double UpdatePid( double pError, double dt );  
    
    /*!
      * \brief Initialize PID-gains and integral term limits:[iMax:iMin]-[I1:I2]
      *
      * \param P  The proportional gain.
      * \param I  The integral gain. 
      * \param D  The derivative gain.
      * \param I1 The integral upper limit.
      * \param I2 The integral lower limit.
      */
    void   InitPid( double P,double I, double D, double I1, double I2 );  
    

  private:
    double pError;          /**< Derivative state. */
    double dError;          /**< Derivative state. */
    double iError;          /**< Integrator state. */    
    double pGain;           /**< Proportional gain. */
    double iGain;           /**< Integral gain. */
    double dGain;           /**< Derivative gain. */
    double iMax;            /**< Maximum allowable integrator state. */
    double iMin;            /**< Minimum allowable integrator state. */
    double currentCommand;  /**< Current position command. */
    float  currentTime;     /**< Current time in seconds. */
    float  lastTime;        /**< Lasttime in seconds. */
    bool   timeInitiated;   /**< Has currentTime and lastTime been initiated? */
};

