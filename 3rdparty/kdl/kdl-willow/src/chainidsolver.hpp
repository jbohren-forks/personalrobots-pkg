#ifndef KDLCHAINIDSOLVER_HPP
#define KDLCHAINIDSOLVER_HPP

#include "jntarray.hpp"
#include "chain.hpp"

namespace KDL
{

	/**
	 * \brief This <strong>abstract</strong> class encapsulates the inverse
	 * dynamics solver for a KDL::Chain.
	 *
	 */
	class ChainIdSolver
	{
		public:
			/** 
			 * Calculate inverse dynamics, from joint positions, velocity and acceleration
			 * to joint torques. Handles only revolute joints.
			 * 
			 * @param q input joint positions
			 * @param q_dot input joint velocities
			 * @param q_dotdot input joint accelerations
			 *
			 * @param torque output joint torques
			 * 
			 * @return if < 0 something went wrong

			 * <strong>TODO:</strong>
			 * - add parameters for external forces and torques on each link
			 * - add support for prismatic joints.
			 */
			virtual int InverseDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Vector* torque)=0;

			// Need functions to return the manipulator mass, coriolis and gravity matrices - Lagrangian Formulation.
	};

}

#endif

