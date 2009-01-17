#ifndef KDLCHAINIDSOLVER_NEWTONEULER_HPP
#define KDLCHAINIDSOLVER_NEWTONEULER_HPP

#include "chainidsolver.hpp"

namespace KDL {

	/**
	 * Implementation of inverse dynamics using iterative newton-euler
	 * formulation. 
	 *
	 */
	class ChainIdSolver_NE : public ChainIdSolver
	{
		public:
			/**
			 * Constructor of the solver. The chain provides both kinematic and dynamic parameters.
			 */
			ChainIdSolver_NE(const Chain &chain);
			~ChainIdSolver_NE();

			virtual int InverseDynamics(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, Vector* torque);

		private:
			const Chain chain;
			/**
			 * Array of constraint forces for each of the joints.
			 */
			Vector* forces;
	};

}

#endif

