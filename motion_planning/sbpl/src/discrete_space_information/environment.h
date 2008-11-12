/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __ENVIRONMENT_H_
#define __ENVIRONMENT_H_


class DiscreteSpaceInformation
{

public:

	//data
	vector<int*> StateID2IndexMapping;
	
#if DEBUG
	FILE* fDeb;
#endif

	virtual bool InitializeEnv(const char* sEnvFile) = 0;


	virtual bool InitializeMDPCfg(MDPConfig *MDPCfg) = 0;
	virtual int  GetFromToHeuristic(int FromStateID, int ToStateID) = 0;
	virtual int  GetGoalHeuristic(int stateID) = 0;
	virtual int  GetStartHeuristic(int stateID) = 0;
	virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) = 0;
	virtual void SetAllPreds(CMDPSTATE* state) = 0;
	virtual void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) = 0;
	virtual void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV) = 0;

	virtual int	 SizeofCreatedEnv() = 0;
	virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL) = 0;
	virtual void PrintEnv_Config(FILE* fOut) = 0;

    //destructor
    virtual ~DiscreteSpaceInformation(){};

	//constructor
	DiscreteSpaceInformation()
	{

#if DEBUG
		if((fDeb = fopen("envdebug.txt", "w")) == NULL)
		{
			printf("ERROR: failed to open debug file for environment\n");
			exit(1);
		}
#endif

	}
};



#endif

