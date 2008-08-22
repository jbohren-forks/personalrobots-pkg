/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* author: Matei Ciocarlie */

#include "object.h"

namespace grasp_module {

object_msg Object::getMsg() const
{
	object_msg om;
       
	om.centroid.x = mCentroid.x; om.centroid.y = mCentroid.y; om.centroid.z = mCentroid.z;

	om.axis1.x = mA1.x; om.axis1.y = mA1.y; om.axis1.z = mA1.z;
	om.axis2.x = mA2.x; om.axis2.y = mA2.y; om.axis2.z = mA2.z;
	om.axis3.x = mA3.x; om.axis3.y = mA3.y; om.axis3.z = mA3.z;

	return om;
}

void Object::setFromMsg(const object_msg &om)
{
	mCentroid.x = om.centroid.x; mCentroid.y = om.centroid.y; mCentroid.z = om.centroid.z;

	mA1.x = om.axis1.x; mA1.y = om.axis1.y; mA1.z = om.axis1.z;
	mA2.x = om.axis2.x; mA2.y = om.axis2.y; mA2.z = om.axis2.z;
	mA3.x = om.axis3.x; mA3.y = om.axis3.y; mA3.z = om.axis3.z;
}

} //namespace grasp_module
