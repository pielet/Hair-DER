//
// This file is part of the libWetHair open source project
//
// The code is licensed solely for academic and non-commercial use under the
// terms of the Clear BSD License. The terms of the Clear BSD License are
// provided below. Other licenses may be obtained by contacting the faculty
// of the Columbia Computer Graphics Group or a Columbia University licensing officer.
//
// The Clear BSD License
//
// Copyright 2017 Yun (Raymond) Fei, Henrique Teles Maia, Christopher Batty,
// Changxi Zheng, and Eitan Grinspun
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted (subject to the limitations in the disclaimer
// below) provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its contributors may be used
//  to endorse or promote products derived from this software without specific
//  prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef __SIMPLE_GRAVITY_FORCE_H__
#define __SIMPLE_GRAVITY_FORCE_H__

#include <Eigen/Core>
#include "Force.h"
#include <iostream>
#include "Scene.h"

class SimpleGravityForce : public Force
{
	Scene* m_scene;
public:

	SimpleGravityForce( const Vector3s& gravity, Scene* scene );

	virtual ~SimpleGravityForce();
	
	virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
	
	virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
	
	virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE );
	
	virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE );
	
	// virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE, int pidx );
	
	virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx );
	
	virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx );
	
	virtual void preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt );
	
	virtual int numConstraintPos();
	
	virtual int numConstraintVel();
	
	virtual int numJ();
	
	virtual int numJv();
	
	virtual int numJxv();
	
	virtual int numTildeK();
	
	virtual bool isParallelized();
	
	virtual bool isPrecomputationParallelized();
	
	virtual Force* createNewCopy();

	virtual const char* name();
	
	static const char* static_name();

	virtual void getAffectedVars( int pidx, std::unordered_set<int>& vars );
	
	virtual bool isContained( int pidx );
	
	virtual bool isExternal();
	
	const Vector3s& m_gravity;
};

#endif
