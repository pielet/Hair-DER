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

#include "SimpleGravityForce.h"
#include <tbb/tbb.h>

const static char* simplegravityname = "simplegravity";

SimpleGravityForce::SimpleGravityForce( const Vector3s& gravity, SceneStepper* stepper )
: Force()
, m_gravity(gravity)
, m_stepper(stepper)
{
	assert( (m_gravity.array() == m_gravity.array()).all() );
	assert( (m_gravity.array() != std::numeric_limits<scalar>::infinity()).all() );
}

SimpleGravityForce::~SimpleGravityForce()
{}



void SimpleGravityForce::preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt )
{
    // do nothing if no liquid
}

int SimpleGravityForce::numConstraintPos()
{
	return 0;
}

int SimpleGravityForce::numConstraintVel()
{
	return 0;
}

void SimpleGravityForce::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	
	VectorXs g = m_gravity;
	
	// Assume 0 potential is at origin
	for( int i = 0; i < m_stepper->getVertNum(); ++i ){
		E -= m( m_stepper->getDof(i) ) * g.segment<3>(0).dot( x.segment<3>( m_stepper->getDof(i) ) );
	} 
}

void SimpleGravityForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	assert( x.size() == gradE.size() );
	
	const int np = m_stepper->getVertNum();
	
	VectorXs g = m_gravity;
	
	tbb::parallel_for(0, np, 1, [&] (int i) {
		gradE.segment<3>( m_stepper->getDof(i) ) -= m( m_stepper->getDof(i) ) * g.segment<3>(0);
	});
}

void SimpleGravityForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	// Nothing to do.
}

void SimpleGravityForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	// Nothing to do.
}

/*
void SimpleGravityForce::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE, int pidx )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	assert( x.size() == gradE.size() );

	int idir = m_scene->getComponent( pidx );
	if( idir == DIM ) return;

	scalar rho = m_scene->getHairDensity(pidx);

	gradE(pidx) = m(pidx) * (m_buoyancy(pidx) / rho - m_gravity(idir));
}
*/

void SimpleGravityForce::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	// Nothing to do.
}

void SimpleGravityForce::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx )
{
	assert( x.size() == v.size() );
	assert( x.size() == m.size() );
	// Nothing to do.
}

int SimpleGravityForce::numJ()
{
	return 0;
}

int SimpleGravityForce::numJv()
{
	return 0;
}

int SimpleGravityForce::numJxv()
{
	return 0;
}

int SimpleGravityForce::numTildeK()
{
	return 0;
}

bool SimpleGravityForce::isParallelized()
{
	return true;
}

bool SimpleGravityForce::isPrecomputationParallelized()
{
	return true;
}

const char* SimpleGravityForce::name()
{
	return simplegravityname;
}

const char* SimpleGravityForce::static_name()
{
	return simplegravityname;
}

Force* SimpleGravityForce::createNewCopy()
{
	return new SimpleGravityForce(*this);
}


bool SimpleGravityForce::isExternal()
{
	return true;
}
