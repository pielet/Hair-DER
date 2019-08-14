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

#include "CompliantImplicitEuler.h"
#include "Scene.h"
#include "TimingUtilities.h"
// #include <Eigen/PardisoSupport>

// TODO: Save space by creating dx, dv, rhs, A only once.

CompliantImplicitEuler::CompliantImplicitEuler(Scene& scene, int max_iters, scalar criterion, bool autoUpdateNextInit) : 
	SceneStepper(), 
    m_scene( scene ), 
    m_max_iters( max_iters ), 
    m_criterion( criterion ), 
    m_bAutoUpdateNextInit( autoUpdateNextInit )
{
	SceneStepper::m_timing_statistics.resize(4, 0);
	SceneStepper::init( m_scene );
	
	const int predicted_nnzs_row = 3 * 4;
	const int ndof = scene.getNumDofs();
	
	m_A_nz.reserve(ndof * predicted_nnzs_row);
	m_Kext_nz.reserve(ndof * predicted_nnzs_row);
	m_A.resize(ndof, ndof);
	m_Kext.resize(ndof, ndof);
	m_M_nz.resize(ndof);
	m_M.resize(ndof, ndof);
	m_A.reserve(ndof * predicted_nnzs_row);
	m_Kext.reserve(ndof * predicted_nnzs_row);
	
	m_gradU.resize(ndof);
	m_vplus = scene.getV();
}

CompliantImplicitEuler::~CompliantImplicitEuler()
{}

void CompliantImplicitEuler::updateNumConstraints(const VectorXs& dx, const VectorXs& dv, scalar dt)
{
	int num_pos, num_vel, num_J, num_Jv, num_Jxv, num_tildeK;
	m_scene.updateNumConstraints(num_pos, num_vel, num_J, num_Jv, num_Jxv, num_tildeK, m_interhair_idx, m_interhair_num);
	
	m_lambda.resize(num_pos);
	m_lambda_v.resize(num_vel);
	m_A_nz.resize(num_tildeK);
	m_J_nz.resize(num_J);
	m_Jv_nz.resize(num_Jv);
	m_Jxv_nz.resize(num_Jxv);
	m_invC_nz.resize(num_pos);
	m_invCv_nz.resize(num_vel);
	m_Phi.resize(num_pos);
	m_Phi_v.resize(num_vel);
	
	m_scene.postPreprocess(m_lambda, m_lambda_v, m_J_nz, m_Jv_nz, m_Jxv_nz, m_A_nz, m_invC_nz, m_invCv_nz, m_Phi, m_Phi_v, dx, dv, dt);
}

bool CompliantImplicitEuler::stepScene( Scene& scene, scalar dt, bool updatePreCompute )
{
	std::cout << "[pre-compute]" << std::endl;
	scalar t0 = timingutils::seconds();
	scalar t1;
	
	VectorXs& x = scene.getX();
	VectorXs& v = scene.getV();
	const VectorXs& m = scene.getM();
	assert(x.size() == v.size());
	assert(x.size() == m.size());
	int ndof = x.size();
	int nprts = scene.getNumParticle();

	VectorXs dv(v.size());
	VectorXs dx(x.size());
	VectorXs dx_scripted(x.size());

	dv.setZero();
	dx = v * dt;
	dx_scripted.setZero();  // fixed point will update foreahead using current v
	
	int np = m_scene.getNumParticle();
	for(int i = 0; i < np; ++i)
	{
		if(scene.isFixed(i)) {
			int numdofs = scene.isTip(i) ? 3 : 4;
			dx_scripted.segment( scene.getDof(i), numdofs ) = v.segment( scene.getDof(i), numdofs ) * dt;
		}
	}
	
	if(updatePreCompute) scene.preCompute( dx_scripted, dv, dt );
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[0] += t1 - t0; // local precomputation
	t0 = t1;
	
	std::cout << "[compute-assist-vars]" << std::endl;
	updateNumConstraints(dx_scripted, dv, dt);
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[1] += t1 - t0; // Jacobian
	t0 = t1;
	
	const int nconstraint = m_lambda.size();
	const int nconstraint_v = m_lambda_v.size();
	
	m_A_nz.erase( std::remove_if( m_A_nz.begin(), m_A_nz.end(), [&] ( const Triplets& t ) {
		return scene.isFixed( scene.getVertFromDof( t.row() )) || scene.isFixed( scene.getVertFromDof( t.col()) ) || (t.value() == 0.0);
	}), m_A_nz.end());
	m_A.setFromTriplets( m_A_nz.begin(), m_A_nz.end() );
	
	if (m_J.rows() != nconstraint) m_J.resize( nconstraint, ndof );
	m_J_nz.erase(std::remove_if(m_J_nz.begin(), m_J_nz.end(), [&] ( const Triplets& t ) {
		return scene.isFixed( scene.getVertFromDof( t.col() )) || ( t.value() == 0.0 );
	}), m_J_nz.end());
	m_J.setFromTriplets( m_J_nz.begin(), m_J_nz.end() );
	m_J.makeCompressed();
	
	if (nconstraint_v > 0) {
		if( m_Jv.rows() != nconstraint_v ) m_Jv.resize( nconstraint_v, ndof );
		m_Jv_nz.erase(std::remove_if(m_Jv_nz.begin(), m_Jv_nz.end(), [&] (const Triplets& t) {
			return scene.isFixed(  scene.getVertFromDof(t.col() ) ) || (t.value() == 0.0);
		}), m_Jv_nz.end());
		m_Jv.setFromTriplets(m_Jv_nz.begin(), m_Jv_nz.end());
		m_Jv.makeCompressed();
		
		if (m_Jxv.rows() != nconstraint_v) m_Jxv.resize( nconstraint_v, ndof );
		m_Jxv_nz.erase(std::remove_if(m_Jxv_nz.begin(), m_Jxv_nz.end(), [&] (const Triplets& t) {
			return scene.isFixed(  scene.getVertFromDof(t.col() ) ) || (t.value() == 0.0);
		}), m_Jxv_nz.end());
		m_Jxv.setFromTriplets(m_Jxv_nz.begin(), m_Jxv_nz.end());
		m_Jxv.makeCompressed();
	}
	
	if (m_invC.rows() != nconstraint) m_invC.resize(nconstraint, nconstraint);
	m_invC.setFromTriplets(m_invC_nz.begin(), m_invC_nz.end());
	
	if (m_invCv.rows() != nconstraint_v) m_invCv.resize(nconstraint_v, nconstraint_v);
	m_invCv.setFromTriplets(m_invCv_nz.begin(), m_invCv_nz.end());
	
	m_JC = m_J.transpose() * m_invC;
	
	if (nconstraint_v > 0) m_JvC = m_Jv.transpose() * m_invCv;
	
	if (nconstraint_v > 0)
		m_A += (m_JC * m_J) + (m_JvC * (m_Jv / dt + m_Jxv)) ;
	else
		m_A += m_JC * m_J;
	
	m_A *= dt * dt;

	for(int i = 0; i < ndof; ++i)
	{
		if (scene.isFixed( scene.getVertFromDof(i) )) {
			m_M_nz[i] = (Triplets(i, i, 1.0));
		} else {
			m_M_nz[i] = (Triplets(i, i, m(i)));
		}
	}
	
	m_M.setFromTriplets(m_M_nz.begin(), m_M_nz.end());
	
	m_A += m_M;
	
	m_gradU.setZero();
	scene.accumulateExternalGradU(m_gradU, dx_scripted, dv);
	zeroFixedDoFs(scene, m_gradU);

	if (nconstraint_v > 0)
		m_b = v.cwiseProduct(m) - dt * (m_gradU + (m_JC * m_Phi + m_JvC * (m_Phi_v - m_Jv * v)));
	else
		m_b = v.cwiseProduct(m) - dt * (m_gradU + (m_JC * m_Phi));

	m_A.makeCompressed();
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[2] += t1 - t0; t0 = t1; // matrix composition

	std::cout << "[solve-equations]" << std::endl;
	if(m_max_iters > 0) {
		scalar nmb = m_b.norm();
		
		m_iterative_solver.compute(m_A);
		m_iterative_solver.setTolerance(m_criterion / nmb);
		m_iterative_solver.setMaxIterations(m_max_iters);
		m_vplus = m_iterative_solver.solveWithGuess(m_b, v);
		std::cout << "[cg total iter: " << m_iterative_solver.iterations() << ", res: " << (m_iterative_solver.error() * nmb) << "]" << std::endl;
	} else {
		m_solver.compute(m_A);
		m_vplus = m_solver.solve(m_b);
	}
	
	m_lambda = -m_invC * (m_J * m_vplus * dt + m_Phi);
	if (nconstraint_v > 0)
		m_lambda_v = -m_invCv * (m_Jxv * m_vplus * dt + m_Jv * (m_vplus - v) + m_Phi_v);
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[3] += t1 - t0; t0 = t1; // solve equation
	
	for(int i = 0; i < nprts; ++i){
		if(!scene.isFixed(i)){
			int numdofs = scene.isTip(i) ? 3 : 4;
			v.segment(scene.getDof(i), numdofs) = m_vplus.segment(scene.getDof(i), numdofs);
		}
	}
	
	m_scene.storeLambda( m_lambda, m_lambda_v );
	SceneStepper::m_next_x = x + dt * v;

	return true; 
}

std::string CompliantImplicitEuler::getName() const
{
	return "Linear Compliant Implicit Euler";
}

void CompliantImplicitEuler::zeroFixedDoFs( const Scene& scene, VectorXs& vec )
{
	int nprts = scene.getNumParticle();
	for( int i = 0; i < nprts; ++i ){
		if( scene.isFixed(i) ){
			int numdofs = scene.isTip(i) ? 3 : 4;
			vec.segment( scene.getDof(i), numdofs ).setZero();
		}
	}
}
