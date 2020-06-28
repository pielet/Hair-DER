#include "CompliantImplicitEuler.h"
#include "TimingUtilities.h"
#include "SimpleGravityForce.h"
#include "StrandForce.h"

// #include <Eigen/PardisoSupport>

// TODO: Save space by creating dx, dv, rhs, A only once.

CompliantImplicitEuler::CompliantImplicitEuler(const ModelParameters& model, bool autoUpdateNextInit) : 
	SceneStepper(),
	m_model(model),
	m_strand_num(model.m_startIndex.size() - 1),
	m_vert_num(model.m_isFixed.size()),
	m_dt(model.m_dt),
    m_bAutoUpdateNextInit( autoUpdateNextInit )
{
	// fill m_x and index convert array
	int idx = 0;
	int i, j, k, n_vert;
	for (i = 0; i < m_strand_num; ++i) {
		m_strand2dof.push_back(idx);
		n_vert = model.m_startIndex[i + 1] - model.m_startIndex[i];
		m_x.conservativeResize(m_x.size() + 4 * n_vert - 1);
		m_v.conservativeResize(m_x.size());
		for (j = 0; j < n_vert; ++j) {
			int global_vert = model.m_startIndex[i] + j;
			m_x.segment<3>(idx + 4 * j) = model.m_rest_x.segment<3>(3 * global_vert);
			m_v.segment<3>(idx + 4 * j) = model.m_init_v.segment<3>(3 * global_vert);
			for (k = 0; k < 3; ++k)
				m_dof2vert.push_back(global_vert);
			if (j < n_vert - 1) {
				m_x[idx + 4 * j + 3] = 0.0;
				m_v[idx + 4 * j + 3] = 0.0;
				m_dof2vert.push_back(global_vert);
			}
			m_vert2dof.push_back(idx + 4 * j);
		}
		idx += 4 * n_vert - 1;
	}
	m_strand2dof.push_back(idx);

	initNextX(3 * m_vert_num);
	setNextX();

	// add forces
	m_external_force.push_back(new SimpleGravityForce(model.m_gravity, this));
	n_vert = 0;
	for (int i = 0; i < m_strand_num; ++i) {
		n_vert = m_model.m_startIndex[i + 1] - m_model.m_startIndex[i];
		m_strand_force.push_back(new StrandForce(this, i, n_vert));
	}

	// resize & compute hair parameters
	m_dof_num = m_dof2vert.size();
	int n_edge = m_vert_num - m_strand_num;

	m_radii.resize(m_vert_num); m_radii.setZero();
	m_m.resize(m_dof_num); m_m.setZero();
	m_edge_radii.resize(n_edge); m_edge_radii.setZero();
	m_edge_to_hair.resize(n_edge); m_edge_to_hair.setZero();

	computeMassesAndRadiiFromStrands();

	// reserve Jacobian and Hessian
	SceneStepper::m_timing_statistics.resize(4, 0);
	
	const int predicted_nnzs_row = 3 * 4;
	
	m_A_nz.reserve(m_dof_num * predicted_nnzs_row);
	m_A.resize(m_dof_num, m_dof_num);
	m_M_nz.resize(m_dof_num);
	m_M.resize(m_dof_num, m_dof_num);
	m_A.reserve(m_dof_num * predicted_nnzs_row);
	
	m_gradU.resize(m_dof_num);
	m_vplus = m_v;
}

CompliantImplicitEuler::~CompliantImplicitEuler()
{
	for (auto& fi : m_external_force) delete fi;
	for (auto& fi : m_strand_force) delete fi;
}

bool CompliantImplicitEuler::isTip(int particle) const {
	for (int i = 1; i < m_model.m_startIndex.size(); ++i) {
		if (m_model.m_startIndex[i] == particle + 1)
			return true;
	}
	return false;
}

void CompliantImplicitEuler::applyRigidTransform(scalar t) {
	Affine3s trans = m_model.getTransform(t);

	for (int i = 0; i < m_vert_num; ++i) {
		if (isFixed(i)) {
			m_x.segment<3>(getDof(i)) = trans * m_model.m_rest_x.segment<3>(3 * i);
		}
	}
}

void CompliantImplicitEuler::computeMassesAndRadiiFromStrands()
{
	for (int si = 0; si < m_strand_num; ++si) {
		StrandForce* const strand = dynamic_cast<StrandForce*>(m_strand_force[si]);
		if (strand == NULL) continue;

		if (strand->m_strandParams->m_straightHairs != 1.0) {
			Vec2Array& kappas = strand->alterRestKappas();
			for (int k = 0; k < kappas.size(); ++k) {
				kappas[k] *= strand->m_strandParams->m_straightHairs;
			}
		}

		for (int v = 0; v < strand->getNumVertices(); ++v) {
			const int global_vtx = getStartIndex(strand->m_globalIndex) + v;
			const int global_edx = global_vtx - si;
			const int global_dof = getDof(global_vtx);
			const scalar r = strand->m_strandParams->getRadius(v, strand->getNumVertices());

			m_radii[global_vtx] = r;
			m_m.segment<3>(global_dof).setConstant(strand->m_vertexMasses[v]);

			if (v < strand->getNumEdges()) {
				m_edge_to_hair[global_edx] = si;
				// Edge radius, edge's should be indexed the same as 
				m_edge_radii[global_edx] = r;

				// Twist Mass (Second moment of inertia * length)
				const scalar mass = strand->m_strandParams->m_density * M_PI * r * r * strand->m_restLengths[v];
				scalar vtm = 0.25 * mass * 2 * r * r;
				m_m[global_dof + 3] = vtm;
			}
		}
	}
}

void CompliantImplicitEuler::updateNumConstraints(const VectorXs& dx, const VectorXs& dv)
{
	int num_pos = 0,	// num of force: bend 2*(n-2), twist (n-2), strech (n) double if consider viscous
		num_J = 0,		// num of Jacobian: bend 2*(n-2)*11 + twist (n-2)*11 + strech n*6
		num_tildeK = 0;	// number of nonzeros in hessian

	for (int i = 0; i < m_strand_num; ++i)
	{
		m_strand_force[i]->setInternalIndex(num_pos, 0, num_J, 0, 0, num_tildeK);

		num_pos += m_strand_force[i]->numConstraintPos();
		num_J += m_strand_force[i]->numJ();
		num_tildeK += m_strand_force[i]->numTildeK();
	}

	m_lambda.resize(num_pos);
	m_A_nz.resize(num_tildeK);
	m_J_nz.resize(num_J);
	m_invC_nz.resize(num_pos);		// stiffness
	m_Phi.resize(num_pos);
}

void CompliantImplicitEuler::computeIntegrationVars(const VectorXs& dx, const VectorXs& dv)
{
	VectorXs nx = m_x + dx;
	VectorXs nv = m_v + dv;
	// for unparallelized forces
	threadutils::thread_pool::ParallelFor(0, m_strand_num, [&](int i) {
		m_strand_force[i]->computeIntegrationVars(nx, nv, m_m, m_lambda, m_J_nz, m_A_nz, m_invC_nz, m_Phi, m_dt);
	});
}

void CompliantImplicitEuler::accumulateExternalGradU(VectorXs& F, const VectorXs& dx, const VectorXs& dv)
{
	assert(F.size() == m_x.size());
	assert(dx.size() == dv.size());
	assert(dx.size() == 0 || dx.size() == F.size());

	// Accumulate all energy gradients
	for (std::vector<Force*>::size_type i = 0; i < m_external_force.size(); ++i) {
		m_external_force[i]->addGradEToTotal(m_x + dx, m_v + dv, m_m, F);
	}
}

void CompliantImplicitEuler::zeroFixedDoFs(VectorXs& vec)
{
	for (int i = 0; i < m_vert_num; ++i) {
		if (isFixed(i)) {
			int numdofs = isTip(i) ? 3 : 4;
			vec.segment(getDof(i), numdofs).setZero();
		}
	}
}

void CompliantImplicitEuler::storeLambda(const VectorXs& lambda)
{
	threadutils::thread_pool::ParallelFor(0, m_strand_num, [&](int i) {
		m_strand_force[i]->storeLambda(lambda);
	});
}

bool CompliantImplicitEuler::stepScene()
{
	std::cout << "[pre-compute]" << std::endl;
	scalar t0 = timingutils::seconds();
	scalar t1, ts;
	
	assert(m_x.size() == m_v.size());
	assert(m_x.size() == m_m.size());

	VectorXs dv(m_v.size());
	VectorXs dx(m_x.size());
	VectorXs dx_scripted(m_x.size());

	dv.setZero();
	dx = m_v * m_dt;
	dx_scripted.setZero();  // fixed point will update foreahead using current v
	
	for(int i = 0; i < m_vert_num; ++i)
	{
		if(isFixed(i)) {
			int numdofs = isTip(i) ? 3 : 4;
			dx_scripted.segment( getDof(i), numdofs ) = m_v.segment( getDof(i), numdofs ) * m_dt;
		}
	}
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[0] += t1 - t0; // local precomputation
	t0 = t1;
	
	std::cout << "[compute-assist-vars]" << std::endl;
	updateNumConstraints(dx_scripted, dv);
	computeIntegrationVars(dx_scripted, dv);
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[1] += t1 - t0; // Jacobian
	ts = t0 = t1;
	
	const int nconstraint = m_lambda.size();
	
	m_A_nz.erase( std::remove_if( m_A_nz.begin(), m_A_nz.end(), [&] ( const Triplets& t ) {
		return t.value() == 0.0;
	}), m_A_nz.end());

	ts = timingutils::seconds();

	m_A.setFromTriplets( m_A_nz.begin(), m_A_nz.end());

	ts = timingutils::seconds();
	
	if (m_J.rows() != nconstraint) m_J.resize( nconstraint, m_dof_num );
	m_J_nz.erase(std::remove_if(m_J_nz.begin(), m_J_nz.end(), [&] ( const Triplets& t ) {
		return  (t.value() == 0.0);// || scene.isFixed(scene.getVertFromDof(t.col()));
	}), m_J_nz.end());
	m_J.setFromTriplets( m_J_nz.begin(), m_J_nz.end() );
	m_J.makeCompressed();
	
	if (m_invC.rows() != nconstraint) m_invC.resize(nconstraint, nconstraint);
	m_invC.setFromTriplets(m_invC_nz.begin(), m_invC_nz.end());
	
	m_JC = m_J.transpose() * m_invC;
	
	m_A += m_JC * m_J;
	
	m_A *= m_dt * m_dt;

	for(int i = 0; i < m_dof_num; ++i)
	{
		if (isFixed(getVertFromDof(i))) {
			m_M_nz[i] = (Triplets(i, i, 1.0));
		} else {
			m_M_nz[i] = (Triplets(i, i, m_m(i)));
		}
	}
	
	m_M.setFromTriplets(m_M_nz.begin(), m_M_nz.end());
	
	m_A += m_M;
	
	m_gradU.setZero();
	accumulateExternalGradU(m_gradU, dx_scripted, dv);
	zeroFixedDoFs(m_gradU);

	m_b = m_v.cwiseProduct(m_m) - m_dt * (m_gradU + (m_JC * m_Phi));

	m_A.makeCompressed();
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[2] += t1 - t0; t0 = t1; // matrix composition

	std::cout << "[solve-equations]" << std::endl;
	if(m_model.m_max_iters > 0) {
		scalar nmb = m_b.norm();
		
		m_iterative_solver.compute(m_A);
		m_iterative_solver.setTolerance(m_model.m_criterion / nmb);
		m_iterative_solver.setMaxIterations(m_model.m_max_iters);
		m_vplus = m_iterative_solver.solveWithGuess(m_b, m_v);
		std::cout << "[cg total iter: " << m_iterative_solver.iterations() << ", res: " << (m_iterative_solver.error() * nmb) << "]" << std::endl;
	} else {
		m_solver.compute(m_A);
		m_vplus = m_solver.solve(m_b);
	}
	
	m_lambda = -m_invC * (m_J * m_vplus * m_dt + m_Phi);
	
	t1 = timingutils::seconds();
	SceneStepper::m_timing_statistics[3] += t1 - t0; t0 = t1; // solve equation
	
	// update x, v
	for(int i = 0; i < m_vert_num; ++i){
		if(!isFixed(i)){
			int numdofs = isTip(i) ? 3 : 4;
			m_v.segment(getDof(i), numdofs) = m_vplus.segment(getDof(i), numdofs);
		}
	}
	storeLambda( m_lambda );
	m_x += m_v * m_dt;

	setNextX();

	return true; 
}


void CompliantImplicitEuler::setNextX()
{
	for (int i = 0; i < m_vert_num; ++i) {
		SceneStepper::m_next_x.segment<3>(3 * i) = m_x.segment<3>(getDof(i));
	}
}

std::string CompliantImplicitEuler::getName() const
{
	return "Linear Compliant Implicit Euler";
}

