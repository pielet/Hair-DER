#include "PositionOrientationDynamics.h"


BendTwistConstraint::BendTwistConstraint(Quaternions& q, Quaternions& u, scalar wq, scalar wu):
	m_q(q),
	m_u(u),
	m_sq(wq / (wq + wu)),
	m_su(- wu / (wq + wu)),
	m_rest_Darboux(q.conjugate() * u)
{
	// regulate direction
	Quaternions init_qu_plus = add(m_rest_Darboux, Quaternions(1, 0, 0, 0));
	Quaternions init_qu_minus = minus(m_rest_Darboux, Quaternions(1, 0, 0, 0));
	if (init_qu_minus.squaredNorm() > init_qu_plus.squaredNorm())
		m_rest_Darboux.coeffs() *= -1.0;
}


scalar BendTwistConstraint::update()
{
	Quaternions Darboux = m_q.conjugate() * m_u;
	Quaternions Darboux_add = add(Darboux, m_rest_Darboux);
	Darboux.coeffs() -= m_rest_Darboux.coeffs();
	if (Darboux_add.squaredNorm() < Darboux.squaredNorm()) Darboux = Darboux_add;
	Darboux.w() = 0;

	m_q.coeffs() += 0.01 * m_sq * (m_u * Darboux).coeffs();
	m_u.coeffs() += 0.01 * m_su * (m_q * Darboux).coeffs();
	m_q.normalize();
	m_u.normalize();

	return Darboux.norm();
}


StretchShearConstraint::StretchShearConstraint(Vector3s& p1, Vector3s& p2, Quaternions& q, scalar w1, scalar w2, scalar wq) :
	m_l((p2 - p1).norm()),
	m_p1(p1),
	m_p2(p2),
	m_q(q),
	m_sp1(w1 * m_l / (w1 + w2 + 4 * wq*m_l*m_l)),
	m_sp2(-w2 * m_l / (w1 + w2 + 4 * wq*m_l*m_l)),
	m_sq(wq * m_l*m_l / (w1 + w2 + 4 * wq*m_l*m_l)),
	m_e3(Vector3s(0, 0, 1))
{}


scalar StretchShearConstraint::update()
{
	Vector3s constraint = (m_p2 - m_p1) / m_l - m_q * m_e3;

	m_p1 += m_sp1 * constraint;
	m_p2 += m_sp2 * constraint;
	Quaternions q_e_3_bar(m_q.z(), -m_q.y(), m_q.x(), -m_q.w());
	m_q.coeffs() += m_sq * (vec2Qua(constraint) * q_e_3_bar).coeffs();
	m_q.normalize();

	return constraint.norm();
}


EdgeEdgeCollisionConstraint::EdgeEdgeCollisionConstraint(
	Vector3s& p0, Vector3s& p1, Vector3s& p2, Vector3s& p3,
	scalar w0, scalar w1, scalar w2, scalar w3):
	m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3),
	m_inv_mass0(w0), m_inv_mass1(w1), m_inv_mass2(w2), m_inv_mass3(w3)
{}


scalar EdgeEdgeCollisionConstraint::update()
{
	scalar s, t;
	if (!lineLineIntersect(m_p0, m_p1, m_p2, m_p3, s, t)) {
		throw "EdgeEdgeCollisionConstraint->lineLineIntersect failed";
	}

	if (s < 0.0) s = 0.0;
	if (s > 1.0) s = 1.0;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;

	scalar b0 = 1 - s;
	scalar b1 = s;
	scalar b2 = 1 - t;
	scalar b3 = t;

	Vector3s q0 = m_p0 * b0 + m_p1 * b1;
	Vector3s q1 = m_p2 * b2 + m_p3 * b3;
	Vector3s n = (m_p1 - m_p0).cross(m_p3 - m_p2).normalized();
	scalar C = n.dot(q0 - q1) + 1e-6;

	if (C > 0) {
		Vector3s grad0 = n * b0;
		Vector3s grad1 = n * b1;
		Vector3s grad2 = -n * b2;
		Vector3s grad3 = -n * b3;

		s = m_inv_mass0 * b0*b0 + m_inv_mass1 * b1*b1 + m_inv_mass2 * b2*b2 + m_inv_mass3 * b3*b3;
		s = C / s;

		m_p0 -= s * m_inv_mass0 * grad0;
		m_p1 -= s * m_inv_mass1 * grad1;
		m_p2 -= s * m_inv_mass2 * grad2;
		m_p3 -= s * m_inv_mass3 * grad3;
	}

	return C;
}


PositionOrientation::PositionOrientation(const ModelParameters& model) :
	m_collision_solver(this),
	m_model(model),
	m_pos_start_index(model.m_startIndex),
	m_n_strand(model.m_startIndex.size() - 1),
	m_n_vert(model.m_isFixed.size()),
	m_n_edge(m_n_vert - m_n_strand)
{
	// resize system
	m_mp.resize(m_n_vert); m_mp.setOnes();	// mass of moving position = 1
	m_mq.resize(m_n_edge); m_mq.setOnes();
	m_mq *= 10;
	m_old_p.resize(m_n_vert);
	m_old_q.resize(m_n_edge);
	m_edge_to_vert.resize(m_n_edge);
	m_is_tip.resize(m_n_edge);
	m_timing_statistics.resize(2);

	// fill in position and velovity
	int n_vert = 0;
	scalar total_length = 0;
	for (int i = 0; i < m_n_strand; ++i) {
		n_vert = m_pos_start_index[i + 1] - m_pos_start_index[i];
		m_qua_start_index.push_back(m_pos_start_index[i] - i);

		Vector3s from = Vector3s(0, 0, 1.0), to;
		for (int j = 0; j < n_vert; ++j) {
			int global_j = j + m_pos_start_index[i];
			m_p.push_back(m_model.m_rest_x.segment<3>(3 * global_j));
			m_v.push_back(m_model.m_init_v.segment<3>(3 * global_j));

			if (j == 0) continue;
			to = m_p[global_j] - m_p[global_j - 1];
			// scalar rest_length = to.norm();
			Quaternions q = Quaternions::FromTwoVectors(from, to);
			
			if (j == 1) m_q.push_back(q);
			else m_q.push_back(q * m_q.back());
			from = to;

			m_omega.push_back(Vector3s::Zero());
		}
	}

	m_qua_start_index.push_back(m_n_edge);

	for (int i = 0; i < m_n_strand; ++i) {
		for (int j = m_qua_start_index[i]; j < m_qua_start_index[i + 1]; ++j) {
			m_edge_to_vert[j] = j + i;
			if (j < m_qua_start_index[i + 1] - 1) m_is_tip[j] = false;
			else m_is_tip[j] = true;
		}
	}

	initNextX(m_n_vert * 3);
	setNextX();

	// fill in weight (ATTENTION: dont set quaternion weight)
	for (int i = 0; i < m_n_vert; ++i) {
		if (m_model.m_isFixed[i]) {
			m_mp[i] = 0;
		}
	}

	// m_mq[0] = 0;
	// m_omega[0] = Vector3s(1.0, 0.0, 0.0);

	// construct constraint (bilateral interleaving order)
	int p_base, q_base, left, right;
	for (int i = 0; i < m_n_strand; ++i) {
		p_base = m_pos_start_index[i];
		q_base = m_qua_start_index[i];
		n_vert = m_pos_start_index[i + 1] - m_pos_start_index[i];

		for (int j = m_pos_start_index[i]; j < m_pos_start_index[i + 1] - 1; ++j)
			m_stretch_shear_constraint.push_back(new StretchShearConstraint(m_p[j], m_p[j + 1], m_q[j - i], m_mp[j], m_mp[j + 1], m_mq[j - i]));

		for (int j = m_qua_start_index[i]; j < m_qua_start_index[i + 1] - 1; ++j)
			m_bend_twist_constraint.push_back(new BendTwistConstraint(m_q[j], m_q[j + 1], m_mq[j], m_mq[j + 1]));
		
		/*
		// stretch & shear constraint
		left = 0;
		right = n_vert - 2;
		if (n_vert % 2 == 0) {
			m_stretch_shear_constraint.push_back(new StretchShearConstraint(m_p[p_base + right], m_p[p_base + right + 1], m_q[q_base + right], m_mp[p_base + right], m_mp[p_base + right + 1], m_mq[q_base + right]));
			--right;
		}
		while (right >= 0) {
			m_stretch_shear_constraint.push_back(new StretchShearConstraint(m_p[p_base + left], m_p[p_base + left + 1], m_q[q_base + left], m_mp[p_base + left], m_mp[p_base + left + 1], m_mq[q_base + left]));
			left += 2;
			m_stretch_shear_constraint.push_back(new StretchShearConstraint(m_p[p_base + right], m_p[p_base + right + 1], m_q[q_base + right], m_mp[p_base + right], m_mp[p_base + right + 1], m_mq[q_base + right]));
			right -= 2;
		}
		
		// bend & twist constraint
		left = 1;
		right = n_vert - 2;
		if (n_vert % 2 == 1) {
			m_bend_twist_constraint.push_back(new BendTwistConstraint(m_q[q_base + right - 1], m_q[q_base + right], m_mq[q_base + right - 1], m_mq[q_base + right]));
			--right;
		}
		while (right >= 1) {
			m_bend_twist_constraint.push_back(new BendTwistConstraint(m_q[q_base + left - 1], m_q[q_base + left], m_mq[q_base + left - 1], m_mq[q_base + left]));
			left += 2;
			m_bend_twist_constraint.push_back(new BendTwistConstraint(m_q[q_base + right - 1], m_q[q_base + right], m_mq[q_base + right - 1], m_mq[q_base + right]));
			right -= 2;
		}
		*/
	}
}


PositionOrientation::~PositionOrientation()
{
	for (auto& cp : m_constraint)
		delete cp;
}


int PositionOrientation::edgeToVert(int edge) const
{
	assert(edge >= 0 && edge < m_n_edge);
	return m_edge_to_vert[edge];
}


bool PositionOrientation::isTip(int edge) const
{
	assert(edge >= 0 && edge < m_n_edge);
	return m_is_tip[edge];
}


void PositionOrientation::applyRigidTransform(scalar t) {
	Affine3s trans = m_model.getTransform(t);

	for (int i = 0; i < getVertNum(); ++i) {
		if (m_model.m_isFixed[i]) {
			m_p[i] = trans * m_model.m_rest_x.segment<3>(3 * i);
		}
	}
}


std::string PositionOrientation::getName() const
{
	return "Position and Orientation Based Dynamics";
}


void PositionOrientation::setNextX()
{
	for (int i = 0; i < getVertNum(); ++i) {
		m_next_x.segment<3>(3 * i) = m_p[i];
	}
}


bool PositionOrientation::detectCollision(int i0, int i1, int i2, int i3) const
{
	// coplanet detection
	Vector3s x1 = m_old_p[i1] - m_old_p[i0];
	Vector3s x2 = m_old_p[i2] - m_old_p[i0];
	Vector3s x3 = m_old_p[i3] - m_old_p[i0];

	Vector3s v1 = m_p[i1] - m_p[i0] - x1;	// normalized velocity -> t in [0, 1]
	Vector3s v2 = m_p[i2] - m_p[i0] - x2;
	Vector3s v3 = m_p[i3] - m_p[i0] - x3;

	scalar a = v1.dot(v2.cross(v3));	// a * t^3 + b * t^2 + c * t + d = 0
	scalar b = v1.dot(v2.cross(x3)) + v1.dot(x2.cross(v3)) + x1.dot(v2.cross(v3));
	scalar c = v1.dot(x2.cross(x3)) + x1.dot(v2.cross(x3)) + x1.dot(x2.cross(v3));
	scalar d = x1.dot(x2.cross(x3));

	scalar t = 2;
	int solve_type;
	if (isSmall(a)) {
		if (isSmall(b)) {
			if (isSmall(c)) {
				if (isSmall(d)) t = 0.0;
				else return false;
			}
			else t = d / c;
		}
		else {
			Vector2s x;
			solve_type = solveSquareEquation(x, c / b, d / b);
			if (solve_type == 0) return false;
			if (solve_type == 2) {
				if (inZeroOne(x[0])) t = x[0];
				if (inZeroOne(x[1]) && x[1] < t) t = x[1];
			}
		}
	}
	else {
		Vector3s x;
		solve_type = solveCubicEquation(x, b / a, c / a, d / a);
		for (int i = 0; i < solve_type; ++i) {
			if (x[i] >= 0 && x[i] <= 1 && x[i] < t)
				t = x[i];
		}
	}
	if (!inZeroOne(t)) return false;

	// distance between two edges
	scalar u, v;
	if (lineLineIntersect(
			(1 - t) * m_old_p[i0] + t * m_p[i0],
			(1 - t) * m_old_p[i1] + t * m_p[i1],
			(1 - t) * m_old_p[i2] + t * m_p[i2],
			(1 - t) * m_old_p[i3] + t * m_p[i3], u, v
	)) {
		if (inZeroOne(u) && inZeroOne(v)) return true;
	}

	return false;
}


bool PositionOrientation::stepScene()
{
	scalar t0;

	/****************** pre-update velocity and position ****************/
	for (int i = 0; i < m_n_vert; ++i) {
		m_v[i] += m_model.m_dt * m_mp[i] * m_model.m_gravity;
		m_old_p[i] = m_p[i];
		m_p[i] += m_model.m_dt * m_v[i];
	}

	for (int j = 0; j < m_n_edge; ++j) {
		m_old_q[j] = m_q[j];
		m_q[j].coeffs() += 0.5 * m_model.m_dt * (vec2Qua(m_omega[j])* m_q[j]).coeffs();
		m_q[j].normalize();
	}
	
	/***************** generate collision constraints ******************/
	t0 = timingutils::seconds();
	std::cout << "[collision detection]" << std::endl;
	
	m_collision_solver.clear();
	for (int j = 0; j < m_n_edge - 2; ++j) {
		int next_test = isTip(j) ? j + 1 : j + 2;
		for (int jj = next_test; jj < m_n_edge; ++jj) {
			unsigned n1 = edgeToVert(j);
			unsigned n2 = edgeToVert(jj);
			if (detectCollision(n1, n1 + 1, n2, n2 + 1))
				m_collision_solver.addPair(j, jj);
		}
	}
	/*
	for (int s = 0; s < m_n_strand; ++s) {
		for (int j = m_qua_start_index[s]; j < m_qua_start_index[s + 1]; ++j) {
			// self-collision
			for (int jj = j + 2; jj < m_qua_start_index[s + 1]; ++jj) {
				if (detectCollision(j + s, j + s + 1, jj + s, jj + s + 1))
					m_collision_solver.addPair(j, jj);
			}
			// inter-hair collision
			for (int ss = s + 1; ss < m_n_strand; ++ss) {
				for (int jj = m_qua_start_index[ss]; jj < m_qua_start_index[ss + 1]; ++jj) {
					if (detectCollision(j + s, j + s + 1, jj + ss, jj + ss + 1)) {
						std::cout << j << '\n';
						m_collision_solver.addPair(j, jj);
					}
				}
			}
		}
	}
	*/
	m_timing_statistics[0] += timingutils::seconds() - t0;
	
	
	/********************** project constraint ***********************/
	t0 = timingutils::seconds();
	std::cout << "[project constraint]" << std::endl;

	scalar constraint = 0;
	scalar bend_twist_constrain = 0;
	scalar stretch_shear_constrain = 0;
	scalar collision_constraint = 0;
	for (int s = 0; s < m_model.m_max_iters; ++s) {
		bend_twist_constrain = stretch_shear_constrain = 0;
		collision_constraint = 0;
		
		if (s % 2 == 0) {
			for (auto cp : m_stretch_shear_constraint)
				stretch_shear_constrain += cp->update();
			for (auto cp : m_bend_twist_constraint)
				bend_twist_constrain += cp->update();
		}
		else {
			for (int j = m_stretch_shear_constraint.size() - 1; j >= 0; --j)
				m_stretch_shear_constraint[j]->update();
			for (int i = m_bend_twist_constraint.size() - 1; i >= 0; --i)
				m_bend_twist_constraint[i]->update();
		}
		
		m_collision_solver.solveCollision();
		
		std::cout << "solver: " << s + 1 << '\n'
			<< "bend_twist: " << bend_twist_constrain / m_bend_twist_constraint.size() << '\n'
			<< "stretch_shear: " << stretch_shear_constrain / m_stretch_shear_constraint.size() << '\n';
	}
	m_timing_statistics[0] += timingutils::seconds() - t0;

	/*********************** update velocity ************************/
	for (int i = 0; i < m_n_vert; ++i)
		m_v[i] = (m_p[i] - m_old_p[i]) / m_model.m_dt;
	for (int j = 0; j < m_n_edge; ++j) 
		m_omega[j] = 2 / m_model.m_dt * (m_q[j] * m_old_q[j].conjugate()).vec();

	m_collision_solver.updateVelocity();

	setNextX();

	return true;
}