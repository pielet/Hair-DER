#include "PBDCollisionSolver.h"
#include "PositionOrientationDynamics.h"


PBDCollisionSolver::PBDCollisionSolver(PositionOrientation* POBD)
	:m_dynamics(POBD)
{
	m_segment_collision_num.resize(m_dynamics->getEdgeNum());
	m_segment_collision_num.setZero();
};


PBDCollisionSolver::~PBDCollisionSolver()
{};


bool PBDCollisionSolver::addPair(unsigned seg_idx1, unsigned seg_idx2, CollisionPair* target)
{
	int e1 = seg_idx1, e2 = seg_idx2;

	if (seg_idx1 == seg_idx2) return false;

	if (seg_idx1 < seg_idx2) {
		if (!m_dynamics->isTip(seg_idx1) && seg_idx1 + 1 == seg_idx2) 
			return false;
	}
	else {
		if (!m_dynamics->isTip(seg_idx2) && seg_idx2 + 1 == seg_idx1)
			return false;
		else {
			e1 = seg_idx2;
			e2 = seg_idx1;
		}
	}

	int n1 = m_dynamics->edgeToVert(e1);
	int n2 = m_dynamics->edgeToVert(e2);

	Vector3s p0 = m_dynamics->m_old_p[n1];
	Vector3s p1 = m_dynamics->m_old_p[n1 + 1];
	Vector3s p2 = m_dynamics->m_old_p[n2];
	Vector3s p3 = m_dynamics->m_old_p[n2 + 1];

	double C = (p1 - p0).cross(p3 - p2).normalized().dot(p0 - p2);

	if (target)
		target->insert({ segment_pair(e1, e2), CollisionInfo(C) });
	else {
		auto ret = m_collision_pair.insert({ segment_pair(e1, e2), CollisionInfo(C) });
		if (ret.second) {
			++m_segment_collision_num[e1];
			++m_segment_collision_num[e2];
		}
	}

	return true;
}


// =================================================
// return true: valid constraint
// return false: need to update contact edge(s)
// =================================================
bool PBDCollisionSolver::solvePairCollision(collision_pair& cpair)
{
	int n1 = m_dynamics->edgeToVert(cpair.first.first);
	int n2 = m_dynamics->edgeToVert(cpair.first.second);
	scalar init_C = cpair.second.init_C;

	Vector3s& p0 = m_dynamics->m_p[n1];
	Vector3s& p1 = m_dynamics->m_p[n1 + 1];
	Vector3s& p2 = m_dynamics->m_p[n2];
	Vector3s& p3 = m_dynamics->m_p[n2 + 1];

	scalar m0 = m_dynamics->m_mp[n1];
	scalar m1 = m_dynamics->m_mp[n1 + 1];
	scalar m2 = m_dynamics->m_mp[n2];
	scalar m3 = m_dynamics->m_mp[n2 + 1];

	scalar s, t;
	if (!lineLineIntersect(p0, p1, p2, p3, s, t)) {
		throw "EdgeEdgeCollisionConstraint->lineLineIntersect failed";
	}

	cpair.second.setParam(s, t);

	if (s < 0.0 || s > 1.0 || t < 0.0 || t > 1.0) return false;

	scalar b0 = 1 - s;
	scalar b1 = s;
	scalar b2 = 1 - t;
	scalar b3 = t;

	Vector3s q0 = p0 * b0 + p1 * b1;
	Vector3s q1 = p2 * b2 + p3 * b3;
	Vector3s n = (p1 - p0).cross(p3 - p2).normalized();
	scalar C = n.dot(q0 - q1);

	if (C * init_C < 0) {
		Vector3s grad0 = n * b0;
		Vector3s grad1 = n * b1;
		Vector3s grad2 = -n * b2;
		Vector3s grad3 = -n * b3;

		scalar k = m0 * b0*b0 + m1 * b1*b1 + m2 * b2*b2 + m3 * b3*b3;
		k = C / k;

		p0 -= k * m0 * grad0;
		p1 -= k * m1 * grad1;
		p2 -= k * m2 * grad2;
		p3 -= k * m3 * grad3;
	}

	return true;
}


void PBDCollisionSolver::solveCollision()
{
	for (auto iter = m_collision_pair.begin(); iter != m_collision_pair.end(); ) {
		if (!solvePairCollision(*iter)) {
			unsigned e1 = iter->first.first;
			unsigned e2 = iter->first.second;

			scalar a = iter->second.alpha;
			scalar b = iter->second.beta;

			bool add_pair = true;

			if (a < 0.0) {
				if (e1 == 0 || m_dynamics->isTip(e1 - 1)) add_pair = false;
				else --e1;
			}
			else if (a > 1.0) {
				if (m_dynamics->isTip(e1)) add_pair = false;
				else ++e1;
			}
			else if (b < 0.0) {
				if (e2 == 0 || m_dynamics->isTip(e2 - 1)) add_pair = false;
				else --e2;
			}
			else {
				if (m_dynamics->isTip(e2)) add_pair = false;
				else ++e2;
			}

			if (add_pair) addPair(e1, e2, &m_new_pair);

			--m_segment_collision_num[iter->first.first];
			--m_segment_collision_num[iter->first.second];
			iter = m_collision_pair.erase(iter);
		}
		else ++iter;
	}

	// merge and update new pairs
	for (auto pair : m_new_pair) {
		auto ret = m_collision_pair.insert(pair);
		if (ret.second) {
			++m_segment_collision_num[pair.first.first];
			++m_segment_collision_num[pair.first.second];
			solvePairCollision(*ret.first);
		}
	}

	m_new_pair.clear();
}


void PBDCollisionSolver::clear()
{
	m_collision_pair.clear();
	m_new_pair.clear();
	m_segment_collision_num.setZero();
}


void PBDCollisionSolver::updateVelocity()
{
	for (auto pair : m_collision_pair) {
		unsigned n1 = m_dynamics->edgeToVert(pair.first.first);
		unsigned n2 = m_dynamics->edgeToVert(pair.first.second);

		scalar a = pair.second.alpha;
		scalar b = pair.second.beta;

		Vector3s n = (m_dynamics->m_p[n1 + 1] - m_dynamics->m_p[n1]).cross(m_dynamics->m_p[n2 + 1] - m_dynamics->m_p[n2]);
		n.normalize();

		Vector3s& v0 = m_dynamics->m_v[n1];
		Vector3s& v1 = m_dynamics->m_v[n1 + 1];
		Vector3s& v2 = m_dynamics->m_v[n2];
		Vector3s& v3 = m_dynamics->m_v[n2 + 1];

		Vector3s inter_v1 = (1 - a) * v0 + a * v1;
		Vector3s inter_v2 = (1 - b) * v2 + b * v3;

		// normal: completely inelastic collision
		Vector3s delta_v_n = (n.dot(inter_v1 - inter_v2) / 2) * n;

		scalar ks = 2 / (a*a + (1 - a)*(1 - a) + b * b + (1 - b)*(1 - b));
		v0 -= (1 - a) * ks * delta_v_n;
		v1 -= a * ks * delta_v_n;
		v2 += (1 - b) * ks * delta_v_n;
		v3 += b * ks * delta_v_n;

		// tangential: friction (Coulomb's model)
		scalar delta_v_t = m_dynamics->m_model.m_friction * delta_v_n.norm();
		Vector3s v0_t = v0 - n.dot(v0) * n;
		Vector3s v1_t = v1 - n.dot(v1) * n;
		Vector3s v2_t = v2 - n.dot(v2) * n;
		Vector3s v3_t = v3 - n.dot(v3) * n;

		if (delta_v_t > v0_t.norm()) v0 -= v0_t;
		else v0 -= delta_v_t * v0_t.normalized();
		if (delta_v_t > v1_t.norm()) v1 -= v1_t;
		else v1 -= delta_v_t * v1_t.normalized();
		if (delta_v_t > v2_t.norm()) v2 -= v2_t;
		else v2 -= delta_v_t * v2_t.normalized();
		if (delta_v_t > v3_t.norm()) v3 -= v3_t;
		else v3 -= delta_v_t * v3_t.normalized();
	}
}