#ifndef __POSITION_ORIENTATION_BASED_DYNAMICS__
#define __POSITION_ORIENTATION_BASED_DYNAMICS__

#include <vector>
#include "MathDefs.h"
#include "MathUtilities.h"
#include "TimingUtilities.h"
#include "ModelParameters.h"
#include "SceneStepper.h"
#include "PBDCollisionSolver.h"

inline Quaternions times(scalar s, const Quaternions& q) { return Quaternions(s * q.coeffs()); }
inline Quaternions add(const Quaternions& q1, const Quaternions& q2) { return Quaternions(q1.coeffs() + q2.coeffs()); }
inline Quaternions minus(const Quaternions& q1, const Quaternions& q2) { return Quaternions(q1.coeffs() - q2.coeffs()); }
inline Quaternions vec2Qua(const Vector3s& vec) { return Quaternions(0, vec(0), vec(1), vec(2)); }

class Constraint
{
public:
	virtual scalar update() = 0;
};

class BendTwistConstraint : public Constraint
{
public:
	BendTwistConstraint(Quaternions& q, Quaternions& u, scalar wq, scalar wu);
	virtual scalar update();
private:
	Quaternions& m_q;
	Quaternions& m_u;
	Quaternions m_rest_Darboux;
	scalar m_sq, m_su;
};


class StretchShearConstraint :public Constraint
{
public:
	StretchShearConstraint(Vector3s& p1, Vector3s& p2, Quaternions& q, scalar w1, scalar w2, scalar wq);
	virtual scalar update();
private:
	scalar m_l;
	Vector3s& m_p1;
	Vector3s& m_p2;
	Quaternions& m_q;
	scalar m_sp1, m_sp2, m_sq;
	const Vector3s m_e3;
};


class EdgeEdgeCollisionConstraint : public Constraint
{
public:
	EdgeEdgeCollisionConstraint(Vector3s& p0, Vector3s& p1, Vector3s& p2, Vector3s& p3, scalar w0, scalar w1, scalar w2, scalar w3);
	virtual scalar update();
private:
	scalar m_inv_mass0, m_inv_mass1, m_inv_mass2, m_inv_mass3;
	Vector3s& m_p0;
	Vector3s& m_p1;
	Vector3s& m_p2;
	Vector3s& m_p3;
};


class PositionOrientation : public SceneStepper
{
public:
	PositionOrientation(const ModelParameters& model);
	virtual ~PositionOrientation();

	virtual int getVertNum() const { return m_n_vert; }
	virtual int getEdgeNum() const { return m_n_edge; }
	virtual int getStrandNum() const { return m_n_strand; }

	virtual bool stepScene();

	virtual void applyRigidTransform(scalar t);

	virtual std::string getName() const;

	int edgeToVert(int edge) const;
	bool isTip(int edge) const;

protected:

	const ModelParameters& m_model;
	PBDCollisionSolver m_collision_solver;

	int m_n_strand;
	int m_n_vert;
	int m_n_edge;

	Vec3Array m_p;
	Vec3Array m_v;
	VectorXs m_mp;
	QuaArray m_q;
	Vec3Array m_omega;
	VectorXs m_mq;
	// Vector3sArray m_moment_inertia;

	Vec3Array m_old_p;
	QuaArray m_old_q;
	std::vector<int> m_edge_to_vert;
	std::vector<bool> m_is_tip;

	std::vector<int> m_pos_start_index;
	std::vector<int> m_qua_start_index;

	std::vector<Constraint*> m_constraint;
	std::vector<Constraint*> m_bend_twist_constraint;
	std::vector<Constraint*> m_stretch_shear_constraint;
	std::vector<Constraint*> m_dynamic_constraint;

	virtual void setNextX();

	bool detectCollision(int i1, int i2, int i3, int i4) const;

	friend class PBDCollisionSolver;
};

#endif // !__POSITION_ORIENTATION_BASED_DYNAMICS__