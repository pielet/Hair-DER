#ifndef __COMPLIANT_IMPLICIT_EULER__
#define __COMPLIANT_IMPLICIT_EULER__

#include <iostream>
#include <fstream>

#include "MathUtilities.h"
#include "ThreadUtils.h"
#include "Force.h"
#include "ModelParameters.h"
#include "StrandParameters.h"
#include "SceneStepper.h"


class CompliantImplicitEuler : public SceneStepper
{
public:
	CompliantImplicitEuler( const ModelParameters& model, bool autoUpdateNextInit = true );
	virtual ~CompliantImplicitEuler();
	
	virtual bool stepScene();

	virtual std::string getName() const;

	// StrandForce require
	StrandParameters* getStrandParameters() { return m_model.m_strandParameters; }
	const VectorXs& getX() const { return m_x; }

	virtual int getVertNum() const { return m_vert_num; }
	virtual int getStrandNum() const { return m_strand_num; }
	virtual int getDof(int i) const { return m_vert2dof[i]; }

	int getDofFromStrand(int i) const { return m_strand2dof[i]; }
	int getVertFromDof(int i) const { return m_dof2vert[i]; }
	int getStartIndex(int i) const { return m_model.m_startIndex[i]; }
	
	bool isTip(int i) const;
	bool isFixed(int i) const { return m_model.m_isFixed[i]; }

	virtual void applyRigidTransform(scalar t);

private:
	// Total number of ...
	int m_strand_num;
	int m_vert_num;
	int m_dof_num;
	scalar m_dt;

	// Hair parameters
	VectorXs m_x;
	VectorXs m_v;
	VectorXs m_a;
	VectorXs m_m;
	VectorXs m_radii;
	VectorXs m_edge_radii;
	VectorXs m_edge_to_hair;
	const ModelParameters& m_model;
	
	// Index convertion array
	std::vector<int> m_strand2dof;
	std::vector<int> m_vert2dof;
	std::vector<int> m_dof2vert;

	// Forces
	std::vector<Force*> m_strand_force;
	std::vector<Force*> m_external_force;

	// Tool function
	void computeMassesAndRadiiFromStrands();
	void zeroFixedDoFs(VectorXs& vec);
	void updateNumConstraints(const VectorXs& dx, const VectorXs& dv);
	void computeIntegrationVars(const VectorXs& dx, const VectorXs& dv);
	void accumulateExternalGradU(VectorXs& F, const VectorXs& dx, const VectorXs& dv);
	void storeLambda(const VectorXs& lambda);

	virtual void setNextX();

	bool m_bAutoUpdateNextInit;
	
	// update var
	TripletXs m_A_nz;
	TripletXs m_J_nz;
	TripletXs m_M_nz;
	TripletXs m_invC_nz;
	
	SparseXs m_A;
	SparseXs m_J;
	SparseXs m_JC;
	SparseXs m_M;
	SparseXs m_invC;
	
	VectorXs m_lambda;
	VectorXs m_gradU;
	VectorXs m_Phi;
	VectorXs m_b;
	VectorXs m_vplus;

	Eigen::SimplicialLDLT< SparseXs > m_solver;
	Eigen::ConjugateGradient< SparseXs > m_iterative_solver;
};

#endif
