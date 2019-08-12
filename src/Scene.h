#ifndef __SCENE_H__
#define __SCENE_H__

#include <fstream>
#include "rapidxml.hpp"
#include "MathDefs.h"
#include "ThreadUtils.h"
#include "Force.h"
#include "StrandParameters.h"
#include "ModelParameters.h"

class Scene
{
    const ModelParameters& m_model;

    VectorXs m_x;
    VectorXs m_v;
    VectorXs m_m;
    VectorXs m_radii;
    VectorXs m_edge_radii;
    VectorXs m_edge_to_hair;

    std::vector<int> m_startDofsIndex;
    std::vector<int> m_vertToDof;

    Vector6i m_constraint_idx;

    std::vector<Force*> m_forces;
    std::vector<Force*> m_external_forces;
    std::vector<Force*> m_internal_forces;
    std::vector<Force*> m_inter_hair_forces;
    std::vector< std::vector<Force*> > m_hair_internal_forces; // hair idx -> forces only affecting that hair

    void resizeSystem();
    void initForces();
    void categorizeForces();

public:
    Scene(const ModelParameters& model);
    ~Scene();

    StrandParameters* getStrandParameters() { return m_model.m_strandParameters; }

    const VectorXs& getX() const { return m_x; }
    VectorXs& getX() { return m_x; }

    const VectorXs& getV() const { return m_v; }
    VectorXs& getV() { return m_v; }
    
    const VectorXs& getM() const { return m_m; }
    VectorXs& getM() { return m_m; }

    int getNumParticle() const { return m_model.m_isFixed.size(); }
    int getNumStrand() const { return m_model.m_startIndex.size() - 1; }
    int getNumDofs() const { return m_x.size(); }

    bool isFixed( int particle ) const;
    bool isTip( int particle ) const;

    int getParticleIndex( int strandIndex ) const;
    int getDofIndex( int strandIndex ) const;
    int getDof( int vertIndex ) const;
    int getVertFromDof( int DofIndex ) const;
    int getComponent( int DofIndex ) const;

    void computeMassesAndRadiiFromStrands();

    void updateNumConstraints(int& num_constraint_pos_, int& num_constraint_vel_, int& num_J_, int& num_Jv_, int& num_Jxv_, int& num_tildeK_,
                                        Vector6i& interhair_param, Vector6i& interhair_num);
    void postPreprocess(VectorXs& lambda, VectorXs& lambda_v,
                            TripletXs& J, TripletXs& Jv, TripletXs& Jxv, TripletXs& tildeK,
                            TripletXs& stiffness, TripletXs& damping, VectorXs& Phi, VectorXs& Phiv, const VectorXs& dx, const VectorXs& dv, const scalar& dt);
    
    void preCompute( const VectorXs& dx, const VectorXs& dv, const scalar& dt );
    void postCompute( scalar dt );

    void accumulateExternalGradU( VectorXs& F, const VectorXs& dx, const VectorXs& dv );

    void storeLambda(const VectorXs& lambda, const VectorXs& lambda_v);
};

#endif