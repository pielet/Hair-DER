#include "Scene.h"
#include "SimpleGravityForce.h"
#include "StrandForce.h"

Scene::Scene( const ModelParameters& model ):
    m_model(model)
{
    int idx = 0;
    int i, j, numPartical;
    for (i = 0; i < getNumStrand(); ++i) {
        m_startDofsIndex.push_back( idx );
        numPartical = model.m_startIndex[i + 1] - model.m_startIndex[i];
        m_x.conservativeResize( m_x.size() + 4 * numPartical - 1);
        for (j = 0; j < numPartical; ++j) {
            int global_j = model.m_startIndex[i] + j;
            m_x.segment<3>(idx + 4 * j) = model.m_strands.segment<3>(3 * global_j);
            if (j < numPartical - 1)
                m_x[idx + 4*j + 3] = 0.0;
            m_vertToDof.push_back( idx + 4 * j);
        }
        idx += 4 * numPartical - 1;
    }
    m_startDofsIndex.push_back( idx );

    resizeSystem();
    initForces();
    computeMassesAndRadiiFromStrands();
}

Scene::~Scene() {
    for (auto i = m_forces.begin(); i != m_forces.end(); ++i) {
        delete *i;
    }
}

void Scene::initForces() {
    // add gravity and strand force
    m_forces.push_back( new SimpleGravityForce( m_model.m_gravity, this ) );

    int numParticle = 0;
    for (int i = 0; i < getNumStrand(); ++i) {
        numParticle = m_model.m_startIndex[i + 1] - m_model.m_startIndex[i];
        m_forces.push_back( new StrandForce( this, i, numParticle ) );
    }

    categorizeForces();
}

void Scene::resizeSystem() {
    int nDof = getNumDofs();
    int nVert = getNumParticle();
    int nEdge = getNumParticle() - getNumStrand();

    m_v.resize( nDof ); m_v.setZero();
    m_radii.resize( nVert ); m_radii.setZero();
    m_m.resize( nDof ); m_m.setZero();
    m_edge_radii.resize( nEdge ); m_edge_radii.setZero();
    m_edge_to_hair.resize( nEdge ); m_edge_to_hair.setZero();
}

void Scene::categorizeForces()
{
    m_external_forces.clear();
    m_internal_forces.clear();
    m_inter_hair_forces.clear();
    m_hair_internal_forces.resize( getNumStrand() );
    
    for( size_t i = 0; i < getNumStrand(); ++i)
    {
        m_hair_internal_forces[i].resize(0);
    }
    
    for( size_t i = 0; i < m_forces.size(); ++i )
    {
        if (m_forces[i]->isExternal()) {
            m_external_forces.push_back( m_forces[i] );
        } 
        else {
            m_internal_forces.push_back( m_forces[i] );
            
            if (!m_forces[i]->isInterHair()) {
                int hidx = m_forces[i]->getAffectedHair();
                if (hidx < 0) continue;
                m_hair_internal_forces[hidx].push_back( m_forces[i] );
            } else {
                m_inter_hair_forces.push_back( m_forces[i] );
            }
        }
    }
}

bool Scene::isFixed( int particle ) const {
    assert( particle >= 0 && particle < getNumParticle() );
    return m_model.m_isFixed[particle];
}

bool Scene::isTip( int particle ) const {
    assert( particle >= 0 && particle < getNumParticle() );

    for (int i = 1; i < m_model.m_startIndex.size(); ++i) {
        if (m_model.m_startIndex[i] == particle + 1)
            return true;
    }
    return false;
}

int Scene::getParticleIndex( int strandIndex ) const {
    assert( strandIndex >= 0 && strandIndex < getNumStrand() );
    return m_model.m_startIndex[strandIndex];
}

int Scene::getDofIndex( int strandIndex ) const {
    assert( strandIndex >= 0 && strandIndex < getNumStrand() );
    return m_startDofsIndex[strandIndex];
}

int Scene::getDof( int vertIndex ) const {
    assert( vertIndex >= 0 && vertIndex < getNumParticle() );
    return m_vertToDof[vertIndex];
}

int Scene::getVertFromDof( int DofIndex ) const {
    assert( DofIndex >= 0 && DofIndex < m_x.size());

    int i;
    for (i = 0;i < getNumParticle(); ++i) {
        if (DofIndex >= m_vertToDof[i] && DofIndex - m_vertToDof[i] < 4) return i;
    }
    return i;
}

int Scene::getComponent( int DofIndex ) const {
    assert( DofIndex >= 0 && DofIndex < m_x.size());
    return DofIndex - getDof( getVertFromDof( DofIndex ) );
}

void Scene::computeMassesAndRadiiFromStrands()
{
    int nStrand = 0;
    for (int f = 0; f < m_forces.size(); ++f) {
        StrandForce* const strand = dynamic_cast<StrandForce*>( m_forces[f] );
        if ( strand == NULL ) continue;

        if ( strand->m_strandParams->m_straightHairs != 1.0 ){
            Vec2Array& kappas = strand->alterRestKappas();
            for( int k = 0; k < kappas.size(); ++k ){
                kappas[k] *= strand->m_strandParams->m_straightHairs;
            }
        }

        for( int v = 0; v < strand->getNumVertices(); ++v ) {
            const int globalVtx = getParticleIndex( strand->m_globalIndex ) + v;
            const int globalEdx = globalVtx - nStrand;
            const int globalDof = getDof( globalVtx );
            const scalar r = strand->m_strandParams->getRadius( v, strand->getNumVertices() );

            m_radii[ globalVtx ] = r;
            m_m.segment<3>( globalDof ).setConstant( strand->m_vertexMasses[v] );

            if ( v < strand->getNumEdges() ){
                m_edge_to_hair[globalEdx] = nStrand;
                // Edge radius, edge's should be indexed the same as 
                m_edge_radii[ globalEdx ] = r;

                // Twist Mass (Second moment of inertia * length)
                const scalar mass = strand->m_strandParams->m_density * M_PI * r * r * strand->m_restLengths[v];
                scalar vtm = 0.25 * mass * 2 * r * r;
                m_m[ globalDof + 3 ] = vtm;
            }
        }
        ++nStrand;
    }

    assert( nStrand == getNumStrand() );
}

void Scene::updateNumConstraints(int& num_constraint_pos_, int& num_constraint_vel_, int& num_J_, int& num_Jv_, int& num_Jxv_, int& num_tildeK_,
                                Vector6i& interhair_param, Vector6i& interhair_num)
{
    m_constraint_idx = Vector6i::Zero();
  
    int ns = getNumStrand();
    for(int i = 0; i < ns; ++i)
    {
        const std::vector< Force* >& hair_forces = m_hair_internal_forces[i];
        int nhair_forces = (int) hair_forces.size();
        
        Vector6i constraint_start = m_constraint_idx;
        
        for(int j = 0; j < nhair_forces; ++j)
        {
            int num_pos = hair_forces[j]->numConstraintPos();
            int num_vel = hair_forces[j]->numConstraintVel();
            int num_J = hair_forces[j]->numJ();
            int num_Jv = hair_forces[j]->numJv();
            int num_Jxv = hair_forces[j]->numJxv();
            int num_TildeK = hair_forces[j]->numTildeK();
            
            hair_forces[j]->setInternalIndex(m_constraint_idx(0), m_constraint_idx(1), m_constraint_idx(2), m_constraint_idx(3), m_constraint_idx(4), m_constraint_idx(5));
            m_constraint_idx(0) += num_pos;
            m_constraint_idx(1) += num_vel;
            m_constraint_idx(2) += num_J;
            m_constraint_idx(3) += num_Jv;
            m_constraint_idx(4) += num_Jxv;
            m_constraint_idx(5) += num_TildeK;
        }
    
        Vector6i num_constraints = m_constraint_idx - constraint_start;
        
        // m_flows[i]->setConstraintParameters(constraint_start, num_constraints);
    }
  
    interhair_param = m_constraint_idx;
    
    for( std::vector<Force*>::size_type i = 0; i < m_inter_hair_forces.size(); ++i )
    {
        int num_pos = m_inter_hair_forces[i]->numConstraintPos();
        int num_vel = m_inter_hair_forces[i]->numConstraintVel();
        int num_J = m_inter_hair_forces[i]->numJ();
        int num_Jv = m_inter_hair_forces[i]->numJv();
        int num_Jxv = m_inter_hair_forces[i]->numJxv();
        int num_TildeK = m_inter_hair_forces[i]->numTildeK();
        
        m_inter_hair_forces[i]->setInternalIndex(m_constraint_idx(0), m_constraint_idx(1), m_constraint_idx(2), m_constraint_idx(3), m_constraint_idx(4), m_constraint_idx(5));
        m_constraint_idx(0) += num_pos;
        m_constraint_idx(1) += num_vel;
        m_constraint_idx(2) += num_J;
        m_constraint_idx(3) += num_Jv;
        m_constraint_idx(4) += num_Jxv;
        m_constraint_idx(5) += num_TildeK;
    }
    
    num_constraint_pos_ = m_constraint_idx(0);
    num_constraint_vel_ = m_constraint_idx(1);
    num_J_ = m_constraint_idx(2);
    num_Jv_ = m_constraint_idx(3);
    num_Jxv_ = m_constraint_idx(4);
    num_tildeK_ = m_constraint_idx(5);
    
    interhair_num = m_constraint_idx - interhair_param;
}

void Scene::postPreprocess(VectorXs& lambda, VectorXs& lambda_v,
                                    TripletXs& J, TripletXs& Jv, TripletXs& Jxv, TripletXs& tildeK,
                                    TripletXs& stiffness, TripletXs& damping, VectorXs& Phi, VectorXs& Phiv, const VectorXs& dx, const VectorXs& dv, const scalar& dt)
{
    const int nf = (int) m_internal_forces.size();
    if (dx.size() == 0) {
        // for unparallelized forces
        threadutils::thread_pool::ParallelFor(0, nf, [&] (int i) {
            if(!m_internal_forces[i]->isParallelized()) m_internal_forces[i]->computeIntegrationVars( m_x, m_v, m_m, lambda, lambda_v, J, Jv, Jxv, tildeK, stiffness, damping, Phi, Phiv, dt);
        });
    
        // for parallelized forces
        for(int i = 0; i < nf; ++i)
        {
            if(m_internal_forces[i]->isParallelized()) m_internal_forces[i]->computeIntegrationVars( m_x, m_v, m_m, lambda, lambda_v, J, Jv, Jxv, tildeK, stiffness, damping, Phi, Phiv, dt);
        }
    } 
    else {
        VectorXs nx = m_x + dx;
        VectorXs nv = m_v + dv;
        // for unparallelized forces
        threadutils::thread_pool::ParallelFor(0, nf, [&] (int i) {
            if(!m_internal_forces[i]->isParallelized()) m_internal_forces[i]->computeIntegrationVars( nx, nv, m_m, lambda, lambda_v, J, Jv, Jxv, tildeK, stiffness, damping, Phi, Phiv, dt);
        });
        
        // for parallelized forces
        for(int i = 0; i < nf; ++i)
        {
            if(m_internal_forces[i]->isParallelized()) m_internal_forces[i]->computeIntegrationVars( nx, nv, m_m, lambda, lambda_v, J, Jv, Jxv, tildeK, stiffness, damping, Phi, Phiv, dt);
        }
    }
}

void Scene::preCompute( const VectorXs& dx, const VectorXs& dv, const scalar& dt )
{
    assert( dx.size() == dv.size() );

    int nf = m_forces.size();
    if( dx.size() == 0 ) {
        threadutils::thread_pool::ParallelFor(0, nf, [&] (int i) {
        if(!m_forces[i]->isPrecomputationParallelized()) m_forces[i]->preCompute( m_x, m_v, m_m, dt );
        });
        
        for(int i = 0; i < nf; ++i)
        {
        if(m_forces[i]->isPrecomputationParallelized()) m_forces[i]->preCompute( m_x, m_v, m_m, dt );
        }
    }
    else  {
        VectorXs nx = m_x + dx;
        VectorXs nv = m_v + dv;
        threadutils::thread_pool::ParallelFor(0, nf, [&] (int i) {
        if(!m_forces[i]->isPrecomputationParallelized()) m_forces[i]->preCompute( nx, nv, m_m, dt );
        });
        
        for(int i = 0; i < nf; ++i)
        {
        if(m_forces[i]->isPrecomputationParallelized()) m_forces[i]->preCompute( nx, nv, m_m, dt );
        }
    }
}

void Scene::postCompute( scalar dt ) {
    const int nforces = (int) m_forces.size();
    for (int i = 0; i < nforces; ++i) m_forces[i]->postStepScene(dt);
}

void Scene::accumulateExternalGradU( VectorXs& F, const VectorXs& dx, const VectorXs& dv )
{
    assert( F.size() == m_x.size() );
    assert( dx.size() == dv.size() );
    assert( dx.size() == 0 || dx.size() == F.size() );
    
    // Accumulate all energy gradients
    if( dx.size() == 0 ) {
        for( std::vector<Force*>::size_type i = 0; i < m_external_forces.size(); ++i ) {
            m_external_forces[i]->addGradEToTotal( m_x, m_v, m_m, F );
        }
    }
    else for( std::vector<Force*>::size_type i = 0; i < m_external_forces.size(); ++i ) {
        m_external_forces[i]->addGradEToTotal( m_x+dx, m_v+dv, m_m, F );
    }
}

void Scene::storeLambda(const VectorXs& lambda, const VectorXs& lambda_v)
{
    int nf = m_internal_forces.size();
    threadutils::thread_pool::ParallelFor(0, nf, [&] (int i) {
        m_internal_forces[i]->storeLambda(lambda, lambda_v);
    });
}
