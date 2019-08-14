#ifndef __MODEL_PARAMETERS_H__
#define __MODEL_PARAMETERS_H__

#include <fstream>
#include <sstream>
#include "rapidxml.hpp"

#include "MathDefs.h"
#include "StrandParameters.h"

struct ModelParameters
{
    // changable parameters
    double m_duration;
    double m_dt;
    double m_gx;
    double m_gy;
    double m_gz;
    Vector3s m_gravity;

    int m_max_iters;
    double m_criterion;

    double m_radius;
    double m_youngModulus;
    double m_shearModulus;
    double m_density;
    double m_viscosity;
    double m_baseRotation;
    bool m_accumulateWithViscous;
    bool m_accumulateViscousOnlyForBendingModes;
    StrandParameters* m_strandParameters = NULL;

    // unchanable parameters
    std::string m_description;
    VectorXs m_strands;
    std::vector<int> m_startIndex;
    std::vector<bool> m_isFixed;

    ModelParameters( const std::string& file_name ) {
        std::ifstream fin( file_name );

        if ( !fin.is_open() ){
            std::cerr << "Can NOT open xml file\n";
            return;
        }

        // parse xml file
        fin.seekg(0, std::ios::end);
        int length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char* char_xml = new char[length + 1];
        fin.read(char_xml, length);
        char_xml[length] = '\0';
        rapidxml::xml_document<> doc;
        doc.parse<0>( char_xml );
        rapidxml::xml_node<>* scene = doc.first_node();

        // set model parameters
        auto des = scene->first_node( "description" );
        m_description = scene->first_node( "description" )->first_attribute()->value();
        m_duration = getDoubleNodeValue( scene->first_node( "duration" ) );
        m_dt = getDoubleNodeValue( scene->first_node( "dt" ));
        m_gravity(0) = m_gx = getDoubleNodeValue( scene->first_node( "simplegravity" ), "fx" );
        m_gravity(1) = m_gy = getDoubleNodeValue( scene->first_node( "simplegravity" ), "fy" );
        m_gravity(2) = m_gz = getDoubleNodeValue( scene->first_node( "simplegravity" ), "fz" );

        // set stepper paratemers
        rapidxml::xml_node<>* node = scene->first_node( "StepperParameters" );
        m_max_iters = (int) getDoubleNodeValue( node->first_node( "max_iters" ));
        m_criterion = getDoubleNodeValue( node->first_node( "criterion" ) );

        // set strand paratemers
        node = scene->first_node( "StrandParameters" );
        m_radius = getDoubleNodeValue( node->first_node( "radius") );
        m_youngModulus = getDoubleNodeValue( node->first_node( "youngsModulus" ) );
        m_shearModulus = getDoubleNodeValue( node->first_node( "shearModulus" ) );
        m_density = getDoubleNodeValue( node->first_node( "density" ) );
        m_viscosity = getDoubleNodeValue( node->first_node( "viscosity" ) );
        m_baseRotation = getDoubleNodeValue( node->first_node( "baseRotation" ) );
        m_accumulateWithViscous = compareString( node->first_node( "accumulateWithViscous" ), "1");
        m_accumulateViscousOnlyForBendingModes = compareString( node->first_node( "accumulateViscousOnlyForBendingModes" ), "1");

        setStrandParameters();

        // set m_strands(position),  m_startIndex and m_isFixed
        int idx = 0;
        double x, y, z;
        // set m_strands
        for (auto strand_node = scene->first_node("Strand"); strand_node; strand_node = strand_node->next_sibling()) {
            m_startIndex.push_back( idx );
            for (node = strand_node->first_node(); node; node = node->next_sibling()) {
                std::istringstream oss( node->first_attribute()->value() );
                oss >> x >> y >> z;
                m_strands.conservativeResize( m_strands.size() + 3 );
                m_strands.segment<3>( 3 * idx ) = Vec3(x, y, z);

                if (node->first_attribute("fixed")) 
                    m_isFixed.push_back(true);
                else
                    m_isFixed.push_back(false);
                
                idx++;
            }
        }
        m_startIndex.push_back(idx);

        delete [] char_xml;
        fin.close();
    }

    ~ModelParameters() { if (m_strandParameters) delete m_strandParameters; }

    double getDoubleNodeValue( const rapidxml::xml_node<>* node, const char* attrName = 0 ) const {
        if (attrName) 
            return atof( node->first_attribute( attrName )->value() );
        else
            return atof( node->first_attribute()->value() );
    }

    bool compareString( const rapidxml::xml_node<>* node, const char* other ) const {
        return strcmp( node->first_attribute()->value() , other );
    }

    int getNumStrand() const { return m_startIndex.size() - 1; }

    void setStrandParameters () {
        if (m_strandParameters != NULL) delete m_strandParameters;

        m_strandParameters = new StrandParameters( 
            m_radius, 
            m_youngModulus, 
            m_shearModulus, 
            1, 
            m_density, 
            m_viscosity, 
            m_baseRotation, 
            m_dt, 
            m_accumulateWithViscous, 
            m_accumulateViscousOnlyForBendingModes 
        );

        m_gravity = Vector3s( m_gx, m_gy, m_gz );
    }
};

#endif