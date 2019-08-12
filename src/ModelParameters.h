#ifndef __MODEL_PARAMETERS_H__
#define __MODEL_PARAMETERS_H__

#include <fstream>
#include <sstream>
#include "rapidxml.hpp"

#include "MathDefs.h"
#include "StrandParameters.h"

struct ModelParameters
{
    std::string m_description;
    double m_duration;
    Vector3s m_gravity;
    StrandParameters* m_strandParameters;
    VectorXs m_strands;
    std::vector<int> m_startIndex;
    std::vector<bool> m_isFixed;

    ModelParameters( const std::string& file_name, double dt ) :
        m_gravity( Vector3s::Zero() )
    {
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
        double gravity = getDoubleNodeValue( scene->first_node( "simplegravity" ) );
        m_gravity(1) = gravity;

        // set m_strandParatemers
        rapidxml::xml_node<>* node = scene->first_node( "StrandParameters" );
        double radius = getDoubleNodeValue( node->first_node( "radius") );
        double young = getDoubleNodeValue( node->first_node( "youngsModulus" ) );
        double shear = getDoubleNodeValue( node->first_node( "shearModulus" ) );
        double density = getDoubleNodeValue( node->first_node( "density" ) );
        double viscosity = getDoubleNodeValue( node->first_node( "viscosity" ) );
        double base_ratation = getDoubleNodeValue( node->first_node( "baseRotation" ) );
        bool accum_v = compareString( node->first_node( "accumulateWithViscous" ), "1");
        bool accum_v_bend = compareString( node->first_node( "accumulateViscousOnlyForBendingModes" ), "1");

        m_strandParameters = new StrandParameters( radius, young, shear, 1, density, viscosity, base_ratation, dt, accum_v, accum_v_bend );

        // set m_strands (position) m_startIndex and m_isFixed
        int idx = 0;
        double x, y, z;
        // set m_strands
        for (auto strand_node = scene->first_node("Strand"); strand_node; strand_node = strand_node->next_sibling()) {
            m_startIndex.push_back(idx);
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

    double getDoubleNodeValue( const rapidxml::xml_node<>* node ) const {
        return atof(node->first_attribute()->value());
    }

    bool compareString( const rapidxml::xml_node<>* node, const char* other ) const {
        return strcmp( node->first_attribute()->value() , other );
    }

    int getNumStrand() const { return m_startIndex.size() - 1; }
};

#endif