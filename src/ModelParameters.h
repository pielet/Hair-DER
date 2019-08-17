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
    VectorXs m_rest_x;
    std::vector<int> m_startIndex;
    std::vector<bool> m_isFixed;
    std::vector<Affine3s> m_transform;
    int m_nframe;
    float m_timeInterval;

    ModelParameters( const std::string& xml_file, const std::string trans_file = "") {
        std::ifstream fin( xml_file );

        if ( !fin.is_open() ){
            std::cerr << "Can NOT open xml file\n";
            return;
        }

        // parse xml file
        std::string buffer(std::istreambuf_iterator<char>(fin), {});
        fin.close();
        char* char_xml = new char[buffer.length() + 1];
        buffer.copy(char_xml, buffer.length(), 0);
        char_xml[buffer.length()] = '\0';
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

        // set m_rest_x(position),  m_startIndex and m_isFixed
        int idx = 0;
        double x, y, z;
        // set m_rest_x
        for (auto strand_node = scene->first_node("Strand"); strand_node; strand_node = strand_node->next_sibling()) {
            m_startIndex.push_back( idx );
            for (node = strand_node->first_node(); node; node = node->next_sibling()) {
                std::istringstream oss( node->first_attribute()->value() );
                oss >> x >> y >> z;
                m_rest_x.conservativeResize( m_rest_x.size() + 3 );
                m_rest_x.segment<3>( 3 * idx ) = Vec3(x, y, z);

                if (node->first_attribute("fixed")) 
                    m_isFixed.push_back(true);
                else
                    m_isFixed.push_back(false);
                
                idx++;
            }
        }
        m_startIndex.push_back(idx);

        delete [] char_xml;
        
        if (!trans_file.empty()) {
            fin.open(trans_file, std::ios::binary);
            if (!fin.is_open()) {
                std::cerr << "Can NOT open transform matrix file\n";
                return;
            }

            fin.read((char*)&m_nframe, sizeof(int));
            fin.read((char*)&m_timeInterval, sizeof(float));

            float mat_buffer[16];
            Affine3s mat, inv_trans;

            fin.read((char*)mat_buffer, sizeof(float) * 16);
            inv_trans.matrix() = Eigen::Map<Matrix4f>(mat_buffer).transpose().cast<double>();
            inv_trans = inv_trans.inverse();
            m_transform.push_back(Affine3s::Identity());

            for (int i = 1; i < m_nframe; ++i) {
                fin.read((char*)mat_buffer, sizeof(float) * 16);
                mat.matrix() = Eigen::Map<Matrix4f>(mat_buffer).transpose().cast<double>();
                m_transform.push_back(mat * inv_trans);
            }
        }
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

    Affine3s getTransform( double t ) const {
        if (t > m_timeInterval * (m_nframe - 1))
            return Affine3s::Identity();
        
        int interval_idx = int(t / m_timeInterval);
        float alpha = (t - m_timeInterval * interval_idx) / m_timeInterval;

        return lerp(alpha, m_transform[interval_idx], m_transform[interval_idx + 1]);
    }

    inline void lerpDecompose(const Affine3s &aff, Vector3s &pos, Quaternions &rot, Vector3s &scale) const
    {
		Matrix3s rot_mat, scale_mat;
		aff.computeRotationScaling(&rot_mat, &scale_mat);

		pos = aff.translation();
		rot = Quaternions(rot_mat);
		scale = scale_mat.diagonal();
	}

	inline Affine3s lerpCompose(float alpha,
		const Vector3s &pos0, const Quaternions &rot0, const Vector3s &scale0,
		const Vector3s &pos1, const Quaternions &rot1, const Vector3s &scale1) const
	{
		float one_minus_alpha = 1.0f - alpha;

		Affine3s result;
		result.fromPositionOrientationScale(
			one_minus_alpha * pos0 + alpha * pos1,
			rot0.slerp(alpha, rot1),
			one_minus_alpha * scale0 + alpha * scale1);

		return result;
	}

	/*
	* Lerp between to Affine3s to get the correct interpolation, All affine should not contain the shear
	* components.
	*/
	inline Affine3s lerp(float alpha, const Affine3s &aff0, const Affine3s &aff1) const
    {
		Vector3s pos0; Quaternions rot0; Vector3s scale0;
		Vector3s pos1; Quaternions rot1; Vector3s scale1;
		lerpDecompose(aff0, pos0, rot0, scale0);
		lerpDecompose(aff1, pos1, rot1, scale1);

		if (rot0.dot(rot1) < 0.0f)
			rot1 = Quaternions(-rot1.w(), -rot1.x(), -rot1.y(), -rot1.z());

		return lerpCompose(alpha, pos0, rot0, scale0, pos1, rot1, scale1);
	}
};

#endif