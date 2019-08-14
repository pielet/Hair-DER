#ifndef __GUI_H__
#define __GUI_H__

#include <vector>
#include <GLFW/glfw3.h>
#include <AntTweakBar.h>
#include "ModelParameters.h"

struct GUI
{
    GUI ( ModelParameters* model, double* t ) : m_model(model) {
        m_bar = TwNewBar("Main");
        TwDefine(" Main label='~ Parameters ~' color='100 100 100' alpha=128 fontSize=3 position='10 10' size='270 500' valuesWidth=fit ");

        TwAddVarRO(m_bar, "name", TW_TYPE_STDSTRING, &(model->m_description), NULL);

        TwAddVarRW(m_bar, "t", TW_TYPE_DOUBLE, t, "group=Simulation");
        TwAddVarRW(m_bar, "dt", TW_TYPE_DOUBLE, &(model->m_dt), "group=Simulation");
        TwAddVarRW(m_bar, "duration", TW_TYPE_DOUBLE, &(model->m_duration), "group=Simulation");
        TwDefine(" Main/Simulation");

        TwAddVarRW(m_bar, "gravity x", TW_TYPE_DOUBLE, &(model->m_gx), "group=Gravity");
        TwAddVarRW(m_bar, "gravity y", TW_TYPE_DOUBLE, &(model->m_gy), "group=Gravity");
        TwAddVarRW(m_bar, "gravity z", TW_TYPE_DOUBLE, &(model->m_gz), "group=Gravity");
        TwDefine(" Main/Gravity ");

        TwAddVarRW(m_bar, "max iteration", TW_TYPE_INT32, &(model->m_max_iters), "group=Stepper");
        TwAddVarRW(m_bar, "criterion", TW_TYPE_DOUBLE, &(model->m_criterion), "group=Stepper");
        TwDefine(" Main/Stepper ");

        TwAddVarRW(m_bar, "radius", TW_TYPE_DOUBLE, &(model->m_radius), "group=Strands");
        TwAddVarRW(m_bar, "young modulus", TW_TYPE_DOUBLE, &(model->m_youngModulus), "group=Strands");
        TwAddVarRW(m_bar, "shear modulus", TW_TYPE_DOUBLE, &(model->m_shearModulus), "group=Strands");
        TwAddVarRW(m_bar, "density", TW_TYPE_DOUBLE, &(model->m_density), "group=Strands");
        TwAddVarRW(m_bar, "viscosity", TW_TYPE_DOUBLE, &(model->m_viscosity), "group=Strands");
        TwAddVarRW(m_bar, "base ratation", TW_TYPE_DOUBLE, &(model->m_baseRotation), "group=Strands");
        TwAddVarRW(m_bar, "accumulate with viscous", TW_TYPE_BOOLCPP, &(model->m_accumulateWithViscous), "group=Strands");
        TwAddVarRW(m_bar, "accumulate viscous only for bending modes", TW_TYPE_BOOLCPP, &(model->m_accumulateViscousOnlyForBendingModes), "group=Strands");
        TwDefine(" Main/Strands ");
    }

    TwBar* m_bar;
    ModelParameters* m_model;
};

#endif