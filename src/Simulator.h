#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <GL/glew.h>
#include <AntTweakBar.h>

#include "ModelParameters.h"
#include "Scene.h"
#include "CompliantImplicitEuler.h"
#include "GUI.h"

class Simulator
{
    ModelParameters m_model;
    Scene* m_scene;
    CompliantImplicitEuler* m_stepper;
    GUI m_gui;

    // simulation parameter
    int m_numStrand;
    int m_numParticle;
    int m_numDof;
    int m_currentStep = 0;
    double m_t = 0;

    GLdouble* m_glBuffer;

    // flag
    bool m_isQuit = false;
    bool m_isPause = true;
    bool m_isSimulationEnd = false;

    // step function
    bool oneStep();

    // callback entry
    void resetParameter();
    void startOrPause();
    void step();

public:
    Simulator( const std::string& model_file, const std::string& trans_file );
    ~Simulator();

    bool isQuit() const { return m_isQuit; }
    void quit() { m_isQuit = true; }

    int getNumParticle() const { return m_numParticle; }
    int getNumStrand() const { return m_numStrand; }
    const std::vector<int>& getStartIndex() const { return m_model.m_startIndex; }
    GLdouble* getBuffer() { return m_glBuffer; }

    // GUI callback function
    static void TW_CALL resetParameterCB( void* pThis );
    static void TW_CALL startOrPauseCB( void* pThis );
    static void TW_CALL stepCB( void* pThis );
    static void TW_CALL quitCB( void* pThis );

    void autoStep();
};

#endif