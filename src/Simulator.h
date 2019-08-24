#ifndef __SIMULATOR_H__
#define __SIMULATOR_H__

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <GL/glew.h>
#include <AntTweakBar.h>

#include "ModelParameters.h"
#include "Camera.h"
#include "Scene.h"
#include "CompliantImplicitEuler.h"
#include "GUI.h"

class Simulator
{
    ModelParameters m_model;
    Camera m_camera;
    Scene* m_scene;
    CompliantImplicitEuler* m_stepper;
    GUI m_gui;

    // simulation parameter
    int m_numStrand;
    int m_numParticle;
    int m_numDof;
    int m_currentStep = 0;
    double m_t = 0;

    GLfloat* m_glBuffer;

    // flag
    bool m_isQuit = false;
    bool m_isPause = true;
    bool m_isSimulationEnd = false;
    bool m_shouldSave = false;

    // thread
    std::thread m_autoStepTid;
    std::thread m_oneStepTid;

    // step function
    bool oneStep();

    // GUI callback function
    static void TW_CALL centerCameraCB( void* pThis );
    static void TW_CALL resetParameterCB( void* pThis );
    static void TW_CALL startOrPauseCB( void* pThis );
    static void TW_CALL stepCB( void* pThis );
    static void TW_CALL quitCB( void* pThis );

    // callback entry
    void resetParameter();
    void startOrPause();
    void step();
    void quit() { m_isQuit = true; };

    void autoStep();

public:
    Simulator( const std::string& model_file, const std::string& trans_file );
    Simulator( const Simulator& other ) = delete;
    ~Simulator();

    bool isQuit() const { return m_isQuit; }
    int getNumParticle() const { return m_numParticle; }
    int getNumStrand() const { return m_numStrand; }
    int getStep() const { return m_currentStep; }
    const std::vector<int>& getStartIndex() const { return m_model.m_startIndex; }

    bool shouldSave() const { return m_shouldSave; }
    bool& shouldSave() { return m_shouldSave; }

    GLfloat* getBuffer() { return m_glBuffer; }
    Camera* getCamera() { return &m_camera; }
};

#endif