#include "Simulator.h"

Simulator::Simulator(const std::string& model_file, const std::string& trans_file) :
    m_model( model_file, trans_file ),
    m_camera( m_model.m_lookAt, m_model.m_eye, m_model.m_up ),
    m_gui( &m_model, &m_t )
{   
    m_scene = new Scene( m_model );
    m_stepper = new CompliantImplicitEuler( *m_scene, m_model.m_max_iters, m_model.m_criterion );

    m_numStrand = m_scene->getNumStrand();
    m_numParticle = m_scene->getNumParticle();
    m_numDof = m_scene->getNumDofs();

    // create inital data buffer
    m_glBuffer = new GLfloat[m_numParticle * 3];
    for (int i = 0; i < m_numParticle * 3; ++i) {
        m_glBuffer[i] = m_model.m_rest_x(i);
    }

    // add button to GUI
    TwAddButton(m_gui.m_bar, "center", centerCameraCB, this, "key=c");
    TwAddButton(m_gui.m_bar, "reset", resetParameterCB, this, "key=r");
	TwAddButton(m_gui.m_bar, "startOrPause", startOrPauseCB, this, "label='start' key=SPACE");
	TwAddButton(m_gui.m_bar, "step", stepCB, this, "key=s");
	TwAddButton(m_gui.m_bar, "quit", quitCB, this, "key=q");

    // start autostep thread
    m_autoStepTid = std::thread(std::bind(&Simulator::autoStep, this));
}

Simulator::~Simulator() {
    m_isQuit = true;

    if (m_oneStepTid.joinable()) m_oneStepTid.join();
    m_autoStepTid.join();

    if (!m_glBuffer) 
        delete[] m_glBuffer;
    
    delete m_scene;
    delete m_stepper;
}

void TW_CALL Simulator::centerCameraCB( void* pThis ) {
    Simulator* sim = (Simulator*) pThis;
    (sim->m_camera).center();
}

void TW_CALL Simulator::resetParameterCB( void* pThis ) {
    Simulator* sim = (Simulator*) pThis;
    sim->resetParameter();
}

void TW_CALL Simulator::startOrPauseCB( void* pThis ) {
    Simulator* sim = (Simulator*) pThis;
    sim->startOrPause();
}

void TW_CALL Simulator::stepCB( void* pThis ) {
    Simulator* sim = (Simulator*) pThis;
    sim->step();
}

void TW_CALL Simulator::quitCB( void* pThis ) {
    Simulator* sim = (Simulator*) pThis;
    sim->quit();
}

bool Simulator::oneStep() {
    int i = 0, j = 0;

	if (m_currentStep == m_model.m_step)
		return true;

	if (m_stepper->stepScene( *m_scene, m_model.m_dt )) {
		m_stepper->accept( *m_scene, m_model.m_dt );

		// update vertex array
        for (int i = 0; i < m_numParticle; ++i) {
            Vector3s cp = m_scene->getX().segment<3>( m_scene->getDof(i) );
            for (j = 0;j < 3; ++j) {
                m_glBuffer[3 * i + j] = cp(j);
            }
        }

        if (++m_currentStep == m_model.m_step)
			m_isSimulationEnd = true;
		m_t += m_model.m_dt;
        m_shouldSave = true;
		m_scene->applyRigidTransform( m_t );
		
		return true;

	} else {
		std::cerr << "Fail to update at step: " << m_currentStep;
        m_isSimulationEnd = true;
		return false;
	}
}

void Simulator::autoStep() {
    while (!m_isQuit )
	{
		if (!m_isSimulationEnd && !m_isPause) {
			oneStep();
			// std::this_thread::sleep_for(std::chrono::microseconds(int(m_model.m_dt * 1e6)));
		}
	}
}

void Simulator::resetParameter() {
    if (m_isPause) {
		// update parameters
		m_model.setStrandParameters();
        
        delete m_scene; delete m_stepper;
        m_scene = new Scene(m_model);
        m_stepper = new CompliantImplicitEuler(*m_scene, m_model.m_max_iters, m_model.m_criterion);

		// update data array
		for (int i = 0; i < m_numParticle * 3; ++i) {
			m_glBuffer[i] = m_model.m_rest_x(i);
		}

		// clear flags
		m_t = 0;
		m_currentStep = 0;
		m_isSimulationEnd = false;

		std::cout << "===================== reset ===================\n";
    }
}

void Simulator::startOrPause() {
    if (m_isPause) {
		TwSetParam(m_gui.m_bar, "startOrPause", "label", TW_PARAM_CSTRING, 1, "pause");
        if (m_oneStepTid.joinable()) m_oneStepTid.join();
		m_isPause = false;
	} else {
		TwSetParam(m_gui.m_bar, "startOrPause", "label", TW_PARAM_CSTRING, 1, "start");
		m_isPause = true;
	}
}

void Simulator::step() {
    if (m_isPause) {
        if (m_oneStepTid.joinable()) m_oneStepTid.join();
        m_oneStepTid = std::thread(std::bind(&Simulator::oneStep, this));
    }
}
