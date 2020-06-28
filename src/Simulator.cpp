#include "Simulator.h"

Simulator::Simulator(const std::string& model_file, const std::string& trans_file) :
    m_model( model_file, trans_file ),
    m_camera( m_model.m_lookAt, m_model.m_eye, m_model.m_up ),
    m_gui( &m_model, &m_t )
{   
	switch (m_model.m_stepper_type)
	{
	case StepperType::DER:
		m_stepper = new CompliantImplicitEuler(m_model); break;
	case StepperType::POBD:
		m_stepper = new PositionOrientation(m_model); break;
	default:
		std::cerr << "Wrong Stepper Type!";
		break;
	}

	m_numStrand = m_stepper->getStrandNum();
	m_numParticle = m_stepper->getVertNum();
	m_numDof = 4 * m_numParticle - m_numStrand;

    // create inital data buffer
    m_glBuffer = new GLfloat[m_numParticle * 3];
    for (int i = 0; i < m_numParticle * 3; ++i) {
        m_glBuffer[i] = m_stepper->getNextX()[i];
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

	if (m_stepper->stepScene()) {
		/*
		std::vector<scalar> total_time = m_stepper->getTimingStatistics();
		for (int i = 0; i < total_time.size(); ++i)
			std::cout << total_time[i] / (m_currentStep + 1) << '\t';
		std::cout << '\n';
		*/

		// update vertex array
		VectorXs next_x = m_stepper->getNextX();
        for (int i = 0; i < m_numParticle * 3; ++i) {
			m_glBuffer[i] = next_x(i);
        }

        if (++m_currentStep == m_model.m_step)
			m_isSimulationEnd = true;
		m_t += m_model.m_dt;
        m_shouldSave = true;
		m_stepper->applyRigidTransform( m_t );
		
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
			std::this_thread::sleep_for(std::chrono::microseconds(int(m_model.m_dt * 1e6)));
		}
	}
}

void Simulator::resetParameter() {
    if (m_isPause) {
		// update parameters
		m_model.setStrandParameters();
        
        delete m_stepper;
		switch (m_model.m_stepper_type)
		{
		case StepperType::DER:
			m_stepper = new CompliantImplicitEuler(m_model); break;
		case StepperType::POBD:
			m_stepper = new PositionOrientation(m_model); break;
		default:
			std::cerr << "Wrong Stepper Type!";
			break;
		}

		// update data array
		for (int i = 0; i < m_numParticle * 3; ++i) {
			m_glBuffer[i] = m_stepper->getNextX()[i];
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
