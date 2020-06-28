#include "SceneStepper.h"

SceneStepper::~SceneStepper() {}

int SceneStepper::getDof(int i) const { return 0; }

void SceneStepper::initNextX(size_t size)
{
	m_next_x.resize(size);
}

const VectorXs& SceneStepper::getNextX() const
{
	return m_next_x;
}

const std::vector<scalar>& SceneStepper::getTimingStatistics() const
{
	return m_timing_statistics;
}

void SceneStepper::write(std::vector<scalar>&) const {};

void SceneStepper::read(const scalar* data) {};

size_t SceneStepper::size() { return 0; };
