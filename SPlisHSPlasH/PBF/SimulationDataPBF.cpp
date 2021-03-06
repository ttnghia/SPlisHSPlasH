#include "SimulationDataPBF.h"
#include "SPlisHSPlasH/SPHKernels.h"

using namespace SPH;

SimulationDataPBF::SimulationDataPBF()
{
	m_model = NULL;
}

SimulationDataPBF::~SimulationDataPBF(void)
{
	cleanup();
}

void SimulationDataPBF::init(FluidModel *model)
{
	m_model = model;
	m_lambda.resize(model->numParticles());
	m_deltaX.resize(model->numParticles());
	m_oldX.resize(model->numParticles());
	m_lastX.resize(model->numParticles());
	reset();
}

void SimulationDataPBF::cleanup()
{
	m_lambda.clear();
	m_deltaX.clear();
	m_oldX.clear();
	m_lastX.clear();
}

void SimulationDataPBF::reset()
{
	for(unsigned int i=0; i < m_deltaX.size(); i++)
	{
		m_deltaX[i].setZero();
		m_lambda[i] = 0.0;
		getLastPosition(i) = m_model->getPosition(0, i);
		getOldPosition(i) = m_model->getPosition(0, i);
	}
}

void SimulationDataPBF::performNeighborhoodSearchSort()
{
	const unsigned int numPart = m_model->numParticles();
	if (numPart == 0)
		return;

	auto const& d = m_model->getNeighborhoodSearch()->point_set(0);
	d.sort_field(&m_lambda[0]);
	d.sort_field(&m_deltaX[0]);
	d.sort_field(&m_oldX[0]);
	d.sort_field(&m_lastX[0]);
}

