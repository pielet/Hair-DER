#ifndef __SCENE_STEPPER__
#define __SCENE_STEPPER__

#include "MathDefs.h"
#include "MathUtilities.h"

class SceneStepper
{
protected:
	std::vector<scalar> m_timing_statistics;
	VectorXs m_next_x;

	virtual void setNextX() = 0;
	virtual void initNextX(size_t size);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~SceneStepper();

	virtual int getVertNum() const = 0;
	virtual int getStrandNum() const = 0;
	virtual int getDof(int idx) const;
	
	virtual bool stepScene() = 0;

	virtual void applyRigidTransform(scalar t) = 0;

	virtual std::string getName() const = 0;

	virtual const VectorXs& getNextX() const;
	virtual const std::vector<scalar>& getTimingStatistics() const;
	
	virtual void write(std::vector<scalar>&) const;
	virtual void read(const scalar* data);
	
	virtual size_t size();
};

#endif
