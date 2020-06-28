#ifndef __PBD_COLLISION_SOLVER__
#define __PBD_COLLISION_SOLVER__

#include <utility>
#include <unordered_map>

#include "MathDefs.h"
#include "MathUtilities.h"

class PositionOrientation;

typedef std::pair<unsigned, unsigned> segment_pair;

struct CollisionInfo {
	scalar alpha, beta;
	const scalar init_C;

	CollisionInfo(scalar C) : alpha(0), beta(0), init_C(C) {};
	void setParam(scalar a, scalar b) { alpha = a; beta = b; }
};

namespace std {
	template<>
	struct hash<segment_pair>
	{
		typedef size_t result_size;
		typedef segment_pair argument_size;
		size_t operator() (const segment_pair& sp) const {
			return hash<unsigned>()(sp.first) ^ hash<unsigned>()(sp.second);	// unorder
		}
	};
};

typedef std::pair<const segment_pair, CollisionInfo> collision_pair;
typedef std::unordered_map<segment_pair, CollisionInfo> CollisionPair;


class PBDCollisionSolver
{
public:
	PBDCollisionSolver(PositionOrientation* POBD);
	~PBDCollisionSolver();

	bool addPair(unsigned seg_idx1, unsigned seg_idx2, CollisionPair* target = nullptr);
	void solveCollision();
	void updateVelocity();
	void clear();                             

private:
	PositionOrientation* m_dynamics;

	CollisionPair m_collision_pair;
	CollisionPair m_new_pair;
	VectorXi m_segment_collision_num;

	bool solvePairCollision(collision_pair&);
};


#endif // !__PBD_COLLISION_SOLVER__
