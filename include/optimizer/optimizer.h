#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include "../map/map.h"

//G2O
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;


class SingleMapOptimizer
{
public:
	SingleMapOptimizer(Map * pMap);

	void SetMap(Map * pNewMap);
	Map * GetMap();
	
	void Optimize(int nIterations = 100);

	void OptimizeCommonRegion(int nIterations = 100);

	void OptimizeCascade(int nIterations = 100);

private: 
	Map * m_pMap;
	
};


inline void SingleMapOptimizer::SetMap(Map * pNewMap){
	this->m_pMap = pNewMap;
}

inline Map * SingleMapOptimizer::GetMap(){
	return this->m_pMap;
}



#endif