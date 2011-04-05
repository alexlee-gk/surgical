#ifndef _thread_RRT_h
#define _thread_RRT_h

#include "../DiscreteRods/thread_discrete.h"
#include "../DiscreteRods/threadutils_discrete.h"
#include "linearization_utils.h"
#include "rrt_utils.h" 
#include <lshkit.h> 
#include "lsh-table.h" 
#include <Eigen/Geometry>
#include <vector>
#include <set>

using namespace std;
using namespace lshkit; 

class Thread_RRT
{
 public:

  //typedef RRTNode* Key;
  //typedef float* Value;
  //typedef float* Domain;
  //typedef RRTNode::Accessor ACCESSOR;
  //typedef lshkit::metric::l2<float> METRIC;
  //typedef lshkit::HyperPlaneLsh LSH;

  Thread_RRT();
  ~Thread_RRT();

  //void planPath(const Thread* start, const Thread* goal, vector<Two_Motions>& movements);
  void initialize(const Thread* start, const Thread* goal);
  //void planStep(VectorXd* goal, VectorXd* prev, VectorXd* next_thread);
  void planStep(Thread& new_sample_thread, Thread& closest_sample_thread, Thread& new_extend_thread);
  void updateBestPath(); 
  vector<RRTNode*>* getTree() { return &_tree; }
  Thread* generateSample(const Thread* start);
  Thread* generateSample(int N); 

  double distanceBetween(const Thread* start, const Thread* end) {
    RRTNode* sNode = new RRTNode(start);
    RRTNode* eNode = new RRTNode(end);
    double score = utils.distanceBetween(sNode, eNode); 
    delete sNode; 
    delete eNode; 
    return score;
  }

  double l2PointsDifference(const Thread* start, const Thread* end) { 
    RRTNode* sNode = new RRTNode(start);
    RRTNode* eNode = new RRTNode(end);
    double score = utils.l2PointsDifference(sNode, eNode); 
    delete sNode; 
    delete eNode; 
    return score;
  }

  RRTNode* findClosestNode(const Thread* target, bool approximateNode=true);
  typedef Repeat<HyperPlaneLsh> HASH; 
 
 private:
  vector<RRTNode*> _tree;
//  VectorXd _goal;
//  Matrix3d _goal_rot;
//  const Thread* _start_thread;
//  const Thread* _goal_thread;

  const RRTNode* _start_node; 
  const RRTNode* _goal_node;

  void insertIntoRRT(RRTNode* node);  
//  void getNextGoal(VectorXd* next, Matrix3d* next_rot);
  Thread* getNextGoal();
  //double extendToward(const VectorXd& next, const Matrix3d& next_rot);
  double extendToward(Thread* target);
  double extendAsFarToward(Thread* target);
  //double largeRotation(const VectorXd& next);
  double largeRotation(const Thread* target);
  //RRTNode* findClosestNode(const VectorXd& next);


//  void simpleInterpolation(const Vector3d& cur_pos, const Matrix3d& cur_rot, const Vector3d& next, const Matrix3d& next_rot, Vector3d* res_translation, Matrix3d* res_rotation);
  void simpleInterpolation(Thread* start, const Thread* end, vector<Two_Motions*>& motions);

  double distToGoal;
  double bestDist;
  double TOLERANCE;
  double totalPenalty; 
//  VectorXd next;
//  Matrix3d next_rot;
  Thread* next_thread;
  //lshkit::LshIndex<lshkit::HyperPlaneLsh, RRTNode* > *index; 
  LshMultiTable<HASH>*index; 

  RRTNodeUtils utils; 
  
};

#endif
