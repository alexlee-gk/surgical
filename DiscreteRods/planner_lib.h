#define NUM_ITERS_SQP 2
#define SQP_BREAK_THRESHOLD 2

#include "../MotionPlanning/worldSQP.h"

/*
 * Use SQP solver given traj_in. Puts results in traj_out and control_out
 */
void solveSQP(vector<World*>& traj_in, vector<World*>& traj_out, vector<VectorXd>& control_out, vector<vector<World*> >& sqp_debug_data, const char* namestring)
{
  int num_iters = NUM_ITERS_SQP; 

  // Wrap controls and put threads in traj_out as copies 
  traj_out.resize(traj_in.size());
  for (int i = 0; i < traj_in.size(); i++) {
    traj_out[i] = new World(*traj_in[i]);
  }
  VectorXd initialState;
  traj_in[0]->getStateForJacobian(initialState);

  WorldSQP *wsqp = 
    new WorldSQP(traj_out.size(), initialState.size());
  wsqp->set_namestring(namestring);
  wsqp->iterative_control_opt(traj_out, control_out, sqp_debug_data, num_iters);

};

void solveSQP(vector<World*>& traj_in, vector<World*>& traj_out, vector<VectorXd>& control_out, const char* namestring)
{
  vector<vector<World*> > sqp_debug_data;
  solveSQP(traj_in, traj_out, control_out, sqp_debug_data, namestring);
};


void sample_on_sphere(VectorXd& u, const double norm) {
  // sample coordinate wise 
  for (int i = 0; i < u.size(); i++) { 
    u(i) = drand48();
  }
  // place on the unit-ball
  u.normalize();
  // coordinate wise transform to norm-ball 
  for(int i = 0; i < u.size(); i++) { 
    if (drand48() < 0.5) { 
      u(i) *= -norm;
    } else { 
      u(i) *= norm; 
    }
  }
}

void openLoopController(vector<World*> traj_in, vector<VectorXd>& controls_in, vector<World*>& traj_out) {
  World* world = new World(*traj_in[0]);
  for (int i = 0; i < controls_in.size(); i++) {
    traj_out.push_back(new World(*world));
    world->applyRelativeControlJacobian(controls_in[i]);
  }
  traj_out.push_back(new World(*world));
}

double l2PointsDifference(VectorXd& a, VectorXd& b) {
  return (a-b).norm();
}

double l2PointsDifference(World* a, World* b) { 
  VectorXd state_a, state_b;
  a->getStateForJacobian(state_a);
  b->getStateForJacobian(state_b);
  return l2PointsDifference(state_a, state_b); 
}


double cost_metric(World* a, World* b) { 
  return l2PointsDifference(a,b);
}

void getTrajectoryStatistics(vector<World*>& worlds) {
  // curvature
  // energy
  // twist
  // velocity
  // collisions
  // sensitivity to control?
  // topological? 

  for (int i = 0; i < worlds.size(); i++) { 
    vector<ThreadConstrained*> world_threads;
    worlds[i]->getThreads(world_threads); // not copies, so don't mess with it
    for (int j = 0; j < world_threads.size(); j++) {
      vector<Thread*> threads;
      world_threads[j]->getThreads(threads); 
      cout << threads.front()->calculate_energy() << endl; //energy 
    }
  }
}
