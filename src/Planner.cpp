#include "Planner.h"
#include "StopWatch.h"
#include <numeric>

Planner::Planner(ros::NodeHandle &nh) : nh_(nh) {
  pub_marker = nh.advertise<visualization_msgs::Marker>("marker_paths", 1);
  pub_marker_shortest = nh.advertise<visualization_msgs::Marker>("marker_path_shortest", 1);
  sub_map = nh.subscribe("/map", 1, &Planner::CallbackMap, this);
}

void Planner::CallbackMap(const nav_msgs::OccupancyGrid::ConstPtr &msg_map) {
  int attempts = 100;
  int inf_counter = 0;
  for (size_t j = 0; j < attempts; ++j) {

    double res = msg_map->info.resolution;
    double width = msg_map->info.width * res;
    double height = msg_map->info.height * res;

    auto marker = MarkerStuff::Lines(std::vector<geometry_msgs::Point>(), "map", 1.0, 0.0, 0.0, 0.1);
    auto marker_shortest = MarkerStuff::Lines(std::vector<geometry_msgs::Point>(), "map", 0.0, 1.0, 0.0, 0.5);

// Construct the robot state space in which we're planning. We're
// planning in [0,1]x[0,1], a subset of R^2.

    StopWatch watch;
    watch.Start();
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
// Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(res / 2 - 0.01, width - res / 2 + 0.01);

//  space->setLongestValidSegmentFraction(0.000125);
//  std::cout << "fracc: " << space->getLongestValidSegmentFraction() << std::endl;


//  space->setStateSamplerAllocator([&msg_map](const ob::StateSpace *space) {
//    std::shared_ptr<MyStateSampler> state_sampler = std::make_shared<MyStateSampler>(space);
//    state_sampler->setMap(msg_map);
//    ob::StateSamplerPtr aa(state_sampler);
//    return aa;
//  });


// Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));


//  std::cout << "StateValidityCheckingResolution: " << si->getStateValidityCheckingResolution() << std::endl;
//  si->setStateValidityCheckingResolution(0.000125);

// Set the object used to check which states in the space are valid
    std::shared_ptr<ValidityChecker> validity_checker = std::make_shared<ValidityChecker>(si);
    validity_checker->setMap(msg_map);
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(validity_checker));



//  si->allocValidStateSampler();

//  si->setStateValidityCheckingResolution(0.0001);
//  si->setLongestValidSegmentFraction()

//  std::shared_ptr<myMotionValidator> motion_validator = std::make_shared<myMotionValidator>(si);
//  motion_validator->setMap(msg_map);
//  si->setMotionValidator(motion_validator);

    si->setup();
// Set our robot's starting state to be the bottom-left corner of
// the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = res / 2;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = res / 2;
// Set our robot's goal state to be the top-right corner of the
// environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 960 * res;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 330 * res;
// Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
// Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // Construct our optimizing planner using the RRTstar algorithm.

    std::shared_ptr<og::RRTstar> rrtstar = std::make_shared<og::RRTstar>(si);
    ob::PlannerPtr optimizingPlanner(rrtstar);
// Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();



// attempt to solve the planning problem within one second of
// planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(0.100);
    ob::PathPtr path3 = pdef->getSolutionPath();
//  path3->print(std::cout);

    std::shared_ptr<oc::PathControl> path = std::static_pointer_cast<oc::PathControl>(pdef->getSolutionPath());

    if (rrtstar->bestCost().value() == std::numeric_limits<double>::infinity()) {
      inf_counter++;
    }

    geometry_msgs::Point pt_old;

    watch.Stop();
    std::cout << "Planning took " << watch.ElapsedMilliSeconds() << " ms." << std::endl;

    for (unsigned int i = 0; i < path->getStateCount(); ++i) {
      const ob::State *state = path->getState(i);

      // We know we're working with a RealVectorStateSpace in this
      // example, so we downcast state into the specific type.
      const ob::RealVectorStateSpace::StateType *state2D =
        state->as<ob::RealVectorStateSpace::StateType>();

      // Extract the robot's (x,y) position from its state
      double x = state2D->values[0];
      double y = state2D->values[1];

      if (i == 0) {
        pt_old.x = x;
        pt_old.y = y;
        continue;
      }

      geometry_msgs::Point pt_new;
      pt_new.x = x;
      pt_new.y = y;

      marker_shortest.points.push_back(pt_old);
      marker_shortest.points.push_back(pt_new);
      pub_marker_shortest.publish(marker_shortest);

//    std::this_thread::sleep_for(std::chrono::milliseconds(25));
//    ros::spinOnce();

      pt_old.x = x;
      pt_old.y = y;
    }

    ob::PlannerData pd(si);
    optimizingPlanner->getPlannerData(pd);


    unsigned int ind_start;
    ind_start = pd.getStartIndex(0);
    // Breadth First Tree Traversal
    std::queue<unsigned int> vertices_to_visit;
    vertices_to_visit.push(ind_start);
    while (!vertices_to_visit.empty()) {
      unsigned int ind_parent = vertices_to_visit.front();
      vertices_to_visit.pop();

      const ob::PlannerDataVertex &vertex_parent = pd.getVertex(ind_parent);
      const auto &state_parent
        = vertex_parent.getState()->as<ob::RealVectorStateSpace::StateType>();

      geometry_msgs::Point pt_old;
      pt_old.x = state_parent->values[0];
      pt_old.y = state_parent->values[1];

      // obtains a list of children
      std::vector<unsigned int> edges_from_parent;
      unsigned int num_edges_out_parent = pd.getEdges(ind_parent, edges_from_parent);


      for (size_t i = 0; i < num_edges_out_parent; ++i) {
        unsigned int ind_child_vertex = edges_from_parent[i];
        const auto &state_child
          = pd.getVertex(ind_child_vertex).getState()->as<ob::RealVectorStateSpace::StateType>();

        geometry_msgs::Point pt_new;
        pt_new.x = state_child->values[0];
        pt_new.y = state_child->values[1];

        marker.points.push_back(pt_old);
        marker.points.push_back(pt_new);
        pub_marker.publish(marker);

        // add child to the queue
        vertices_to_visit.push(ind_child_vertex);
//      std::this_thread::sleep_for(std::chrono::milliseconds(25));
//      ros::spinOnce();
      }

    }




    // Recursive Depth First Traversal
    /*std::function<void(unsigned int)> drawer = [&pd, &pub_marker, &marker, &drawer](unsigned int ind_parent) {
      const ob::PlannerDataVertex &vertex_parent = pd.getVertex(ind_parent);
      const auto &state_parent
        = vertex_parent.getState()->as<ob::RealVectorStateSpace::StateType>();

      geometry_msgs::Point pt_old;
      pt_old.x = state_parent->values[0];
      pt_old.y = state_parent->values[1];

      // obtains a list of children
      std::vector<unsigned int> edges_from_parent;
      unsigned int num_edges_out_parent = pd.getEdges(ind_parent, edges_from_parent);

      // stops when there are no child
      if (num_edges_out_parent == 0)
        return;

      // draws connections
      for (size_t i = 0; i < num_edges_out_parent; ++i) {
        unsigned int ind_child_vertex = edges_from_parent[i];
        const auto &state_child
          = pd.getVertex(ind_child_vertex).getState()->as<ob::RealVectorStateSpace::StateType>();

        geometry_msgs::Point pt_new;
        pt_new.x = state_child->values[0];
        pt_new.y = state_child->values[1];

        marker.points.push_back(pt_old);
        marker.points.push_back(pt_new);
        pub_marker.publish(marker);

        // calls itself for each child
        drawer(ind_child_vertex);
  //      std::this_thread::sleep_for(std::chrono::milliseconds(25));
  //      ros::spinOnce();
      }
    };
    drawer(ind_start);*/
  }

  std::cout << "failed ratio: " << (float) inf_counter / attempts << std::endl;
}
