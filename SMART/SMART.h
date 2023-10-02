/*******************************************
* Author: Zongyuan Shen. 
* All Rights Reserved. 
*******************************************/
#ifndef SMART_H
#define SMART_H

#define _USE_MATH_DEFINES
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <random>

using namespace std;

struct gridpoint {
	int i;
	int j;
	double x;
	double y;
	double yaw;
};

enum cellStatuslabel {
	unknown = 1,
    inStaticObs = 2,
    inCPR = 3,
    hotSpot = 4,
    nonhotSpot = 5,
};

class node {    
public:
    /***************************/
    /******** Function *********/
    /***************************/
    node(double _x, double _y);
    ~node();
    double getX();
    double getY();
    int getRow();
    int getCol();
    void setTreeID(int index);
    int getTreeID();
    void setParent(node*);
    void setChildren(node*);
    void clearChildren();
    vector<node*> getChildren();
    node* getParent();
    void setDistToRobot(double value);
    double getDistToRobot();
    void setDistToGoal(double value);
    double getDistToGoal();
    void pruneChild(node*);
    void setNeighbors(node*);
    vector<node*> getNeighbors();
    void setDistToNeighbor(double value);
    vector<double> getDistToNeighbor();
    void setOldParent(node*);
    node* getOldParent();
    
    /***************************/
    /******** Variable *********/
    /***************************/
    double x, y;
    int row, col;
    node* parent;
    node* oldParent;
    vector<node*> children;
    double distToRobot;
    double distToGoal;
    int treeID;
    vector<node*> neighbors;
    vector<double> distToNeighbor;
};

struct gridcell {
    double x, y; // coordinate of center
    vector<node*> nodeSet;
    double distToGoal;
    double distToRobot;
    vector<int> treeID;
    double utility;
    int status;
};

class SMART {
public:
    /*********************************/
    /***** Function for planning *****/
    /*********************************/
    SMART(int, double, int, int, vector<vector<double>>, vector<double>, vector<double>);
    ~SMART();

    // Load the binary static obstacle map
    // Note: 1 = free, 0 = occupied
    void staticObsMapLoad();

    // Create a batch of random samples in the space free of static obstacle with associated neighbor information
    void graphCreate();

    // Create initial tree on the sample batch using FMT* algorithm
    void initialTreeCreate();

    // Search shortest initial path that is free of static obstacle
    bool initialPathSearch();

    // Control entire navigation process
    void mainFunction();

    // Compute critical pruning region (CPR) given local reaction zone (LRZ) and obstacle hazard zone (OHZ)
    void updateCPR();

    // Check if robot reaches goal
    bool reachGoal();

    // Check if robot collides with dynamic obstacle
    bool failureOccur();

    // Check if robot moves into an obstacle hazard zone (OHZ)
    // Note: If yes, then the OHZ is ignored but the actual obstacle is considered for collision checking
    bool isRobotInsideOHZ(double, double);

    // Check the feasibility of path nodes and edges in local reaction zone (LRZ) against dynamic obstacle
    bool isPathFeasible();

    // Prune the node and edge within the critical pruning region (CPR)
    void treePruning();

    // Propagate new information of tree index and dist-to-goal to the successor when the node is disconnected or reconnected to a tree
    void propagateNewInfoToSuccessor(node*);

    // Search shortest feasible replanned path
    bool replannedPathSearch();

    // refresh replanner information given new obstacle information if the replanned path is not found in previous iteration
    void refreshReplanner();

    // Clear replanner information once replanned path is found
    void clearReplanner();

    // Repair connectivity of disjoint tree until new replanned path is available
    bool treeRepair();

    // Search hot spot within local search region (LSR)
    bool hotSpotSearch();

    // Update tree index information for a cell
    void updateTreeID(int, int);
    bool isHotSpot(int, int);
    bool cellInsideCPR(int, int);

    // Compute utility for hot spot
    void computeUtility(int, int);
    bool cellInsideArea(int, int);
    bool cellInsideValidLSR(int, int);

    // Take the hot spot and perform tree reconnections in the local neighborhood
    void treeReconnect();

    // propagate the new parent-child relationship over the reconnected disjoint tree
    void resetParent(node*, node*);

    // Optimize the subtrees that are reconnected to the main tree
    bool isOptimizationStartNodeValid(node*);
    void optimizationStartNodeProcess();
    void treeOptimization();
    void rewireCascade(node*);
    void findBestParent(node*);
    void propagateNewDistToGoalToSuccessor(node*);

    // All pruned nodes and the roots of the remaining disjoint trees are reconnected with main tree and a single morphed tree is formed
    void formSingleTree();
    void updateDisjointTreeRoot();

    // Create random sample and join nearby disjoint trees if possible
    void standardTreeRepair();
    node* sampleFree();

    // Return the neighboring nodes in the 3 by 3 local neighborhood of a node
    vector<node*> getNearNode(double, double);

    // Lazy feasibility checking given critical pruning region (CPR)
    bool lazyFeasibilityChecker_edge(node*, node*);
    bool lazyFeasibilityChecker_node(node*);

    // Collision checking given static obstacle map
    bool staticObsFeasibilityChecker_edge(node*, node*);

    // Check if robot reaches a waypoint; if yes, set a new waypoint
    bool reachWaypoint();

    // For data record
    void dataRecord();
    void dynObsInfoRecord();
    void treeInfoRecord();
    void pathInfoRecord();
    void footPrintInfoRecord();
    void trajectoryLengthCompute();

    /*********************************/
    /***** Variables for planning ****/
    /*********************************/
    node* treeRoot; // Main tree root at the goal position
    node* wayPointNode; // One of path nodes for robot navigation
    double TRH; // Time horizon of local reaction zone
    double TOH; // Time horizon of obstacle hazard zone
    double robotSpeed; // User-defined constant value in m/s
    double obstacleSpeed; // User-defined constant value in m/s
    double obstacleRadius; // User-defined constant value in m; robot is simplified as a point mass by adding its radius to the obstacle for collision checking
    double reachThreshold; // Threshold in m to determine if robot reaches a waypoint
    double totalReplanTime, totalTravelTime, averageReplanTime; // in second
    double totalReplanNum;
    double collisionCheckingResolution; // Distance between successive interpolating points on the edge
    double goalX;
    double goalY;
    double robotX;
    double robotY; 
    int dynObsNum; // User-defined number of dynamic obstacle
    int iterationIndex;
    int mainTreeID; // Index of goal-rooted main tree; it is a fixed value
    int newTreeID; // Index of disjoint tree that is disconnected from the main tree
    int LSR_size_current; // Current size of local search region (LSR)
    int LSR_size_max; // Maximum size of local search region (LSR) such that LSR covers the entire space; it is a fixed value
    bool initDone;
    bool replanTrigger;
    bool robotInsideOHZ;
    double LRZ_radius; // Radius of local reaction zone (LRZ)
    double OHZ_radius; // Radius of obstacle hazard zone (OHZ)
    mt19937 localSampleGen;
    uniform_real_distribution<double> localSampleDis; // Randomly draw a sample in a free cell according to uniform distribution
    mt19937 freeCellGen;
    uniform_int_distribution<int> freeCellDis; // Randomly pick a free cell according to uniform distribution
    vector<vector<gridcell>> tiling; // Tiling structure to store tree information
    clock_t startReplanningTime;
    vector<vector<int>> staticObsFreeCell; // A set of cell (row, col) free of static obstacle
    vector<vector<double>> CPR; // Critical pruning region
    vector<node*> path; // Feasible path from main tree
    vector<node*> prunedNode; // Nodes within CPR
    vector<node*> optimizationStartNode; // Start node for rewire cascade
    vector<node*> disjointTreeRoot; // Root of disjoint tree that is disconnected from the main tree
    gridpoint bestHotSpot; // Hot spot with highest utility
    gridpoint searchCenter; // Center cell of local search region (LSR)
    int staticObsMapRow, staticObsMapCol;
    
    // For data record
    vector<vector<double>> dynObsInfo;
    vector<vector<double>> treeInfo;
    vector<vector<double>> pathInfo;
    vector<vector<double>> footPrintInfo;
    
    /*********************************/
    /***** Function for scenario *****/
    /*********************************/
    // Set a waypoint for the dynamic obstacle by randomly picking a moving heading and distance according to uniform distribution
    void dynScenarioSimulate();
    bool obstacleInsideArea(double, double);

    // Prevent the dynamic obstacle from blocking the goal
    bool blockGoalNode(double, double);

    // Given previous position, waypoint and time duration, compute the current pose for the dynamic obstacle
    void simulateObstaclePos();

    // Given previous position, waypoint and time duration, compute the current pose for the robot
    void simulateRobotPos();

    /*********************************/
    /***** Variables for scenario ****/
    /*********************************/
    int scene; // Scenario index; in different scenarios, dynamic obstacle trajetories are different
    double timeDuration; // User-defined loop interval in second
    bool simulationFail, simulationSuccess;
    vector<vector<double>> obsGoal;
    vector<vector<double>> obsPosition;
    vector<mt19937> obstacle_gen;
    uniform_real_distribution<double> obstacle_distance; // Randomly pick a moving distance according to uniform distribution
    uniform_real_distribution<double> obstacle_heading; // Randomly pick a moving heading according to uniform distribution
};
#endif