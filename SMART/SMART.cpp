/*******************************************
* Author: Zongyuan Shen. 
* All Rights Reserved. 
*******************************************/
#include "SMART.h"

/***************************/
/***** Global variable *****/
/***************************/
const double cellSize = 1.0;
const int invalidTreeID = 0;

/***************************/
/******** Node Class *******/
/***************************/
node::node(double _x, double _y) : x(_x), y(_y), row(floor(_y / cellSize)), col(floor(_x / cellSize)),
parent(nullptr), distToRobot(numeric_limits<double>::max()), distToGoal(numeric_limits<double>::max()), treeID(invalidTreeID){}
node::~node(void){}

double node::getX() {return x;}
double node::getY() {return y;}
int node::getRow() {return row;}
int node::getCol() {return col;}
void node::setParent(node* _parent) {parent = _parent;}
node* node::getParent() {return parent;}
void node::setOldParent(node* _parent) {oldParent = _parent;}
node* node::getOldParent() {return oldParent;}
void node::setChildren(node* _child) {children.push_back(_child);}
vector<node*> node::getChildren() {return children;}
void node::clearChildren() {vector <node*>().swap(children);}
void node::pruneChild(node* _child) 
{
	for (vector<node*>::iterator it = children.begin(); it != children.end();)
	{
		if (*it == _child)
		{
			it = children.erase(it);
			break;
		}
		++it;
	}
}
void node::setDistToRobot(double value) {distToRobot = value;}
double node::getDistToRobot() {return distToRobot;}
void node::setDistToGoal(double value) {distToGoal = value;}
double node::getDistToGoal() {return distToGoal;}
void node::setTreeID(int index) {treeID = index;}
int node::getTreeID() {return treeID;}
void node::setNeighbors(node* _neighbor) {neighbors.push_back(_neighbor);}
vector<node*> node::getNeighbors() {return neighbors;}
void node::setDistToNeighbor(double _value){distToNeighbor.push_back(_value);}
vector<double> node::getDistToNeighbor(){return distToNeighbor; }

/***************************/
/******* SMART Class *******/
/***************************/
SMART::SMART(int _trialIndex, double _obstacleSpeed, int scene_index, int _dynObsNum, vector<vector<double>> dynObsPosition, vector<double> robotInitState, vector<double> goalState): 
treeRoot(new node(goalState[0], goalState[1])), TRH(0.8), TOH(0.4), robotSpeed(robotInitState[2]), obstacleSpeed(_obstacleSpeed), obstacleRadius(1.0), 
reachThreshold(0.5), totalReplanTime(0.0), totalTravelTime(0.0), averageReplanTime(0.0), totalReplanNum(0), collisionCheckingResolution(0.1),
goalX(goalState[0]), goalY(goalState[1]), robotX(robotInitState[0]), robotY(robotInitState[1]), dynObsNum(_dynObsNum), iterationIndex(0), mainTreeID(1), newTreeID(1), 
LSR_size_current(3), initDone(false), replanTrigger(false), robotInsideOHZ(false),
localSampleGen(_trialIndex), freeCellGen(_trialIndex), localSampleDis(uniform_real_distribution<double>(- cellSize / 2, cellSize / 2)),
freeCellDis(uniform_int_distribution<int>(0, 1)),
scene(scene_index), timeDuration(0.1), simulationFail(false), simulationSuccess(false),
obstacle_distance(uniform_real_distribution<double>(0.0, 10.0)), 
obstacle_heading(uniform_real_distribution<double>(0.0, 2.0 * M_PI))
{
	LRZ_radius = TRH * robotSpeed;
	OHZ_radius = TOH * obstacleSpeed + obstacleRadius;
	for (int i = (scene - 1) * dynObsNum + 1; i < (scene - 1) * dynObsNum + 1 + dynObsNum; i++)
	{
		mt19937 temp_gen(i); // generate random number sequence with seed i
		obstacle_gen.push_back(temp_gen);
	}
	searchCenter.i = -1;
	searchCenter.j = -1;
	for (int i = 0; i < dynObsNum; i++)
	{
		obsPosition.push_back({dynObsPosition[i][0],dynObsPosition[i][1]});
	}
	obsGoal = obsPosition;
}

SMART::~SMART(void)
{
	std::cout << "SMART object is being deleted" << endl;
	// Delete node object to avoid memory leak
	for (int row = 0; row < staticObsMapRow; row++)
	{
		for (int col = 0; col < staticObsMapCol; col++)
		{
			vector<node*> nodeSet = tiling[row][col].nodeSet;
			for (int node_id = 0; node_id < nodeSet.size(); node_id++) delete nodeSet[node_id];
		}
	}
}

/*********************************/
/***** Function for scenario *****/
/*********************************/
void SMART::dynScenarioSimulate() 
{
	for (int obs_id = 0; obs_id < dynObsNum; obs_id++)
	{
		double dist_sq = (obsGoal[obs_id][0] - obsPosition[obs_id][0]) * (obsGoal[obs_id][0] - obsPosition[obs_id][0]) + (obsGoal[obs_id][1] - obsPosition[obs_id][1]) * (obsGoal[obs_id][1] - obsPosition[obs_id][1]);
		if (dist_sq < reachThreshold * reachThreshold) // Obstacle reaches the waypoint
		{
			while (true)
			{
				// Compute new waypoint for the obstacle given moving distance and heading
				double dist = obstacle_distance(obstacle_gen[obs_id]); // Select a random moving distance
				double heading = obstacle_heading(obstacle_gen[obs_id]); // Select a random heading
				obsGoal[obs_id][0] = obsPosition[obs_id][0] + cos(heading) * dist;
				obsGoal[obs_id][1] = obsPosition[obs_id][1] + sin(heading) * dist;
				// Check the feasibility of new waypoint
				if (obstacleInsideArea(obsGoal[obs_id][0], obsGoal[obs_id][1]) == true && blockGoalNode(obsGoal[obs_id][0], obsGoal[obs_id][1]) == false)
				{
					break;
				}
			}
		}
	}
}

bool SMART::obstacleInsideArea(double x, double y)
{
	double spaceXmax = (double) staticObsMapCol * cellSize;
	double spaceYmax = (double) staticObsMapRow * cellSize;
	if (x >  obstacleRadius  && x < spaceXmax - obstacleRadius && y > obstacleRadius && y < spaceYmax - obstacleRadius) return true;
	return false;
}

bool SMART::blockGoalNode(double x, double y)
{
	double threshold = 4.0 + obstacleRadius;
	double dist = sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
	if (dist < threshold) return true;
	return false;
}

void SMART::simulateObstaclePos()
{
	for (int obs_id = 0; obs_id < dynObsNum; obs_id++)
	{
		double waypoint_x = obsGoal[obs_id][0];
		double waypoint_y = obsGoal[obs_id][1];
		double distToWaypoint_sq = (waypoint_x - obsPosition[obs_id][0]) * (waypoint_x - obsPosition[obs_id][0]) + (waypoint_y - obsPosition[obs_id][1]) * (waypoint_y - obsPosition[obs_id][1]);
		if (distToWaypoint_sq > 0)
		{
			// Compute new obstacle pose given moving distance and heading
			double heading = atan2(waypoint_y - obsPosition[obs_id][1], waypoint_x - obsPosition[obs_id][0]);
			double dist = obstacleSpeed * timeDuration;
			obsPosition[obs_id][0] += cos(heading) * dist;
			obsPosition[obs_id][1] += sin(heading) * dist;
			obsPosition[obs_id][2] = heading;
		}
	}
}

void SMART::simulateRobotPos()
{
 	double waypoint_x = wayPointNode->getX();
	double waypoint_y = wayPointNode->getY();
	double distToWaypoint_sq = (waypoint_x - robotX) * (waypoint_x - robotX) + (waypoint_y - robotY) * (waypoint_y - robotY);
	if (distToWaypoint_sq > 0)
	{
		// Compute new robot pose given moving distance and heading
		double heading = atan2(waypoint_y - robotY, waypoint_x - robotX);
		double dist = robotSpeed * timeDuration;
		robotX += cos(heading) * dist;
		robotY += sin(heading) * dist;
		// Update distToRobot for each free cell
		for (int row = 0; row < staticObsMapRow; row++)
		{
			for (int col = 0; col < staticObsMapCol; col++)
			{
				if (tiling[row][col].status == inStaticObs) continue;
				double x = tiling[row][col].x;
				double y = tiling[row][col].y;
				tiling[row][col].distToRobot = sqrt((x - robotX) * (x - robotX) + (y - robotY) * (y - robotY));
			}
		}
	}
}

/*********************************/
/***** Function for planning *****/
/*********************************/
void SMART::staticObsMapLoad()
{
	char buff[128];
	ifstream file;
	ifstream file2;
	file.open("staticObsMap.txt");
	file2.open("staticObsMap.txt");
	staticObsMapRow = 0;
	staticObsMapCol = 0;
	string line, item;
	while (getline( file2, line ) )
	{
		staticObsMapRow++;
		if ( staticObsMapRow == 1)                 // First row only: determine the number of columns
		{
			 stringstream ss( line );      // Set up up a stream from this line
			 while ( ss >> item ) staticObsMapCol++;  // Each item delineated by spaces adds one to cols
		}
	}
	file2.close();

	std::cout << "\nFile had " << staticObsMapRow << " rows and " << staticObsMapCol << " column" << endl;

	vector<vector<int>> staticObsMap;

	if (file.is_open())
	{
		printf("Load database successfully...\n");
		int x;
		for (int i = 0; i < staticObsMapRow; i++)
		{
			vector<int> dataRow;
			for (int j = 0; j < staticObsMapCol; j++)
			{
				file >> x;
				dataRow.push_back(x);
			}
			staticObsMap.push_back(dataRow);
		}
		file.close();
	}
	else
	{
		printf("Cannot load database...\n");
	}

	for (int i = 0; i < staticObsMapRow; i++)
	{
		for (int j = 0; j < staticObsMapCol; j++)
		{
			std::cout<<staticObsMap[i][j]<<' ';
		}
		std::cout<<endl;
	}
	for (int row = 0; row < staticObsMapRow; row++)
	{
		vector<gridcell> rowData;
		for (int col = 0; col < staticObsMapCol; col++)
		{
			gridcell element;
			double x = double(col) * cellSize + cellSize / 2;
			double y = double(row) * cellSize + cellSize / 2;
			element.x = x;
			element.y = y;
			element.distToGoal = sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
			element.distToRobot = sqrt((x - robotX) * (x - robotX) + (y - robotY) * (y - robotY));
			element.utility = 0;
			if (staticObsMap[row][col] == 1) element.status = inStaticObs;
			else element.status = unknown;
			rowData.push_back(element);
			if (staticObsMap[row][col] == 0) staticObsFreeCell.push_back({row, col});
		}
		tiling.push_back(rowData);
	}
	freeCellDis = std::uniform_int_distribution<int>(0, staticObsFreeCell.size() - 1);
	LSR_size_max = 2* max(staticObsMapRow, staticObsMapCol) - 1;
}

void SMART::graphCreate()
{
	tiling[treeRoot->getRow()][treeRoot->getCol()].nodeSet.push_back(treeRoot);
	int freeCellNum = staticObsFreeCell.size();
	for(int cell_id = 0; cell_id < freeCellNum; cell_id++)
	{	
		// Create random sample in each free cell
		if (staticObsFreeCell[cell_id][0] == treeRoot->getRow() && staticObsFreeCell[cell_id][1] == treeRoot->getCol()) continue;
		double samplingCellX = (double) staticObsFreeCell[cell_id][1] * cellSize + cellSize / 2;
		double samplingCellY = (double) staticObsFreeCell[cell_id][0] * cellSize + cellSize / 2;
		double randX = localSampleDis(localSampleGen) + samplingCellX;
		double randY = localSampleDis(localSampleGen) + samplingCellY;
		node* newNode = new node(randX, randY);
		tiling[newNode->getRow()][newNode->getCol()].nodeSet.push_back(newNode);
		// Check 3 by 3 neighborhood of new node's cell and update neighboring node information for the new node
		int node_i = newNode->getRow();
		int node_j = newNode->getCol();
		for (int local_i = -1; local_i <= 1; local_i++)
		{
			for (int local_j = -1; local_j <= 1; local_j++) 
			{
				int cell_i = node_i + local_i;
				int cell_j = node_j + local_j;
				if (cellInsideArea(cell_i, cell_j) == true) // Ignore the cell outside the space
				{
					vector<node*> neighborNodeSet = tiling[cell_i][cell_j].nodeSet; // Read the nodes in the neighboring cell
					int neighborNodeNum = neighborNodeSet.size();
					for (int neighbor_i = 0; neighbor_i < neighborNodeNum; neighbor_i++)
					{
						node* neighborNode = neighborNodeSet[neighbor_i];
						if (staticObsFeasibilityChecker_edge(neighborNode, newNode) == true) // Ignore the unreachable node
						{
							double dist = sqrt((neighborNode->getX() - newNode->getX()) * (neighborNode->getX() - newNode->getX()) + (neighborNode->getY() - newNode->getY()) * (neighborNode->getY() - newNode->getY()));
							newNode->setNeighbors(neighborNode);
							newNode->setDistToNeighbor(dist); // Store the distance value to avoid online computation
							neighborNode->setNeighbors(newNode);
							neighborNode->setDistToNeighbor(dist);
						}
					}
				}
			}
		}
	}
}

void SMART::initialTreeCreate()
{
	graphCreate();
	treeRoot->setDistToGoal(0);
	treeRoot->setTreeID(mainTreeID);
	vector<node*> closedSet = {treeRoot};
	vector<node*> openSet = treeRoot->getNeighbors();
	vector<double> distToNeighbor = treeRoot->getDistToNeighbor();
	for (int i = 0; i < openSet.size(); i++) 
	{
		openSet[i]->setParent(treeRoot);
		openSet[i]->setDistToGoal(distToNeighbor[i]);
		openSet[i]->setTreeID(treeRoot->getTreeID());
		treeRoot->setChildren(openSet[i]);
	}
	struct 
	{
		bool operator()(node* a, node* b) const { return (a->getDistToGoal() > b->getDistToGoal()); }
	} GreatSort;
	while (!openSet.empty())
	{
		// Pick the best node from open set
		sort(openSet.begin(), openSet.end(), GreatSort);
		node* node_z = openSet.back();
		vector<node*> nearNode_z = node_z->getNeighbors();
		// Set parent for each of z's near nodes that is not visited yet
		for (int i = 0; i < nearNode_z.size(); i++) 
		{
			node* node_x = nearNode_z[i];
			if(std::find(openSet.begin(), openSet.end(), node_x) == openSet.end() && std::find(closedSet.begin(), closedSet.end(), node_x) == closedSet.end())
			{
				vector<node*> nearNode_x = node_x->getNeighbors();
				vector<double> distToNeighbor_x = node_x->getDistToNeighbor();
				double minDistToGoal = numeric_limits<double>::max();
				int minID = -1;
				// Find parent from open set
				for (int i = 0; i < nearNode_x.size(); i++) 
				{
					node* node_y = nearNode_x[i];
					if(std::find(openSet.begin(), openSet.end(), node_y) != openSet.end())
					{
						double newDistToGoal = distToNeighbor_x[i] + node_y->getDistToGoal();
						if (newDistToGoal < minDistToGoal)
						{
							minDistToGoal = newDistToGoal;
							minID = i;
						}
					}
				}
				if (minID != -1)
				{
					node_x->setParent(nearNode_x[minID]);
					node_x->setDistToGoal(minDistToGoal);
					node_x->setTreeID(nearNode_x[minID]->getTreeID());
					nearNode_x[minID]->setChildren(node_x);
					openSet.push_back(node_x);
				}
			}
		}
		// Remove node_z from open set
		auto it = openSet.begin();
		while ( it != openSet.end())
		{
			if (*it == node_z)
			{
				it = openSet.erase(it);
				break;
			}
			else
			{
				it++;
			}
		}
		// Mark node_z as closed 
		closedSet.push_back(node_z);
	}
}

bool SMART::initialPathSearch()
{
	node* temp;
	int minID = -1;
    double minDistToGoal = numeric_limits<double>::max();
	vector<node*> neighbor = getNearNode(robotX,robotY);
	node* robot = new node(robotX,robotY);
	for (int node_id = 0; node_id < neighbor.size(); node_id++)
	{
		if (neighbor[node_id]->getTreeID() != mainTreeID) continue;
		if (staticObsFeasibilityChecker_edge(robot, neighbor[node_id]) == true)
		{
			double dist = sqrt((robotX - neighbor[node_id]->getX()) * (robotX - neighbor[node_id]->getX()) + (robotY - neighbor[node_id]->getY()) * (robotY - neighbor[node_id]->getY()));
			double newDistToGoal = dist + neighbor[node_id]->getDistToGoal();
			if (newDistToGoal < minDistToGoal)
			{
				minDistToGoal = newDistToGoal;
				minID = node_id;
			}
		}
	}
	delete robot;
    if (minID != -1)
	{ 
		temp = neighbor[minID];
		path.push_back(temp);
		while(temp->getParent() != nullptr)
		{
			temp = temp->getParent();
			path.push_back(temp);
		}
		reverse(path.begin(), path.end());
		return true;
	}
	return false;
}

void SMART::mainFunction()
{
	if(initDone == false)
	{
		staticObsMapLoad(); // Load static obstacle map for collision checking
		initialTreeCreate(); // Create initial tree
		initialPathSearch(); // Search initial path
		wayPointNode = path.back(); // Set the waypoint
		initDone = true;
		printf("Initialization is done...\n");
		dataRecord();
	}
	if (replanTrigger == false) simulateRobotPos(); // Update robot pose during navigation
	dynScenarioSimulate(); // Simulation dynamic obstacle random motion
	simulateObstaclePos(); // Update dynamic obstacle pose
	updateCPR(); // Update critical pruning region
	simulationSuccess = reachGoal(); // Check if robot reaches the goal
	if (simulationSuccess == false)
	{
		simulationFail = failureOccur(); // Check if collision happens
		if (simulationFail == false)
		{
			robotInsideOHZ = isRobotInsideOHZ(robotX, robotY); // Check if robot is inside OHZ
			if (isPathFeasible() == false) // Check feasibility of current path
			{
				// Current path is infeasible, trigger replanning step
				if (replanTrigger == false)
				{
					vector<node*>().swap(path);
					startReplanningTime = clock();
					replanTrigger = true;
					totalReplanNum ++;
				}
				// Perform tree-pruning step
				treePruning();
				if (replannedPathSearch() == true)
				{
					// Compute the replanning time
					totalReplanTime += (double)(clock() - startReplanningTime) / CLOCKS_PER_SEC;
					// Reset the first waypoint
					wayPointNode = path.back();
					replanTrigger = false;
					formSingleTree();
					clearReplanner();
				}
				else
				{
					if (replanTrigger == true) refreshReplanner();
					bool repairDone = treeRepair();
					vector<node*>().swap(path);
					optimizationStartNodeProcess();
					treeOptimization();
					// Search a new path from the tree
					if (replannedPathSearch() == true)
					{
						// Compute the replanning time
						totalReplanTime += (double)(clock() - startReplanningTime) / CLOCKS_PER_SEC;
						// Reset the first waypoint
						wayPointNode = path.back();
						replanTrigger = false;
						formSingleTree();
						clearReplanner();
					}
				}
			}
			else reachWaypoint();
		}
	}	
	dataRecord();
}

void SMART::updateCPR()
{
	vector<vector<double>>().swap(CPR);
	for (int obs_id = 0; obs_id < dynObsNum; obs_id++)
	{
		double obsX =  obsPosition[obs_id][0];
        double obsY =  obsPosition[obs_id][1];
		double distToRobot = sqrt((robotX - obsX) * (robotX - obsX) + (robotY - obsY) * (robotY - obsY));
		if (distToRobot <= LRZ_radius + OHZ_radius) CPR.push_back({obsX, obsY, double(obs_id)});
	}
}

bool SMART::reachGoal()
{
	double dist_sq = (goalX - robotX) * (goalX - robotX) + (goalY - robotY) * (goalY - robotY);
	if (dist_sq < reachThreshold * reachThreshold)
	{
		std::cout<<".............................................................................................."<<std::endl;
		std::cout<<"Reach goal!!!"<<endl;
		std::cout<<"obstacle number ="<<dynObsNum<<endl;
		std::cout<<"obstacle speed ="<<obstacleSpeed<<endl;
		std::cout<<"robot speed ="<<robotSpeed<<endl;
		std::cout<<"Total replanning time ="<<totalReplanTime<<endl;
		std::cout<<"Total replanning number ="<<totalReplanNum<<endl;
		if (totalReplanNum >0 )
			averageReplanTime = totalReplanTime/totalReplanNum;
		else
			averageReplanTime = 0;
		std::cout<<"Average replanning time ="<<averageReplanTime<<endl;
		trajectoryLengthCompute();
		std::cout<<".............................................................................................."<<endl;
		return true;
	}
	return false;
}

bool SMART::failureOccur()
{
	for (int index = 0; index < dynObsNum; index++)
	{
		double dist_sq = (obsPosition[index][0] - robotX) * (obsPosition[index][0] - robotX) + (obsPosition[index][1] - robotY) * (obsPosition[index][1] - robotY);
		if (dist_sq <= obstacleRadius * obstacleRadius) 
		{
			std::cout<<".............................................................................................."<<endl;
			std::cout<<"Collision!!!"<<endl;
			std::cout<<"obstacle number ="<<dynObsNum<<endl;
			std::cout<<"obstacle speed ="<<obstacleSpeed<<endl;
			std::cout<<"robot speed ="<<robotSpeed<<endl;
			std::cout<<"Total replanning time ="<<totalReplanTime<<endl;
			std::cout<<"Total replanning number ="<<totalReplanNum<<endl;
			if (totalReplanNum >0 )
				averageReplanTime = totalReplanTime/totalReplanNum;
			else
				averageReplanTime = 0;
			std::cout<<"Average replanning time ="<<averageReplanTime<<endl;
			std::cout<<".............................................................................................."<<endl;
			return true;
		}
		
	}
	return false;
}

bool SMART::isRobotInsideOHZ(double x, double y)
{
	// Check if robot moves into CPR
	int CPRNum = CPR.size();
	for (int CPR_id = 0; CPR_id < CPRNum; CPR_id++)
	{
		double CPR_x = CPR[CPR_id][0];
		double CPR_y = CPR[CPR_id][1];
		double distToRobot = sqrt((robotX - CPR_x) * (robotX - CPR_x) + (robotY - CPR_y) * (robotY - CPR_y));
		if (distToRobot <= OHZ_radius) return true;
	}
	return false;
}

bool SMART::isPathFeasible()
{
	if (path.empty()) return false;
	double buffer;
	if (robotInsideOHZ == true) buffer = obstacleRadius;
	else buffer = OHZ_radius;
	node* robot = new node(robotX, robotY);
	vector<node*> checkPath = path;
	checkPath.push_back(robot);
	int nodeNum = checkPath.size();
	for(int node_i = nodeNum - 1; node_i > 0; node_i--)
	{
		node* nodeA = checkPath[node_i];
		node* nodeB = checkPath[node_i-1];
		double alp = atan2 (nodeB->getY() - nodeA->getY(), nodeB->getX() - nodeA->getX());
		double dist = sqrt((nodeA->getX() - nodeB->getX()) * (nodeA->getX() - nodeB->getX()) + (nodeA->getY() - nodeB->getY()) * (nodeA->getY() - nodeB->getY()));
		int N = ceil(dist / collisionCheckingResolution);
		double x, y;
		for (int i = 0; i <= N; i++)
		{
			if (N > 0)
			{
				x = nodeA->getX() + cos(alp) * dist * i / N;
				y = nodeA->getY() + sin(alp) * dist * i / N;
			}
			else
			{
				x = nodeA->getX();
				y = nodeA->getY();
			}
			double distance = sqrt((robotX - x) * (robotX - x) + (robotY - y) * (robotY - y));
			if (distance > LRZ_radius) continue; // Ignore the point outside local reaction zone (LRZ)
			for (auto item : CPR)
			{
				if (sqrt((item[0] - x) * (item[0] - x) + (item[1] - y) * (item[1] - y)) <= buffer)
				{
					searchCenter.i = floor(y / cellSize);
					searchCenter.j = floor(x / cellSize);
					delete robot;
					return false;
				}
			}
		}
	}
	delete robot;
	return true;
}

void SMART::refreshReplanner()
{
	LSR_size_current = 3;
	for (int row = 0; row < staticObsMapRow; row++)
	{
		for (int col = 0; col < staticObsMapCol; col++)
		{
			vector<int>().swap(tiling[row][col].treeID);
			tiling[row][col].utility = 0;
			if (tiling[row][col].status != inStaticObs)
			{
				if (cellInsideCPR(row, col) == true) tiling[row][col].status = inCPR;
				else tiling[row][col].status = unknown;
			}
		}
	}
}

void SMART::clearReplanner()
{
	newTreeID = 1;
	int disjointTreeRootNum = disjointTreeRoot.size();
	for (int node_id = 0; node_id < disjointTreeRootNum; node_id++)
	{
		node* checkNode = disjointTreeRoot[node_id];
		newTreeID++;
		checkNode->setTreeID(newTreeID); // Set a tree index that has not been used
		propagateNewInfoToSuccessor(checkNode); // Propagate the distToGoal and tree index to its successors
	}
	vector<node*>().swap(disjointTreeRoot);
	LSR_size_current = 3;
	searchCenter.i = -1;
	searchCenter.j = -1;
	for (int row = 0; row < staticObsMapRow; row++)
	{
		for (int col = 0; col < staticObsMapCol; col++)
		{
			vector<int>().swap(tiling[row][col].treeID);
			tiling[row][col].utility = 0;
			if (tiling[row][col].status != inStaticObs) tiling[row][col].status = unknown;
		}
	}
}

void SMART::treePruning()
{
	double buffer;
	if (robotInsideOHZ == true) buffer = obstacleRadius;
	else buffer = OHZ_radius;
	int CPRNum = CPR.size();
	for (int CPR_id = 0; CPR_id < CPRNum; CPR_id++)
	{
		double CPR_x = CPR[CPR_id][0];
		double CPR_y = CPR[CPR_id][1];
		int CPR_row = floor(CPR_y / cellSize);
		int CPR_col = floor(CPR_x / cellSize);
		// Find the grid effected by the CPR
		int CPR_gridSize = ceil(2 * OHZ_radius / cellSize) + 2; // Expand by 2 because of the case where node is outside CPR but its edge is blocked by the CPR
		int border = (CPR_gridSize - 1) / 2;
		for (int cell_row_local = - border; cell_row_local <= border; cell_row_local++) 
		{
			for (int cell_col_local = - border; cell_col_local <= border; cell_col_local++) 
			{
				int cell_row_global = CPR_row + cell_row_local;
				int cell_col_global = CPR_col + cell_col_local;
				if (cellInsideArea(cell_row_global,cell_col_global) == false) continue;
				vector<node*> nodeSet = tiling[cell_row_global][cell_col_global].nodeSet;
				int nodeNum = nodeSet.size();
				for (int node_id = 0; node_id < nodeNum; node_id++)
				{
					node* checkNode = nodeSet[node_id];
					if (checkNode->getTreeID() == invalidTreeID) continue;
					double checkNode_x = checkNode->getX();
					double checkNode_y = checkNode->getY();
					node* parent = checkNode->getParent();
					if (sqrt((CPR_x - checkNode_x) * (CPR_x - checkNode_x) + (CPR_y - checkNode_y) * (CPR_y - checkNode_y)) <= buffer) // Node is infeasible
					{
						checkNode->setDistToGoal(numeric_limits<double>::max()); // Set infinite distToGoal because it has been pruned
						checkNode->setTreeID(invalidTreeID); // Set invalid tree index
						// Update distToGoal, tree index, and connection for its child
						vector<node*> children = checkNode->getChildren();
						int childNum = children.size();
						for (int child_id = 0; child_id < childNum; child_id++)
						{
							node* child = children[child_id];
							child->setOldParent(checkNode); // Record this invalid parent for later addition after replanning step is done
							child->setParent(nullptr); // Break the connection
							child->setDistToGoal(numeric_limits<double>::max()); // Set infinite distToGoal because it is a disjoint tree
							newTreeID++;
							child->setTreeID(newTreeID); // Set a tree index that has not been used
							propagateNewInfoToSuccessor(child); // Propagate the distToGoal and tree index to its successors
							disjointTreeRoot.push_back(child); // It is a disjoint tree root
						}
						checkNode->clearChildren();

						// Add it to the pruned node set for later addition 
						prunedNode.push_back(checkNode);

						// Clear the incoming edge from the parent
						if (parent != nullptr)
						{
							parent->pruneChild(checkNode);
							checkNode->setOldParent(parent);
							checkNode->setParent(nullptr);
						}
					}
					else // Node is feasible, check the edge
					{
						if (parent != nullptr)
						{
							// Check the feasibility of the incoming edge from the parent
							double parent_x = parent->getX();
							double parent_y = parent->getY();
							double angle = atan2 (parent_y - checkNode_y, parent_x - checkNode_x);
							double dist = sqrt((checkNode_x - parent_x) * (checkNode_x - parent_x) + (checkNode_y - parent_y) * (checkNode_y - parent_y));
							int N = ceil(dist / collisionCheckingResolution);
							double sample_x, sample_y;
							double cos_angle = cos(angle);
							double sin_angle = sin(angle);
							for (int i = 0; i <= N; i++)
							{
								if (N > 0)
								{
									sample_x = checkNode_x + cos_angle * dist * i / N;
									sample_y = checkNode_y + sin_angle * dist * i / N;
								}
								else
								{
									sample_x = checkNode_x;
									sample_y = checkNode_y;
								}
								if (sqrt((CPR_x - sample_x) * (CPR_x - sample_x) + (CPR_y - sample_y) * (CPR_y - sample_y)) <= buffer)
								{
									// Clear the incoming edge from the parent
									parent->pruneChild(checkNode);
									checkNode->setOldParent(parent);
									checkNode->setParent(nullptr);
									// Set infinite distToGoal and new tree index and propagate to its successors
									checkNode->setDistToGoal(numeric_limits<double>::max());
									newTreeID++;
									checkNode->setTreeID(newTreeID);
									propagateNewInfoToSuccessor(checkNode);
									disjointTreeRoot.push_back(checkNode);
									break;
								}
							}
						}
					}
				}
			}
		}
	}
}

void SMART::propagateNewInfoToSuccessor(node* temp)
{
	vector<node*> children = temp->getChildren();
	double temp_x = temp->getX();
	double temp_y = temp->getY();
	int updatedTreeID = temp->getTreeID();
	int childNum = children.size();
	for (int child_id = 0; child_id < childNum; child_id++)
	{
		node* child = children[child_id];
		// Propagate distToGoal
		double distToGoal_temp = temp->getDistToGoal();
		if (distToGoal_temp == numeric_limits<double>::max()) // Propagate infinite distToGoal
		{
			if (child->getDistToGoal() != numeric_limits<double>::max()) child->setDistToGoal(numeric_limits<double>::max());
		}
		else // Propagate finite distToGoal
		{
			double child_x = child->getX();
			double child_y = child->getY();
			double newDistToGoal = distToGoal_temp + sqrt((child_x - temp_x) * (child_x - temp_x) + (child_y - temp_y) * (child_y - temp_y));
			child->setDistToGoal(newDistToGoal);
		}
		// Propagate updated tree index
		child->setTreeID(updatedTreeID);
		propagateNewInfoToSuccessor(child);
	}
}

bool SMART::replannedPathSearch()
{
	// Find the nearby reachable main tree node for the robot
	node* robot = new node(robotX, robotY);
	vector<node*> neighbor = getNearNode(robotX,robotY);
	vector<node*> validNeighbor;
	int neighborNum = neighbor.size();
	for (int node_id = 0; node_id < neighborNum; node_id++)
	{	
		node* checkNode = neighbor[node_id];
		if (checkNode->getTreeID() != mainTreeID) continue;
		if (lazyFeasibilityChecker_edge(robot, checkNode) == false || staticObsFeasibilityChecker_edge(robot, checkNode) == false) continue;
		validNeighbor.push_back(checkNode);
		double distToRobot = sqrt((robot->getX() - checkNode->getX()) * (robot->getX() - checkNode->getX()) + (robot->getY() - checkNode->getY()) * (robot->getY() - checkNode->getY()));
		checkNode->setDistToRobot(distToRobot);
	}
	struct 
	{
		bool operator()(node* a, node* b) const { return (a->getDistToGoal() + a->getDistToRobot() < b->getDistToGoal() + b->getDistToRobot()); }
	} LessSort;

	sort(validNeighbor.begin(), validNeighbor.end(), LessSort);
	delete robot;

	// Find the shortest path
	for (int node_id = 0; node_id < validNeighbor.size(); node_id++) 
	{
		bool pathFound = true;
		vector<node*>().swap(path);
		node* temp = validNeighbor[node_id];
		path.push_back(temp);
		while(temp->getParent() != nullptr) // Traverse the path until reaching the treeRoot(goal)
		{
			if (lazyFeasibilityChecker_edge(temp->getParent(), temp) == false || staticObsFeasibilityChecker_edge(temp->getParent(), temp) == false) 
			{
				pathFound = false;
				break;
			}
			temp = temp->getParent();
			path.push_back(temp);
		}
		if (pathFound == false) continue;
		reverse(path.begin(), path.end()); // First element is treeRoot while last element is the first waypoint after reversal
		return true;
	}
	return false;
}

bool SMART::treeRepair()
{
	clock_t start = clock();
	while (true)
	{
		if (hotSpotSearch() == true)
		{
			// Perform informed tree-repair
			treeReconnect();
		}
		else
		{
			// Perform standard tree-repair
			standardTreeRepair();
		}
		if (replannedPathSearch() == true)
		{
			return true;
		}
		else
		{
			double growTime = (double)(clock() - start) / CLOCKS_PER_SEC;
			if (growTime > timeDuration)
			{
				//std::cout<<"growTime = "<<growTime<<endl;
				//break;
				return false;
			}
		}
	}
}

bool SMART::hotSpotSearch()
{
	if (cellInsideArea(searchCenter.i, searchCenter.j) == false) 
	{
		std::cout<<"LSR center is outside workspace..."<<endl;
		return false;
	}
	double maxUtility = - numeric_limits<double>::max();
	int samplingCell_i, samplingCell_j;
	bool bestHotSpotFound = false;
	while (true)
	{
		int border = (LSR_size_current - 1) / 2;
		int border_not_use = border - 2; // Avoid redundant search in LSR
		// Identify the containing disjoint trees for each cell in the local search region
		for (int cell_row_local = - border; cell_row_local <= border; cell_row_local++) 
		{
			for (int cell_col_local = - border; cell_col_local <= border; cell_col_local++) 
			{
				int cell_row_global = searchCenter.i + cell_row_local;
				int cell_col_global = searchCenter.j + cell_col_local;
				if (cellInsideArea(cell_row_global,cell_col_global) == false) continue;
				if (LSR_size_current > 3) 
				{
					if (cell_row_local >= - border_not_use && cell_row_local <= border_not_use && cell_col_local >= - border_not_use && cell_col_local <= border_not_use)
					continue;
				}
				updateTreeID(cell_row_global, cell_col_global);
			}
		}
		// Identify hot-spot, compute utility, and determine the optimal hot-spot
		for (int cell_row_local = - border; cell_row_local <= border; cell_row_local++) 
		{
			for (int cell_col_local = - border; cell_col_local <= border; cell_col_local++) 
			{
				int cell_row_global = searchCenter.i + cell_row_local;
				int cell_col_global = searchCenter.j + cell_col_local;
				if (cellInsideArea(cell_row_global,cell_col_global) == false) continue;
				if (LSR_size_current > 3) 
				{
					if (cell_row_local >= - border_not_use && cell_row_local <= border_not_use && cell_col_local >= - border_not_use && cell_col_local <= border_not_use)
					continue;
				}
				if (isHotSpot(cell_row_global, cell_col_global) == true)
				{
					computeUtility(cell_row_global, cell_col_global);
					if (maxUtility < tiling[cell_row_global][cell_col_global].utility)
					{
						maxUtility = tiling[cell_row_global][cell_col_global].utility;
						samplingCell_i = cell_row_global;
						samplingCell_j = cell_col_global;
						bestHotSpotFound = true;
					}
				}
			}
		}
		// Terminate the search if best hot-spot is found
		if (bestHotSpotFound == true) 
		{
			bestHotSpot.i = samplingCell_i;
			bestHotSpot.j = samplingCell_j;
			bestHotSpot.x = double(samplingCell_j) * cellSize + cellSize / 2;
			bestHotSpot.y = double(samplingCell_i) * cellSize + cellSize / 2;
			return true;
		}
		else
		{
			// Expand LSR by 2 if maximum LSR size does not reach; otherwise, terminate the search
			LSR_size_current += 2;
			if (LSR_size_current > LSR_size_max) 
			{
				return false;
			}
		}
	}
}

bool SMART::cellInsideArea(int row, int col)
{
	if (row >= 0 && row < staticObsMapRow && col >= 0 && col < staticObsMapCol) return true;
	return false;
}

void SMART::updateTreeID(int row, int col)
{
	vector<int>().swap(tiling[row][col].treeID);
	vector<node*> nodeSet = tiling[row][col].nodeSet;
	int nodeNum = nodeSet.size();
	for (int node_id = 0; node_id < nodeNum; node_id++)
	{
		int newID = nodeSet[node_id]->getTreeID();
		if (newID == invalidTreeID) continue;
		if (std::find(tiling[row][col].treeID.begin(), tiling[row][col].treeID.end(), newID) == tiling[row][col].treeID.end()) 
		{
			tiling[row][col].treeID.push_back(newID);
		}
	}
}

bool SMART::isHotSpot(int row, int col)
{
	// Cell is occupied by static obstacle
	if (tiling[row][col].status == inStaticObs) return false;

	// Cell is occupied by CPR
	if (tiling[row][col].status == inCPR) return false;

	// Cell contains no node
	if(tiling[row][col].treeID.empty())
	{
		tiling[row][col].status = nonhotSpot;
		return false;
	}

	// Check if feasible pair of nodes exist
	vector<node*> nodeSet = tiling[row][col].nodeSet;
	int nodeNum = nodeSet.size();
	for (int nodeA_id = 0; nodeA_id < nodeNum; nodeA_id++)
	{
		node* nodeA = nodeSet[nodeA_id];
		int treeID_nodeA = nodeA->getTreeID();
		if (treeID_nodeA == invalidTreeID) continue; // Ignore pruned node
		// Check the nodes in the local neighborhood
		for (int local_i = - 1; local_i <= 1; local_i++) 
		{
			for (int local_j = - 1; local_j <= 1; local_j++) 
			{
				int cell_i = row + local_i;
				int cell_j = col + local_j;
				if (cellInsideArea(cell_i, cell_j) == false) continue;
				if (tiling[cell_i][cell_j].status == inStaticObs) continue;
				if (tiling[cell_i][cell_j].status == inCPR) continue;
				if (tiling[cell_i][cell_j].treeID.empty()) continue;
				if (cellInsideValidLSR(cell_i, cell_j) == false) continue; // Avoid reduandant search
				vector<node*> neighborSet = tiling[cell_i][cell_j].nodeSet;
				int neighborNum = neighborSet.size();
				for (int neighbor_id = 0; neighbor_id < neighborNum; neighbor_id++)
				{
					node* neighbor = neighborSet[neighbor_id];
					int treeID_neighbor = neighbor->getTreeID();
					if (treeID_neighbor == invalidTreeID) continue; // Ignore pruned node
					if (treeID_nodeA != treeID_neighbor)
					{
						if (lazyFeasibilityChecker_edge(nodeA, neighbor) == true && staticObsFeasibilityChecker_edge(nodeA, neighbor) == true)
						{
							tiling[row][col].status = hotSpot;
							return true;
						}
					}
				}
			}
		}
	}
	tiling[row][col].status = nonhotSpot;
	return false;
}

bool SMART::cellInsideValidLSR(int row, int col)
{
	int border = (LSR_size_current - 1) / 2;
	int border_not_use = border - 2;
	int cell_row_local = row - searchCenter.i;
	int cell_col_local = col - searchCenter.j;
	if (cell_row_local >= - border_not_use && cell_row_local <= border_not_use && cell_col_local >= - border_not_use && cell_col_local <= border_not_use)
		return false;
	return true;
}

bool SMART::cellInsideCPR(int row, int col)
{
	double cell_x = double(col) * cellSize + cellSize / 2;
	double cell_y = double(row) * cellSize + cellSize / 2;
	double buffer;
	if (robotInsideOHZ == true) buffer = obstacleRadius;
	else buffer = OHZ_radius;
	for (auto item : CPR)
	{
		double corner_x = cell_x - cellSize / 2;
		double corner_y = cell_y - cellSize / 2;
		if (sqrt((item[0] - corner_x) * (item[0] - corner_x) + (item[1] - corner_y) * (item[1] - corner_y)) > buffer) return false;
		corner_x = cell_x + cellSize / 2;
		corner_y = cell_y + cellSize / 2;
		if (sqrt((item[0] - corner_x) * (item[0] - corner_x) + (item[1] - corner_y) * (item[1] - corner_y)) > buffer) return false;
		corner_x = cell_x - cellSize / 2;
		corner_y = cell_y + cellSize / 2;
		if (sqrt((item[0] - corner_x) * (item[0] - corner_x) + (item[1] - corner_y) * (item[1] - corner_y)) > buffer) return false;
		corner_x = cell_x + cellSize / 2;
		corner_y = cell_y - cellSize / 2;
		if (sqrt((item[0] - corner_x) * (item[0] - corner_x) + (item[1] - corner_y) * (item[1] - corner_y)) > buffer) return false;
	}
	return true;
}

void SMART::computeUtility(int row, int col)
{
	tiling[row][col].utility = 0;
	vector<int> treeID = tiling[row][col].treeID;
	if (std::find(treeID.begin(), treeID.end(), mainTreeID) == treeID.end()) // Cell contains main tree
	{
		vector<node*> nodeSet = tiling[row][col].nodeSet;
		int nodeNum = nodeSet.size();
		for (int node_id = 0; node_id < nodeNum; node_id++)
		{
			if (nodeSet[node_id]->getTreeID() == mainTreeID)
			{
				tiling[row][col].utility = 1 / (tiling[row][col].distToRobot + nodeSet[node_id]->getDistToGoal()); 
				break;
			}
		}
	}
	else // Cell does not contain main tree
	{
		tiling[row][col].utility = 1 / (tiling[row][col].distToRobot + tiling[row][col].distToGoal); 
	}
}

void SMART::treeReconnect()
{
	vector<node*> hotSpotNodeSet = tiling[bestHotSpot.i][bestHotSpot.j].nodeSet; // Read the nodes in the selected hot spot
	int nodeNum = hotSpotNodeSet.size();
	for (int node_id = 0; node_id < nodeNum; node_id++)
	{
		node* nodeA = hotSpotNodeSet[node_id];
		if (nodeA->getTreeID() == invalidTreeID) continue; // Ignore pruned node
		// Read the neighboring nodes of nodeA
		vector<node*> neighborSet = nodeA->getNeighbors();
		vector<double> distToNeighbor = nodeA->getDistToNeighbor();
		int neighborNum = neighborSet.size();
		for (int neighbor_id = 0; neighbor_id < neighborNum; neighbor_id++)
		{
			node* nodeB = neighborSet[neighbor_id];
			double edgeLength = distToNeighbor[neighbor_id];
			if (nodeB->getTreeID() == invalidTreeID) continue; // Ignore pruned node
			if (tiling[nodeB->getRow()][nodeB->getCol()].status != hotSpot) continue; // Ignore non-hotspot cell
			if (nodeA->getTreeID() != nodeB->getTreeID()) // Only consider pair of nodes from different disjoint trees
			{
				if (lazyFeasibilityChecker_edge(nodeA, nodeB) == true) // Process if the edge is not obstructed by CPR
				{
					if (nodeA->getTreeID() == mainTreeID) // nodeA is main tree node
					{
						resetParent(nodeB, nodeA); // Set nodeA as parent and update parent-child for the nodeB's tree
						nodeB->setTreeID(nodeA->getTreeID());
						double newDistToGoal = nodeA->getDistToGoal() + edgeLength;
						nodeB->setDistToGoal(newDistToGoal);
						propagateNewInfoToSuccessor(nodeB); // Update cost-to-go and tree index for the nodeB's tree
						if (isOptimizationStartNodeValid(nodeB) == true) optimizationStartNode.push_back(nodeB);
					}
					else
					{
						if (nodeB->getTreeID() == mainTreeID) // nodeB is main tree node
						{
							resetParent(nodeA, nodeB); // Set nodeB as parent and update parent-child for the nodeA's tree
							nodeA->setTreeID(nodeB->getTreeID());
							double newDistToGoal = nodeB->getDistToGoal() + edgeLength;
							nodeA->setDistToGoal(newDistToGoal);
							propagateNewInfoToSuccessor(nodeA); // Update cost-to-go and tree index for the nodeA's tree
							if (isOptimizationStartNodeValid(nodeA) == true) optimizationStartNode.push_back(nodeA);
						}
						else // nodeB is not main tree node
						{
							resetParent(nodeB, nodeA); // Set nodeA as parent and update parent-child for the nodeB's tree
							nodeB->setTreeID(nodeA->getTreeID());
							propagateNewInfoToSuccessor(nodeB); // Update cost-to-go and tree index for the nodeB's tree
						}	
					}
				}
			}
		}
	}
}

void SMART::resetParent(node* childNode, node* newParent)
{
	node* oldParent = childNode->getParent();
	if (oldParent == nullptr)
	{
		childNode->setParent(newParent);
		newParent->setChildren(childNode);
	}
	else
	{
		oldParent->pruneChild(childNode);
		childNode->setParent(newParent);
		newParent->setChildren(childNode);
		resetParent(oldParent, childNode);
	}
}

bool SMART::isOptimizationStartNodeValid(node* temp)
{
	if (optimizationStartNode.empty()) return true;
	// Avoid repeated addition
	if (std::find(optimizationStartNode.begin(), optimizationStartNode.end(), temp) != optimizationStartNode.end()) return false; 
	// Ignore this node if it is the successor of existing optimization start node
	while (temp->getParent() != nullptr)
	{
		temp = temp->getParent();
		if (std::find(optimizationStartNode.begin(), optimizationStartNode.end(), temp) != optimizationStartNode.end()) return false;
	}
	return true;
}

void SMART::optimizationStartNodeProcess()
{
	// Remove pruned node
	for (vector<node*>::iterator it = optimizationStartNode.begin(); it != optimizationStartNode.end();)
	{
		if ((*it)->getTreeID() == invalidTreeID)
		{
			it = optimizationStartNode.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void SMART::treeOptimization()
{
	struct 
	{
		bool operator()(node* a, node* b) const { return (a->getDistToGoal() > b->getDistToGoal()); }
	} GreatSort;

	while (!optimizationStartNode.empty())
	{
		sort(optimizationStartNode.begin(), optimizationStartNode.end(), GreatSort);
		node* temp = optimizationStartNode.back();
		optimizationStartNode.pop_back();
		rewireCascade(temp);
	}
}

void SMART::rewireCascade(node* temp)
{
	findBestParent(temp); 
	vector<node*> children = temp->getChildren();
	int childNum = children.size();
	for (int child_id = 0; child_id < childNum; child_id++)
	{
		rewireCascade(children[child_id]);
	}
}

void SMART::findBestParent(node* checkNode)
{
	vector<node*> neighborSet = checkNode->getNeighbors();
	vector<double> distToNeighbor = checkNode->getDistToNeighbor();
	int neighborNum = neighborSet.size();
	int minID = -1;
	double minDistToGoal = checkNode->getDistToGoal();
	node* currentParent = checkNode->getParent();
	for (int neighbor_i = 0; neighbor_i < neighborNum; neighbor_i++) 
	{
		node* neighborNode = neighborSet[neighbor_i];
		if (neighborNode == currentParent) continue;
		if (neighborNode->getTreeID() != mainTreeID) continue; 
		double newDistToGoal = neighborNode->getDistToGoal() + distToNeighbor[neighbor_i]; 
		if (newDistToGoal < minDistToGoal)
		{
			if (lazyFeasibilityChecker_edge(neighborNode, checkNode) == true)
			{
				minDistToGoal = newDistToGoal;
				minID = neighbor_i;
			}
		}
	}
	if (minID != -1)
	{
		if (currentParent != nullptr) currentParent->pruneChild(checkNode);
		checkNode->setParent(neighborSet[minID]);
		checkNode->setDistToGoal(minDistToGoal);
		neighborSet[minID]->setChildren(checkNode);
		propagateNewDistToGoalToSuccessor(checkNode);
	}
}

void SMART::propagateNewDistToGoalToSuccessor(node* temp)
{
	vector<node*> children = temp->getChildren();
	double temp_x = temp->getX();
	double temp_y = temp->getY();
	double distToGoal_temp = temp->getDistToGoal();
	int childNum = children.size();
	for (int child_id = 0; child_id < childNum; child_id++)
	{
		node* child = children[child_id];
		double child_x = child->getX();
		double child_y = child->getY();
		double newDistToGoal = distToGoal_temp + sqrt((child_x - temp_x) * (child_x - temp_x) + (child_y - temp_y) * (child_y - temp_y));
		child->setDistToGoal(newDistToGoal);
		propagateNewDistToGoalToSuccessor(child);
	}
}

void SMART::formSingleTree()
{
	// Add pruned node back to either disjoint tree or main tree
	int prunedNodeNum = prunedNode.size();
	for (int node_id = 0; node_id < prunedNodeNum; node_id++)
	{
		node* checkNode = prunedNode[node_id];
		node* oldParent = checkNode->getOldParent();
		if (oldParent != nullptr)
		{
			checkNode->setParent(oldParent);
			oldParent->setChildren(checkNode);
			int treeID_parent = oldParent->getTreeID();
			if (treeID_parent != invalidTreeID) // Parent is not pruned node
			{
				if (treeID_parent == mainTreeID) // Parent is on maintree
				{
					checkNode->setTreeID(treeID_parent);
					double newDistToGoal = oldParent->getDistToGoal() + sqrt((checkNode->getX() - oldParent->getX()) * (checkNode->getX() - oldParent->getX()) + (checkNode->getY() - oldParent->getY()) * (checkNode->getY() - oldParent->getY()));
					checkNode->setDistToGoal(newDistToGoal);
					propagateNewInfoToSuccessor(checkNode);
				}
				else // Parent is on other disjoint trees
				{
					checkNode->setTreeID(treeID_parent);
					propagateNewInfoToSuccessor(checkNode);
				}
			}
			checkNode->setOldParent(nullptr);
		}
	}
	vector<node*>().swap(prunedNode);
	// Add remaining disjoint tree root to its previous parent
	updateDisjointTreeRoot();
	for (vector<node*>::iterator it = disjointTreeRoot.begin(); it != disjointTreeRoot.end();)
	{
		node* checkNode = *it;
		node* oldParent = checkNode->getOldParent();
		checkNode->setOldParent(nullptr);
		if (checkNode->getTreeID() != oldParent->getTreeID())
		{
			checkNode->setParent(oldParent);
			oldParent->setChildren(checkNode);
			int treeID_parent = oldParent->getTreeID();
			if (treeID_parent == mainTreeID) // Parent is on maintree
			{
				checkNode->setTreeID(treeID_parent);
				double newDistToGoal = oldParent->getDistToGoal() + sqrt((checkNode->getX() - oldParent->getX()) * (checkNode->getX() - oldParent->getX()) + (checkNode->getY() - oldParent->getY()) * (checkNode->getY() - oldParent->getY()));
				checkNode->setDistToGoal(newDistToGoal);
				propagateNewInfoToSuccessor(checkNode);
			}
			else // Parent is on other disjoint trees
			{
				checkNode->setTreeID(treeID_parent);
				propagateNewInfoToSuccessor(checkNode);
			}
			it = disjointTreeRoot.erase(it);
		}
		else
		{
			++it;
		}
	}
}

void SMART::updateDisjointTreeRoot()
{
	// Remove the disjoint tree root which becomes infeasible or connected to another subtree
	vector<node*> disjointTreeRoot_raw = disjointTreeRoot;
	vector<node*>().swap(disjointTreeRoot);
	int disjointTreeRootNum = disjointTreeRoot_raw.size();
	for (int node_id = 0; node_id < disjointTreeRootNum; node_id++)
	{
		node* checkNode = disjointTreeRoot_raw[node_id];
		node* oldParent = checkNode->getOldParent();
		// Ignore pruned node
		if (checkNode->getTreeID() != invalidTreeID) 
		{
			// Ignore the disjoint tree which has been reconnected to another tree
			if (checkNode->getParent() == nullptr) 
			{
				// Avoid repeated addition
				if (std::find(disjointTreeRoot.begin(), disjointTreeRoot.end(), checkNode) == disjointTreeRoot.end()) disjointTreeRoot.push_back(checkNode);
			}
		}
	}
}

void SMART::standardTreeRepair()
{
	bool addDone = false;
	node* nodeA = sampleFree();
	vector<node*> neighborSet = getNearNode(nodeA->getX(),nodeA->getY());
	int neighborNum = neighborSet.size();
	for (int neighbor_id = 0; neighbor_id < neighborNum; neighbor_id++)
	{
		node* nodeB = neighborSet[neighbor_id];
		int treeID_neighbor = nodeB->getTreeID();
		if (treeID_neighbor == invalidTreeID) continue; // Ignore pruned node
		if (nodeA->getTreeID() != treeID_neighbor)
		{
			if (lazyFeasibilityChecker_edge(nodeA, nodeB) == true && staticObsFeasibilityChecker_edge(nodeA, nodeB) == true)
			{
				if (addDone == false) addDone = true;
				if (nodeA->getTreeID() == mainTreeID) // nodeA is main tree node
				{
					resetParent(nodeB, nodeA); // Set nodeA as parent and update parent-child for the nodeB's tree
					nodeB->setTreeID(nodeA->getTreeID());
					double newDistToGoal = nodeA->getDistToGoal() + sqrt((nodeA->getX() - nodeB->getX()) * (nodeA->getX() - nodeB->getX()) + (nodeA->getY() - nodeB->getY()) * (nodeA->getY() - nodeB->getY()));
					nodeB->setDistToGoal(newDistToGoal);
					propagateNewInfoToSuccessor(nodeB); // Update cost-to-go and tree index for the nodeB's tree
					if (isOptimizationStartNodeValid(nodeB) == true) optimizationStartNode.push_back(nodeB);
				}
				else
				{
					if (nodeB->getTreeID() == mainTreeID) // nodeB is main tree node
					{
						resetParent(nodeA, nodeB); // Set nodeB as parent and update parent-child for the nodeA's tree
						nodeA->setTreeID(nodeB->getTreeID());
						double newDistToGoal = nodeB->getDistToGoal() + sqrt((nodeA->getX() - nodeB->getX()) * (nodeA->getX() - nodeB->getX()) + (nodeA->getY() - nodeB->getY()) * (nodeA->getY() - nodeB->getY()));
						nodeA->setDistToGoal(newDistToGoal);
						propagateNewInfoToSuccessor(nodeA); // Update cost-to-go and tree index for the nodeA's tree
						if (isOptimizationStartNodeValid(nodeA) == true) optimizationStartNode.push_back(nodeA);
					}
					else // nodeB is not main tree node
					{
						resetParent(nodeA, nodeB); // Set nodeB as parent and update parent-child for the nodeA's tree
						nodeA->setTreeID(nodeB->getTreeID());
						propagateNewInfoToSuccessor(nodeA); // Update cost-to-go and tree index for the nodeA's tree
					}	
				}
			}
		}
	}
	if (addDone == false)
	{
		delete nodeA;
	}
	else
	{
		tiling[nodeA->getRow()][nodeA->getCol()].nodeSet.push_back(nodeA);
		// Check 3 by 3 neighborhood of new node's cell and update neighboring node information
		int node_i = nodeA->getRow();
		int node_j = nodeA->getCol();
		for (int local_i = -1; local_i <= 1; local_i++)
		{
			for (int local_j = -1; local_j <= 1; local_j++) 
			{
				int cell_i = node_i + local_i;
				int cell_j = node_j + local_j;
				if (cellInsideArea(cell_i, cell_j) == true)
				{
					vector<node*> neighborNodeSet = tiling[cell_i][cell_j].nodeSet;
					int neighborNodeNum = neighborNodeSet.size();
					for (int neighbor_i = 0; neighbor_i < neighborNodeNum; neighbor_i++)
					{
						node* neighborNode = neighborNodeSet[neighbor_i];
						if (staticObsFeasibilityChecker_edge(neighborNode, nodeA) == true)
						{
							double dist = sqrt((neighborNode->getX() - nodeA->getX()) * (neighborNode->getX() - nodeA->getX()) + (neighborNode->getY() - nodeA->getY()) * (neighborNode->getY() - nodeA->getY()));
							nodeA->setNeighbors(neighborNode);
							nodeA->setDistToNeighbor(dist);
							neighborNode->setNeighbors(nodeA);
							neighborNode->setDistToNeighbor(dist);
						}
					}
				}
			}
		}
	}
}

node* SMART::sampleFree()
{
	node* newNode;
	while (true)
	{
		int cell_index = freeCellDis(freeCellGen);
		double samplingCellX = (double) staticObsFreeCell[cell_index][1] * cellSize + cellSize / 2;
		double samplingCellY = (double) staticObsFreeCell[cell_index][0] * cellSize + cellSize / 2;
		double randX = localSampleDis(localSampleGen) + samplingCellX;
		double randY = localSampleDis(localSampleGen) + samplingCellY;
		newNode = new node(randX, randY);
		double dist = sqrt((randX - robotX) * (randX - robotX) + (randY - robotY) + (randY - robotY));
		if (lazyFeasibilityChecker_node(newNode) == true)
		{
			return newNode;
		}
		else
		{
			delete newNode;
		}
	}
}

vector<node*> SMART::getNearNode(double x, double y)
{
	int node_i = floor(y / cellSize);
	int node_j = floor(x / cellSize);
	vector<node*> neighborNodeList;
	// Search the neighboring cell for the random node sitting cell 
	for (int local_i = -1; local_i <= 1; local_i++) 
	{
		for (int local_j = -1; local_j <= 1; local_j++) 
		{
			int cell_i = node_i + local_i;
			int cell_j = node_j + local_j;
			if (cellInsideArea(cell_i, cell_j) == true)
			{
				vector<node*> neighborNodeSet = tiling[cell_i][cell_j].nodeSet;
				int neighborNodeNum = neighborNodeSet.size();
				for (int neighbor_i = 0; neighbor_i < neighborNodeNum; neighbor_i++)
				{
					neighborNodeList.push_back(neighborNodeSet[neighbor_i]);
				}
			}
		}
	}
	return neighborNodeList;
}

bool SMART::lazyFeasibilityChecker_edge(node* nodeA, node* nodeB)
{
	double buffer;
	if (robotInsideOHZ == true) buffer = obstacleRadius;
	else buffer = OHZ_radius;
	double alp = atan2 (nodeB->getY() - nodeA->getY(), nodeB->getX() - nodeA->getX());
	double dist = sqrt((nodeA->getX() - nodeB->getX()) * (nodeA->getX() - nodeB->getX()) + (nodeA->getY() - nodeB->getY()) * (nodeA->getY() - nodeB->getY()));
	int N = ceil(dist / collisionCheckingResolution);
	double x, y;
	for (int i = 0; i <= N; i++)
	{
		if (N > 0)
		{
			x = nodeA->getX() + cos(alp) * dist * i / N;
			y = nodeA->getY() + sin(alp) * dist * i / N;
		}
		else
		{
			x = nodeA->getX();
			y = nodeA->getY();
		}
		double distance = sqrt((robotX - x) * (robotX - x) + (robotY - y) * (robotY - y));
		if (distance > LRZ_radius + OHZ_radius + OHZ_radius) continue;
		for (auto item : CPR)
			if (sqrt((item[0] - x) * (item[0] - x) + (item[1] - y) * (item[1] - y)) <= buffer)
				return false;
	}
	return true;  
}

bool SMART::lazyFeasibilityChecker_node(node* nodeA)
{
	double buffer;
	if (robotInsideOHZ == true) buffer = obstacleRadius;
	else buffer = OHZ_radius;
	double x = nodeA->getX();
	double y = nodeA->getY();
	double distance = sqrt((robotX - x) * (robotX - x) + (robotY - y) * (robotY - y));
	if (distance > LRZ_radius + OHZ_radius + OHZ_radius) return true;
	for (auto item : CPR)
		if (sqrt((item[0] - x) * (item[0] - x) + (item[1] - y) * (item[1] - y)) <= buffer)
			return false;
	return true;  
}

bool SMART::staticObsFeasibilityChecker_edge(node* nodeA, node* nodeB)
{
	double alp = atan2 (nodeB->getY() - nodeA->getY(), nodeB->getX() - nodeA->getX());
	double dist = sqrt((nodeA->getX() - nodeB->getX()) * (nodeA->getX() - nodeB->getX()) + (nodeA->getY() - nodeB->getY()) * (nodeA->getY() - nodeB->getY()));
	int N = ceil(dist / collisionCheckingResolution);
	int row_prev = numeric_limits<int>::max();
	int col_prev = numeric_limits<int>::max();
	double x, y;
	for (int i = 0; i <= N; i++)
	{
		if (N > 0)
		{
			x = nodeA->getX() + cos(alp) * dist * i / N;
			y = nodeA->getY() + sin(alp) * dist * i / N;
		}
		else
		{
			x = nodeA->getX();
			y = nodeA->getY();
		}
		int row =  floor(y / cellSize);
		int col = floor(x / cellSize);

		if (row == row_prev && col == col_prev) continue;
		if (tiling[row][col].status == inStaticObs) return false;
		row_prev = row;
		col_prev = col;
	}
	return true;  
}

bool SMART::reachWaypoint()
{
	double waypoint_x = wayPointNode->getX();
	double waypoint_y = wayPointNode->getY();
	double dist = sqrt((waypoint_x - robotX) * (waypoint_x - robotX) + (waypoint_y - robotY) * (waypoint_y - robotY));
	if (dist < reachThreshold)
	{
		//std::cout<<"reachwaypoint..."<<endl;
		path.pop_back();
		robotX = waypoint_x;
		robotY = waypoint_y;
		if (!path.empty()) wayPointNode = path.back();
		return true;
	}
	return false;
}

void SMART::dataRecord()
{
	//std::cout<<"iterationIndex = "<<iterationIndex<<endl;
	dynObsInfoRecord();
	treeInfoRecord();
	pathInfoRecord();
	footPrintInfoRecord();
	iterationIndex++;
}

void SMART::dynObsInfoRecord()
{
	vector<double> pointRow;
	pointRow.push_back(iterationIndex);
 	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	dynObsInfo.push_back(pointRow);
	for (int obs_id = 0; obs_id < dynObsNum; obs_id++)
	{
		double CPRTrue = 0;
		for (int CPR_id = 0; CPR_id < CPR.size(); CPR_id++)
		{
			if (CPR[CPR_id][2] == obs_id)
			{
				CPRTrue = 1;
				break;
			}
		}
		dynObsInfo.push_back({obsPosition[obs_id][0],obsPosition[obs_id][1],obsPosition[obs_id][2],CPRTrue});
	}
}

void SMART::treeInfoRecord()
{
 	// Record the iteration index
 	vector<double> pointRow;
 	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	treeInfo.push_back(pointRow);
	// Record the tree information
	for (int row = 0; row < staticObsMapRow; row++) 
	{
		for (int col = 0; col < staticObsMapCol; col++) 
		{
			vector<node*> nodeSet = tiling[row][col].nodeSet;
			int nodeNum = nodeSet.size();
			for (int node_id = 0; node_id < nodeNum; node_id++)
			{
				node* checkNode = nodeSet[node_id];
				vector<double> pointRow;
				pointRow.push_back(checkNode->getX());
				pointRow.push_back(checkNode->getY());
				if (checkNode->getParent() != nullptr)
				{
					pointRow.push_back(checkNode->getParent()->getX());
					pointRow.push_back(checkNode->getParent()->getY());
				}
				else
				{
					pointRow.push_back(checkNode->getX());
					pointRow.push_back(checkNode->getY());
				}
				pointRow.push_back(checkNode->getDistToGoal());
				pointRow.push_back(checkNode->getTreeID());
				treeInfo.push_back(pointRow);
			}
		}
	}
}
void SMART::pathInfoRecord()
{
	// Record the iteration index
	vector<double> pointRow;
	pointRow.push_back(iterationIndex);
	pointRow.push_back(iterationIndex);
	pathInfo.push_back(pointRow);
	for (vector<node*>::iterator it = path.begin(); it != path.end();)
	{
		pathInfo.push_back({(*it)->getX(), (*it)->getY()});
		++it;
	}
}

void SMART::footPrintInfoRecord()
{
	footPrintInfo.push_back({robotX, robotY});
}

void SMART::trajectoryLengthCompute()
{
	double trajectoryLength = 0;
	if (!footPrintInfo.empty())
	{
		for (int i = 0; i < footPrintInfo.size() - 1; i++)
		{
			trajectoryLength += sqrt((footPrintInfo[i][0] - footPrintInfo[i+1][0]) * (footPrintInfo[i][0] - footPrintInfo[i+1][0]) + (footPrintInfo[i][1] - footPrintInfo[i+1][1]) * (footPrintInfo[i][1] - footPrintInfo[i+1][1]));
		}
	}
	std::cout<<"path length = "<<trajectoryLength<<endl;
	std::cout<<"path traversal time = "<<trajectoryLength / robotSpeed<<endl;
	totalTravelTime = totalReplanTime + trajectoryLength / robotSpeed;
	std::cout<<"travel time = "<<totalTravelTime<<endl;
}