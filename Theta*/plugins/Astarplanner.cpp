#include "Astarplanner.h"
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_plugin::AStarPlanner, nav_core::BaseGlobalPlanner)

int value;
int mapSize;
bool *occupancyGridMap;

//cost of non connected nodes
float infinity = std::numeric_limits<float>::infinity();

// coefficient for breaking ties
float tBreak;

namespace astar_plugin
{

//Default Constructor
AStarPlanner::AStarPlanner()
{
}

/**
  Constructor with shared node handle
**/
AStarPlanner::AStarPlanner(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;
}

/**
  Constructor that initilizes costmap and other parameters
**/
AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros);
}

/**
Implementation of method from BaseGlobalPlanner interface that
initializes the cost map and other parameters of the grid.

**/

void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{

  if (!initialized_)
  {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();

    width = costmap_->getSizeInCellsX();
    height = costmap_->getSizeInCellsY();
    resolution = costmap_->getResolution();
    mapSize = width * height;
    tBreak = 1 + 1 / (mapSize);
    value = 0;

    occupancyGridMap = new bool[mapSize];
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

        if (cost == 0)
          occupancyGridMap[iy * width + ix] = true;
        else
          occupancyGridMap[iy * width + ix] = false;
      }
    }

    ROS_INFO("Theta planner initialized.");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

/**
  Implementation of method from BaseGlobalPlanner interface that calculates
  plan to reach the goal
**/
bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                            std::vector<geometry_msgs::PoseStamped> &plan)
{

  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;

  //Convert to map coordinates relative to costmap origin
  convertToMapCoordinates(startX, startY);
  convertToMapCoordinates(goalX, goalY);

  int startGridSquare;
  int goalGridSquare;

  if (isCoordinateInBounds(startX, startY) && isCoordinateInBounds(goalX, goalY))
  {
    startGridSquare = getGridSquareIndex(startX, startY);

    goalGridSquare = getGridSquareIndex(goalX, goalY);
  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }

  // call global planner

  if (isStartAndGoalValid(startGridSquare, goalGridSquare))
  {

    vector<int> bestPath;
    bestPath.clear();

    bestPath = runAStarOnGrid(startGridSquare, goalGridSquare);

    //if the global planner finds a path
    if (bestPath.size() > 0)
    {

      // convert the path

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        float previous_x = 0.0;
        float previous_y = 0.0;

        int index = bestPath[i];
        int previous_index;
        getGridSquareCoordinates(index, x, y);

        if (i != 0)
        {
          previous_index = bestPath[i - 1];
        }
        else
        {
          previous_index = index;
        }

        getGridSquareCoordinates(previous_index, previous_x, previous_y);

        //Orient the bot towards target
        tf::Vector3 vectorToTarget;
        vectorToTarget.setValue(x - previous_x, y - previous_y, 0.0);
        float angle = atan2((double)vectorToTarget.y(), (double)vectorToTarget.x());

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

        plan.push_back(pose);
      }

      return true;
    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }
  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }
}

/**
  Function to adjust start and goal w.r.t origin point on map
**/
void AStarPlanner::convertToMapCoordinates(float &x, float &y)
{

  x = x - originX;
  y = y - originY;
}

/**
  Function to get index of grid square on map given square coordinates
**/
int AStarPlanner::getGridSquareIndex(float x, float y)
{

  int gridSquare;

  float newX = x / (resolution);
  float newY = y / (resolution);

  gridSquare = calculateGridSquareIndex(newY, newX);

  return gridSquare;
}

/**
  Function to get gridSquare coordinates given index
  
**/
void AStarPlanner::getGridSquareCoordinates(int index, float &x, float &y)
{

  x = getGridSquareColIndex(index) * resolution;

  y = getGridSquareRowIndex(index) * resolution;

  x = x + originX;
  y = y + originY;
}

/**
  Function to check if gridSquare coordinates are in map bounds
  
**/
bool AStarPlanner::isCoordinateInBounds(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}

/**
  Function runs the A* algorithm to find best path to goal on grid

**/
vector<int> AStarPlanner::runAStarOnGrid(int startGridSquare, int goalGridSquare)
{

  vector<int> bestPath;

  float g_score[mapSize];

  for (uint i = 0; i < mapSize; i++)
    g_score[i] = infinity;

  bestPath = findPath(startGridSquare, goalGridSquare, g_score);

  return bestPath;
}
bool AStarPlanner::lineOfSight(int index1, int index2) {
    int x0 = getGridSquareColIndex(index1);
    int y0 = getGridSquareRowIndex(index1);
    int x1 = getGridSquareColIndex(index2);
    int y1 = getGridSquareRowIndex(index2);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int e2;

    while (true) {
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
        if (!isFree(y0 * width + x0)) return false;
    }
    return true;
}
vector<int> AStarPlanner::reconstructPath(map<int, int>& parent, int start, int goal) {
    vector<int> path;
    for (int current = goal; current != start; current = parent[current]) {
        path.push_back(current);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

/**

  Generates the path for the bot towards the goal

**/
vector<int> AStarPlanner::findPath(int startGridSquare, int goalGridSquare, float g_score[]) {
    value++;
    vector<int> bestPath;
    multiset<GridSquare> openList;
    map<int, int> parent;
    map<int, float> f_score;

    parent[startGridSquare] = startGridSquare;
    g_score[startGridSquare] = 0;
    f_score[startGridSquare] = calculateHScore(startGridSquare, goalGridSquare);

    openList.insert({startGridSquare, f_score[startGridSquare]});

    while (!openList.empty()) {
        int current = openList.begin()->currentGridSquare;
        openList.erase(openList.begin());

        if (current == goalGridSquare) {
            return reconstructPath(parent, startGridSquare, goalGridSquare);
        }

        vector<int> neighbors = findFreeNeighborGridSquare(current);
        for (int neighbor : neighbors) {
            float tentative_g_score = g_score[current] + getMoveCost(current, neighbor);
            if (tentative_g_score < g_score[neighbor]) {
                if (lineOfSight(parent[current], neighbor)) {
                    parent[neighbor] = parent[current];
                } else {
                    parent[neighbor] = current;
                }
                g_score[neighbor] = tentative_g_score;
                f_score[neighbor] = g_score[neighbor] + calculateHScore(neighbor, goalGridSquare);
                openList.insert({neighbor, f_score[neighbor]});
            }
        }
    }
    return vector<int>(); // return empty path if none found
}


/**
  Function constructs the path found by findPath function by returning vector of
  gridSquare indices that lie on path.

**/
vector<int> AStarPlanner::constructPath(int startGridSquare, int goalGridSquare, float g_score[])
{
  vector<int> bestPath;
  vector<int> path;

  path.insert(path.begin() + bestPath.size(), goalGridSquare);
  int currentGridSquare = goalGridSquare;

  while (currentGridSquare != startGridSquare)
  {
    vector<int> neighborGridSquares;
    neighborGridSquares = findFreeNeighborGridSquare(currentGridSquare);

    vector<float> gScoresNeighbors;
    for (uint i = 0; i < neighborGridSquares.size(); i++)
      gScoresNeighbors.push_back(g_score[neighborGridSquares[i]]);

    int posMinGScore = distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
    currentGridSquare = neighborGridSquares[posMinGScore];

    //insert the neighbor in the path
    path.insert(path.begin() + path.size(), currentGridSquare);
  }
  for (uint i = 0; i < path.size(); i++)
    bestPath.insert(bestPath.begin() + bestPath.size(), path[path.size() - (i + 1)]);

  return bestPath;
}

/**
  Helper function to add unexplored neighbours of currentGridSquare to openlist
**/
void AStarPlanner::addNeighborGridSquareToOpenList(multiset<GridSquare> &openSquaresList, int neighborGridSquare, int goalGridSquare, float g_score[])
{
  GridSquare gridSq;
  gridSq.currentGridSquare = neighborGridSquare; //insert the neighborGridSquare
  gridSq.fCost = g_score[neighborGridSquare] + calculateHScore(neighborGridSquare, goalGridSquare);
  openSquaresList.insert(gridSq);
}

/**
  Helper function to find free neighbours of currentGridSquare 
**/

vector<int> AStarPlanner::findFreeNeighborGridSquare(int gridSquare)
{

  int rowIndex = getGridSquareRowIndex(gridSquare);
  int colIndex = getGridSquareColIndex(gridSquare);
  int neighborIndex;
  vector<int> freeNeighborGridSquares;

  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
    {
      //check whether the index is valid
      if ((rowIndex + i >= 0) && (rowIndex + i < height) && (colIndex + j >= 0) && (colIndex + j < width) && (!(i == 0 && j == 0)))
      {
        neighborIndex = ((rowIndex + i) * width) + (colIndex + j);

        if (isFree(neighborIndex))
          freeNeighborGridSquares.push_back(neighborIndex);
      }
    }
  return freeNeighborGridSquares;
}

/**
  Checks if start and goal positions are valid and not unreachable.
**/
bool AStarPlanner::isStartAndGoalValid(int startGridSquare, int goalGridSquare)
{
  bool isvalid = true;
  bool isFreeStartGridSquare = isFree(startGridSquare);
  bool isFreeGoalGridSquare = isFree(goalGridSquare);
  if (startGridSquare == goalGridSquare)
  {

    isvalid = false;
  }
  else
  {
    if (!isFreeStartGridSquare && !isFreeGoalGridSquare)
    {

      isvalid = false;
    }
    else
    {
      if (!isFreeStartGridSquare)
      {

        isvalid = false;
      }
      else
      {
        if (!isFreeGoalGridSquare)
        {

          isvalid = false;
        }
        else
        {
          if (findFreeNeighborGridSquare(goalGridSquare).size() == 0)
          {

            isvalid = false;
          }
          else
          {
            if (findFreeNeighborGridSquare(startGridSquare).size() == 0)
            {

              isvalid = false;
            }
          }
        }
      }
    }
  }
  return isvalid;
}

/**
  Helper function to calculate cost of moving from currentGridSquare to neighbour

**/
float AStarPlanner::getMoveCost(int i1, int j1, int i2, int j2)
{
  float moveCost = infinity; //start cost with maximum value. Change it to real cost of gridSquares are connected
  //if gridSquare(i2,j2) exists in the diagonal of gridSquare(i1,j1)
  if ((j2 == j1 + 1 && i2 == i1 + 1) || (i2 == i1 - 1 && j2 == j1 + 1) || (i2 == i1 - 1 && j2 == j1 - 1) || (j2 == j1 - 1 && i2 == i1 + 1))
  {

    moveCost = 1.4;
  }
  //if gridSquare(i2,j2) exists in the horizontal or vertical line with gridSquare(i1,j1)
  else
  {
    if ((j2 == j1 && i2 == i1 - 1) || (i2 == i1 && j2 == j1 - 1) || (i2 == i1 + 1 && j2 == j1) || (i1 == i2 && j2 == j1 + 1))
    {

      moveCost = 1;
    }
  }
  return moveCost;
}

/**
  Wrapper function to calculate cost of moving from currentGridSquare to neighbour

**/
float AStarPlanner::getMoveCost(int gridSquareIndex1, int gridSquareIndex2)
{
  int i1 = 0, i2 = 0, j1 = 0, j2 = 0;

  i1 = getGridSquareRowIndex(gridSquareIndex1);
  j1 = getGridSquareColIndex(gridSquareIndex1);
  i2 = getGridSquareRowIndex(gridSquareIndex2);
  j2 = getGridSquareColIndex(gridSquareIndex2);

  return getMoveCost(i1, j1, i2, j2);
}

/**

**/
float AStarPlanner::calculateHScore(int gridSquareIndex, int goalGridSquare)
{
  int x1 = getGridSquareRowIndex(goalGridSquare);
  int y1 = getGridSquareColIndex(goalGridSquare);
  int x2 = getGridSquareRowIndex(gridSquareIndex);
  int y2 = getGridSquareColIndex(gridSquareIndex);
  return abs(x1 - x2) + abs(y1 - y2);
}

/**
  Calculates the gridSquare index from square coordinates
**/
int AStarPlanner::calculateGridSquareIndex(int i, int j) 
{
  return (i * width) + j;
}

/**

  Calculates gridSquare row from square index

**/
int AStarPlanner::getGridSquareRowIndex(int index) //get the row index from gridSquare index
{
  return index / width;
}

/**

  Calculates gridSquare column from square index

**/
int AStarPlanner::getGridSquareColIndex(int index) //get column index from gridSquare index
{
  return index % width;
}

/**

  Checks if gridSquare at (i,j) is free

**/
bool AStarPlanner::isFree(int i, int j)
{
  int gridSquareIndex = (i * width) + j;

  return occupancyGridMap[gridSquareIndex];
}

/**

  Checks if gridSquare at index gridSquareIndex is free

**/
bool AStarPlanner::isFree(int gridSquareIndex)
{
  return occupancyGridMap[gridSquareIndex];
}
};

/**
  Overloaded operator for comparing cost among two gridSquares.

**/
bool operator<(GridSquare const &c1, GridSquare const &c2) { return c1.fCost < c2.fCost; }