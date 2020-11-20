// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#include <unistd.h>
#include <vector>
#include "v_repExtAMRProject.h"
#include "v_repLib.h"
#include <iostream>
#include <map>
#include <list>
#include <ctime>
#include <fenv.h>
#include <time.h>
#include <fstream>
#include <chrono>
#include <pthread.h> 
#include <thread>
#include <Eigen/Core>
#include <Eigen/Dense>

#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

pthread_t planning_thread;
pthread_t waiting_thread;

int RRT = 2;  // 0 for RRT, 1 for RRT*, 2 for Anytime RRT*
float pi = 3.1415926535;
std::vector<Eigen::Vector3f> path;
int i_path;
float rotationAlpha;
float rotationGamma;
float height;
bool solutionFound;
bool solutionUpdated;
int start;
int stop;
int rootIndex = 0;
int previousRootIndex = 0;
float alphaCoefficient = 0;
bool pathInterpolation = false;
bool pathInterpolation2 = false;
bool pathOperation = false;
bool goalReached = false;
simInt nullSimInt;
Eigen::Vector3f nullPoint;
int indexPathPlotted = 0;
bool restart=false;
int numOfStep=3;

std::vector<int> pointsToBePlotted;
std::vector<int> pointsToBePruned;
std::vector<int> rewiredLinksToBePlotted;
std::vector<int> rewiringIndexesPoint;

//while a solution isn't find, run the algorithm
bool RRT_IsRunning=false;

bool timeBudgetReached= false;
//map size
float MAP_WIDTH = 5;
float MAP_HEIGHT = 5;

//vectors for points, and parent of the TREE
std::vector<Eigen::Vector3f> TREEpoints;
std::vector<int> TREEparentsIndexes;
std::vector<std::vector<int>> TREEchildrenIndexes;
std::vector<float> TREEdistancesFromParent;

//vector for points of the PATH
std::vector<Eigen::Vector3f> PATHpoints;
std::vector<int> PATHpointsIndexes;

std::vector<Eigen::Vector3f> PATHpointsNew;
std::vector<int> PATHpointsIndexesNew;

std::vector<std::vector<Eigen::Vector3f>> PATHpointsList;
std::vector<std::vector<int>> PATHpointsIndexesList;
std::vector<float> PATHcosts;
std::vector<float> PATHtimes;
float finalCost = 0;
float previousCost = 0;

std::vector<simInt> objects;
std::vector<simInt> plottedPathPoints;
std::vector<simInt> plottedPathLinks;
std::vector<simInt> plottedTreePoints;
std::vector<simInt> plottedTreeLinks;

//root point
Eigen::Vector3f ROOT;

//goal point
Eigen::Vector3f GOAL;
int GOALindex;		//index of the goal

void removeTreePlots()
{
	for(int i=0; i<plottedTreePoints.size(); i++)
	{
		simSetModelProperty(plottedTreePoints[i],sim_modelproperty_not_visible);
		simSetModelProperty(plottedTreeLinks[i],sim_modelproperty_not_visible);
	}
}
void prunePointsAndLinks()
{
	if(pointsToBePruned.size()>0)
	{
		int treshold = pointsToBePruned.size();
		for(int i=0; i<treshold; i++)
		{

			if(pointsToBePruned[i]<plottedTreePoints.size())
			{
				simSetModelProperty(plottedTreePoints[pointsToBePruned[i]],sim_modelproperty_not_visible);
				simSetModelProperty(plottedTreeLinks[pointsToBePruned[i]],sim_modelproperty_not_visible);
			}
		}
		pointsToBePruned.erase(pointsToBePruned.begin(),pointsToBePruned.begin()+treshold);
	}
}
void plotPointsAndLinks()
{
	
	//Sizes of nodes and links
	simFloat sizeNode[3];
	sizeNode[0]=0.05;
	sizeNode[1]=0.05;
	sizeNode[2]=0.0;

	simFloat sizeLink[3];
	sizeLink[0]=0.001;
	sizeLink[1]=0.02;

	//colors
	simFloat treeColor[3];
	treeColor[0] = 0;
	treeColor[1] = 0.8;
	treeColor[2] = 0;
	
	simFloat purpleColor[3];
	purpleColor[0] = 0.9;
	purpleColor[1] = 0;
	purpleColor[2] = 0.7;

	
	simFloat newPosition[3];
	simFloat newOrientation[3];
	
	if (pointsToBePlotted.size()>0)
	{
		
		int treshold = pointsToBePlotted.size();

		for(int i = 0; i < treshold; i++)
		{

			simInt hNode;
			hNode= simCreatePureShape(2,16,sizeNode,0,NULL);
			newPosition[0]=TREEpoints[pointsToBePlotted[0]](0);
			newPosition[1]=TREEpoints[pointsToBePlotted[0]](1);
			newPosition[2]=0;
			simSetObjectPosition(hNode, -1, newPosition);
			simSetObjectInt32Parameter(hNode, 3003, 1);
			simSetObjectInt32Parameter(hNode, 10, 1);
			simSetShapeColor(hNode,NULL,0,treeColor);	
			plottedTreePoints.push_back(hNode);

			//Plot new link of the tree
			sizeLink[2]=TREEdistancesFromParent[pointsToBePlotted[0]];

			simInt hLink;
			hLink= simCreatePureShape(0,16,sizeLink,0,NULL);
			newPosition[0]=( TREEpoints[pointsToBePlotted[0]](0) + TREEpoints[TREEparentsIndexes[pointsToBePlotted[0]]](0) )/2;
			newPosition[1]=( TREEpoints[pointsToBePlotted[0]](1) + TREEpoints[TREEparentsIndexes[pointsToBePlotted[0]]](1) )/2;
			newPosition[2]=0;
			simSetObjectPosition(hLink, -1, newPosition);

			newOrientation[0]= rotationAlpha;
			newOrientation[1]= TREEpoints[pointsToBePlotted[0]](2);
			newOrientation[2]= rotationGamma;
			
			simSetObjectOrientation(hLink, -1, newOrientation);

			simSetObjectInt32Parameter(hLink, 3003, 1);
			simSetObjectInt32Parameter(hLink, 10, 1);
			simSetShapeColor(hLink,NULL,0,treeColor);	
			plottedTreeLinks.push_back(hLink);

			pointsToBePlotted.erase(pointsToBePlotted.begin());
		}
			
	}
		
	if(rewiredLinksToBePlotted.size()>0)
	{
		int treshold = rewiredLinksToBePlotted.size();

		for(int i = 0; i< treshold; i++)
		{

			//change the link
			if(TREEpoints[ rewiredLinksToBePlotted[0] ] != nullPoint)
				sizeLink[2]=TREEdistancesFromParent[ rewiredLinksToBePlotted[0] ];

			simInt hNew;
			hNew = simCreatePureShape(0,16,sizeLink,0,NULL);

			newPosition[0]=( TREEpoints[rewiredLinksToBePlotted[0]](0) + TREEpoints[rewiringIndexesPoint[0]][0] )/2;
			newPosition[1]=( TREEpoints[rewiredLinksToBePlotted[0]](1) + TREEpoints[rewiringIndexesPoint[0]][1] )/2;
			newPosition[2]=0.0001;
			simSetObjectPosition(hNew, -1, newPosition);

			newOrientation[0]= rotationAlpha;
			newOrientation[1]= TREEpoints[rewiredLinksToBePlotted[0]](2);
			newOrientation[2]= rotationGamma;
			simSetObjectOrientation(hNew, -1, newOrientation);

			simSetObjectInt32Parameter(hNew, 3003, 1);
			simSetObjectInt32Parameter(hNew, 10, 1);
			simSetShapeColor(hNew,NULL,0,treeColor);	

			if(TREEpoints[ rewiredLinksToBePlotted[0] ] != nullPoint)
				simRemoveObject(plottedTreeLinks[rewiredLinksToBePlotted[0]]);

			plottedTreeLinks[rewiredLinksToBePlotted[0]] = hNew;

			rewiredLinksToBePlotted.erase( rewiredLinksToBePlotted.begin() );
			rewiringIndexesPoint.erase( rewiringIndexesPoint.begin() );
		}
		
	}
	
	if(PATHpointsList.size()>0)
	{	
		
		if(plottedPathPoints.size()>1)
		{
			for(int i=0; i<plottedPathPoints.size()-(1*numOfStep); i++)
			{
				simRemoveObject(plottedPathPoints[i]);
				simRemoveObject(plottedPathLinks[i]);
			}
			
		}
		simFloat size[3];
		size[0]=0.05;
		size[1]=0.05;
		size[2]=0.0;
		simFloat newPosition[3];
		
		simFloat pastPathColor[3];
		pastPathColor[0] = 1;
		pastPathColor[1] = 0;
		pastPathColor[2] = 0;

		if(PATHpointsList.size()>1*numOfStep & RRT==2)
		{
			for(int i=0; i<numOfStep; i++)
			{
				simSetShapeColor(plottedPathPoints[plottedPathPoints.size()-1-i],NULL,0,pastPathColor);
				simSetShapeColor(plottedPathLinks[plottedPathPoints.size()-1-i],NULL,0,pastPathColor);
			}
		}

		plottedPathPoints.clear();
		plottedPathLinks.clear();

		//Plot path points
		simFloat pathColor[3];
		pathColor[0] = 0;
		pathColor[1] = 0;
		pathColor[2] = 1;

		int lastIndex=PATHpointsList.size()-1;
				
		for(int i=0; i<PATHpointsList[lastIndex].size()-1; i++)
		{
			simInt hNew;
			hNew= simCreatePureShape(2,16,size,0,NULL);
			newPosition[0]=PATHpointsList[lastIndex][i](0);
			newPosition[1]=PATHpointsList[lastIndex][i](1);
			newPosition[2]=0.0015;
			simSetObjectPosition(hNew, -1, newPosition);
			simSetObjectInt32Parameter(hNew, 3003, 1);
			simSetObjectInt32Parameter(hNew, 10, 1);
			simSetShapeColor(hNew,NULL,0,pathColor);

			plottedPathPoints.push_back(hNew);

			//Plot new link of the tree
			sizeLink[2]=TREEdistancesFromParent[PATHpointsIndexesList[lastIndex][i]];

			simInt hLink;
			hLink= simCreatePureShape(0,16,sizeLink,0,NULL);
			newPosition[0]=( TREEpoints[PATHpointsIndexesList[lastIndex][i]](0) + TREEpoints[TREEparentsIndexes[PATHpointsIndexesList[lastIndex][i]]](0) )/2;
			newPosition[1]=( TREEpoints[PATHpointsIndexesList[lastIndex][i]](1) + TREEpoints[TREEparentsIndexes[PATHpointsIndexesList[lastIndex][i]]](1) )/2;
			newPosition[2]=0.0015;
			simSetObjectPosition(hLink, -1, newPosition);

			newOrientation[0]= rotationAlpha;
			newOrientation[1]= TREEpoints[PATHpointsIndexesList[lastIndex][i]](2);
			newOrientation[2]= rotationGamma;
			simSetObjectOrientation(hLink, -1, newOrientation);

			simSetObjectInt32Parameter(hLink, 3003, 1);
			simSetObjectInt32Parameter(hLink, 10, 1);
			simSetShapeColor(hLink,NULL,0,pathColor);	

			plottedPathLinks.push_back(hLink);
		}
		
		if(PATHpointsList.size()>1*numOfStep & RRT==2)
		{
			for(int i=0; i<numOfStep; i++)
			{
				simSetShapeColor(plottedPathPoints[plottedPathPoints.size()-1-i],NULL,0,purpleColor);
				simSetShapeColor(plottedPathLinks[plottedPathPoints.size()-1-i],NULL,0,purpleColor);
			}
		}
		
		
		indexPathPlotted = lastIndex;
		
	}
}

void pruneBranches(int index, int indexTreshold)
{
	if(  index!=indexTreshold  )
	{
		pointsToBePruned.push_back(index);

		for( int i=0; i<TREEchildrenIndexes[index].size(); i++ )
		{
			pruneBranches( TREEchildrenIndexes[index][i] , indexTreshold );
		}
		
		TREEpoints[index] = nullPoint;
		TREEparentsIndexes[index]=-1;
		TREEchildrenIndexes[index].clear();
		TREEdistancesFromParent[index]= -1;
	}
	else
	{
		TREEparentsIndexes[indexTreshold]=0;
		TREEdistancesFromParent[indexTreshold]= 0;
	}
}


void Planning_init(){
	
	if(!RRT_IsRunning)
	{
		
		std::cout << "Simulation is Running" << std::endl;
		TREEpoints.clear();
		TREEparentsIndexes.clear();
		TREEchildrenIndexes.clear();
		TREEdistancesFromParent.clear();
		PATHpoints.clear();
		PATHpointsIndexes.clear();
		PATHpointsNew.clear();
		PATHpointsIndexesNew.clear();
		PATHpointsList.clear();
		PATHpointsIndexesList.clear();
		PATHcosts.clear();
		PATHtimes.clear();
		objects.clear();
		
		plottedPathPoints.clear();
		plottedPathLinks.clear();
		plottedTreePoints.clear();
		plottedTreeLinks.clear();
		
		rootIndex=0;
		previousRootIndex=0;
		pathInterpolation=false;
		pathInterpolation2=0;
		goalReached = false;
		indexPathPlotted=0;
		
		pointsToBePlotted.clear();
		pointsToBePruned.clear();
		rewiredLinksToBePlotted.clear();
		rewiringIndexesPoint.clear();
		timeBudgetReached= false;
		finalCost=0;
		previousCost=0;
		GOALindex=NULL;
		
		i_path = 0;
		path.clear();	
		
		start=0;
		stop=0;
		solutionFound=false;
		solutionUpdated=false;
		
		//get initial position of the youBot
		simFloat pYouBot[3];
		simInt hCube = simGetObjectHandle("youBot");
		simGetObjectPosition(hCube, -1, pYouBot);
		height = pYouBot[2]; //store the fixed height of the youBot

		//get the position of the goal
		simFloat pGoal[3];
		simInt hGoal = simGetObjectHandle("Goal");
		simGetObjectPosition(hGoal, -1, pGoal);

		//get initial orientation of the robot 
		simFloat oYouBot[3];
		simGetObjectOrientation(hCube, -1, oYouBot);
		rotationAlpha = -pi/2; //store the fixed rotation of the alpha angle for the youBot
		rotationGamma = -pi/2; //store the fixed rotation of the gamma angle for the youBot
		if (oYouBot[0]>0)
		{
			oYouBot[1]=pi-oYouBot[1];
		}
		else if(oYouBot[0]==0)
		{
			if(oYouBot[1]>0)
				oYouBot[1]=pi/2;
			else
				oYouBot[1]=-pi/2;
		}

		//save the obstacles into a vector
		simInt hDummy = simGetObjectHandle("Obstacles");
		int counter = 0;
		while (true)
		{
			simInt hObject;
			hObject = simGetObjectChild(hDummy, counter);
			if (hObject!=-1)
			{
				objects.push_back(hObject);
				counter++;
			}
			else
				break;
		}
		if (objects.size()==0)
		{
			simFloat size[3];
			size[0]=0.0;
			size[1]=0.0;
			size[2]=0.0;

			simInt obj;
			obj= simCreatePureShape(2,2,size,0,NULL);
			objects.push_back(obj);
		}


		//********************************RRTstar*********************************

		start = clock();
		stop = clock();
		
		//message that RRT* algorithm is started
		simAddStatusbarMessage("RRT* algorithm started");

		//seeds random functions
		srand48 ( time(NULL) );

		ROOT(0)=pYouBot[0];		//x coordinate
		ROOT(1)=pYouBot[1];		//y coordinate
		ROOT(2)=oYouBot[1];		//rotationBeta
		
		GOAL(0)= pGoal[0];
		GOAL(1)= pGoal[1];
		GOAL(2)= NULL;		 //we don't need it
		
		//insert the root in the tree
		TREEpoints.push_back(ROOT);
		TREEparentsIndexes.push_back(-1);
		plottedTreePoints.push_back( nullSimInt );
		
		std::vector<int> emptyVector;
		emptyVector.clear();
		TREEchildrenIndexes.push_back(emptyVector);
		
		TREEdistancesFromParent.push_back(0);
		plottedTreeLinks.push_back( nullSimInt );
		
		//Plot Root
		simFloat rootSize[3];
		rootSize[0]=0.1;
		rootSize[1]=0.1;
		rootSize[2]=0.0;
		simFloat rootPosition[3];
		simInt hRoot;
		simFloat rootColor[3];
		rootColor[0] = 0;
		rootColor[1] = 1;
		rootColor[2] = 0;

		hRoot= simCreatePureShape(2,2,rootSize,0,NULL);
		rootPosition[0]=ROOT(0);
		rootPosition[1]=ROOT(1);
		rootPosition[2]=0;
		simSetObjectPosition(hRoot, -1, rootPosition);
		simSetObjectInt32Parameter(hRoot, 3003, 1);
		simSetObjectInt32Parameter(hRoot, 10, 1);
		simSetShapeColor(hRoot,NULL,0,rootColor);

		RRT_IsRunning=true;
		
	}
}

void PlanningDuringExecution()
{
	//RRT star algorithm
	
	//get initial position of the youBot
	simInt hOriCube = simGetObjectHandle("youBot"); //original model of the youBot
	
	simInt hArray[1];
	hArray[0] = hOriCube;
	
	simCopyPasteObjects(hArray,1,1); //copied model of the youBot
	
	simInt hCopiedCube;
	hCopiedCube = simGetObjectHandle("youBot#0");
	simSetModelProperty(hCopiedCube,sim_modelproperty_not_visible);
	simSetObjectInt32Parameter(hCopiedCube, 3004, 0);
	
	while(true)
	{
		simAddStatusbarMessage("planning thread is running");
		restart=false;
		
		if(pathInterpolation)
		{			
			pathInterpolation2=true;
			// define a simple path (parameterized in s, [0,1])
			while(pathInterpolation2)
			{}
				
			path.clear();
			i_path=0;
			Eigen::Vector3f p_s;
			
			for(int c=0; c<numOfStep; c++)
			{	
				if(PATHpointsList[PATHpointsList.size()-1].size()>c+1)
				{
					previousCost+=TREEdistancesFromParent[PATHpointsIndexesList[PATHpointsIndexesList.size()-1][PATHpointsIndexesList[PATHpointsIndexesList.size()-1].size()-2-c]];
					
					int interpolationN = TREEdistancesFromParent[PATHpointsIndexesList[PATHpointsIndexesList.size()-1-c][PATHpointsIndexesList[PATHpointsIndexesList.size()-1].size()-2-c]]*80;
					int n = interpolationN;
					float s = 0.0;
					float s_inc = 1.0 / (float)n; 

					for(int i = 0; i < n+1; i++)
					{
						//interpolation variable
						s = i * s_inc;

						//position
						p_s[0] = s * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][0] + (1.0 - s) * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-1-c][0];
						p_s[1] = s * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][1] + (1.0 - s) * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-1-c][1];
						//orientation
						if(PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][2] - PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-1-c][2] > pi)
							PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][2] -= 2*pi;
						else if(PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][2] - PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-1-c][2] < -pi)
							PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][2] += 2*pi;

						p_s[2] = s * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-2-c][2] + (1.0 - s) * PATHpointsList[PATHpointsList.size()-1][PATHpointsList[PATHpointsList.size()-1].size()-1-c][2];

						path.push_back(p_s);
					}
				}
			}

			previousRootIndex=rootIndex;
			if(PATHpointsIndexesList[PATHpointsIndexesList.size()-1].size()>numOfStep)
				rootIndex=PATHpointsIndexesList[PATHpointsIndexesList.size()-1][PATHpointsIndexesList[PATHpointsIndexesList.size()-1].size()-1-(1*numOfStep)];
			else
				rootIndex=PATHpointsIndexesList[PATHpointsIndexesList.size()-1][0];
				
			if(RRT==2)
				pruneBranches(previousRootIndex,rootIndex);

			for(int i=0; i<numOfStep; i++)
			{
				PATHpointsIndexesList[PATHpointsIndexesList.size()-1].pop_back();
				PATHpointsList[PATHpointsList.size()-1].pop_back();
			}

			pathInterpolation=false;
		}
		
		if(RRT==2)
		{
			//generate a random point in the map
			float newX = (drand48() * MAP_WIDTH)-(MAP_WIDTH/2);
			float newY = (drand48() * MAP_HEIGHT)-(MAP_HEIGHT/2);

			//position of new point
			Eigen::Vector3f newPoint;
			newPoint(0)=newX;
			newPoint(1)=newY;
			newPoint(2)=NULL; //it will be replaced

			//range for near points
			float maxDistance = MAP_WIDTH+MAP_HEIGHT;
			int nearestPointIndex = -1;

			//find nearest point
			for(int c=0; c<TREEpoints.size(); c++)
			{
				if(TREEpoints[c]!=nullPoint)
				{
					//distance from 2 points
					Eigen::Vector3f difference = newPoint-TREEpoints[c];
					float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

					//if the new point is the nearest...
					if(module<maxDistance)
					{
						maxDistance=module;
						nearestPointIndex=c;
					}
				}
			}

			//range for near points
			float range = (MAP_WIDTH+MAP_HEIGHT)/20;

			//stepsize
			float stepsize = range * 0.5;

			//distance from 2 points
			Eigen::Vector3f difference = newPoint-TREEpoints[nearestPointIndex];
			float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

			if(module>stepsize)
			{
				float stepsizeModuleRatio = stepsize/module;
				difference = difference*stepsizeModuleRatio;

				//generate new point
				newPoint = TREEpoints[nearestPointIndex] + difference;
			}

			//vector of near points
			std::vector<int> TREEnearPointsIndex;

			//find near points
			for(int c=0; c<TREEpoints.size(); c++)
			{
				if(TREEpoints[c]!=nullPoint)
				{
					//distance from 2 points
					Eigen::Vector3f difference = newPoint-TREEpoints[c];
					float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

					//if the new point is near...
					if(module<range)
					{
						TREEnearPointsIndex.push_back(c); //..add it into the vector of near points
					}
				}
			}

			//if there are near points
			if(TREEnearPointsIndex.size()!=0)
			{
				//minimum cost (initialized too big in order to be replaced)
				float minCost=MAP_WIDTH*MAP_HEIGHT*200000;

				//index of costless point
				int costlessPointIndex;
				float distanceFromCostlessPoint;

				//for all near points..
				for(int c=0; c<TREEnearPointsIndex.size(); c++)
				{
					//..get the cost
					float cost=0;
					int localNode=TREEnearPointsIndex[c];
					float distanceFromParent = TREEdistancesFromParent[localNode];
					while(localNode!=rootIndex)
					{
						cost+= distanceFromParent;
						localNode=TREEparentsIndexes[localNode];
						distanceFromParent = TREEdistancesFromParent[localNode];
					}

					//distance from 2 points
					Eigen::Vector3f difference = newPoint-TREEpoints[TREEnearPointsIndex[c]];
					float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

					float orientationDiff= atan2(difference[0],difference[1]);

					if(orientationDiff - TREEpoints[TREEnearPointsIndex[c]][2] > pi)
							orientationDiff -= 2*pi;
						else if(orientationDiff - TREEpoints[TREEnearPointsIndex[c]][2] < -pi)
							orientationDiff += 2*pi;
					difference[2]=orientationDiff-TREEpoints[TREEnearPointsIndex[c]][2];
					cost=cost+module;

					//store the costless node
					if(cost<minCost)
					{
						minCost=cost;
						costlessPointIndex=TREEnearPointsIndex[c];
						distanceFromCostlessPoint=module;

						//set the orientation
						newPoint[2]= atan2(difference[0],difference[1]);
					}
				}


				//cost to go
				//Get the cost from the c-th near point
				float nearPointCost=0;
				int localNode=costlessPointIndex;
				float distanceFromParent = TREEdistancesFromParent[localNode];
				while(localNode!=rootIndex)
				{
					nearPointCost+= distanceFromParent;
					localNode=TREEparentsIndexes[localNode];
					distanceFromParent = TREEdistancesFromParent[localNode];
				}

				//get the distance between new point and c-th near point
				Eigen::Vector3f difference = TREEpoints[costlessPointIndex]-newPoint;
				float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

				//branch and bound
				if(solutionFound)
				{
					Eigen::Vector3f differenceFromGoal = TREEpoints[GOALindex]-newPoint;
					float moduleFromGoal = sqrt((differenceFromGoal[0]*differenceFromGoal[0])+(differenceFromGoal[1]*differenceFromGoal[1]));
					if(nearPointCost+module+moduleFromGoal>finalCost+0.000001)
						restart=true;
				}


				if(!restart)
				{
					//collision detection
					bool collision = false;
					//check collision also for all the points of the internal path
					std::vector<Eigen::Vector3f> localpath;	
					int n = 50;
					float s = 0.0;
					float s_inc = 1.0 / (float)n; 
					Eigen::Vector3f p_s;

					for(int i = 1; i < n+1; i++)
					{
						//interpolation variable
						s = i * s_inc;

						//interpolate points
						p_s[0] = s * newPoint[0]  + (1.0 - s) * TREEpoints[costlessPointIndex][0];
						p_s[1] = s * newPoint[0]  + (1.0 - s) * TREEpoints[costlessPointIndex][1];

						if(newPoint[2] - TREEpoints[costlessPointIndex][2] > pi)
							newPoint[2] -= 2*pi;
						else if(newPoint[2] - TREEpoints[costlessPointIndex][2] < -pi)
							newPoint[2] += 2*pi;
						p_s[2] = s * newPoint[2] + (1.0 - s) * TREEpoints[costlessPointIndex][2];
						//push the points into the local path vector
						localpath.push_back(p_s);

					}

					//check collision for the internal path
					for(int c=0; c<localpath.size(); c++)
					{
						//position of the local point
						simFloat pLocalPoint[3];
						pLocalPoint[0]=localpath[c](0);
						pLocalPoint[1]=localpath[c](1);
						pLocalPoint[2]=height;

						//orientation of the local point
						simFloat oLocalPoint[3];
						oLocalPoint[0]=rotationAlpha;
						oLocalPoint[1]=localpath[c](2);
						oLocalPoint[2]=rotationGamma;

						//check collision for the current point
						for(int i=0; i<objects.size(); i++)
						{
							simSetObjectPosition(hCopiedCube, -1, pLocalPoint );
							simSetObjectOrientation(hCopiedCube, -1, oLocalPoint );

							if(simCheckCollision(hCopiedCube, objects[i])==1)
							{
								collision=true;
								break;
							}
						}
					}


					//if there is not collision
					if (!collision)
					{
						//insert the new point, his cost, his parent in the tree and the child to the children's vector
						TREEpoints.push_back(newPoint);
						TREEparentsIndexes.push_back(costlessPointIndex);

						std::vector<int> emptyVector;
						emptyVector.clear();
						TREEchildrenIndexes.push_back(emptyVector);
						TREEchildrenIndexes[costlessPointIndex].push_back(TREEpoints.size()-1);

						TREEdistancesFromParent.push_back(distanceFromCostlessPoint);
						pointsToBePlotted.push_back(TREEpoints.size()-1);

						//cost of the new point
						float newPointCost = minCost;


						//REWIRING OF THE TREE:
						//find near points again, becouse new point now is different

						//check collision for the rewired path
						for(int c=0; c<TREEnearPointsIndex.size(); c++)
						{
							//collision detection between new point and c-th near point
							collision = false;
							//check collision for all the points of the internal path
							localpath.clear();	

							//difference
							float newAngle = atan2(difference[0],difference[1]);

							for(int i = 1; i < n; i++){
								//interpolation variable
								s = i * s_inc;

								//position
								p_s[0] = s * newPoint[0] + (1.0 - s) * TREEpoints[TREEnearPointsIndex[c]][0];
								p_s[1] = s * newPoint[1] + (1.0 - s) * TREEpoints[TREEnearPointsIndex[c]][1];

								//orientation
								if(newPoint[2] - newAngle > pi)
									newPoint[2] -= 2*pi;
								else if(newPoint[2] - newAngle < -pi)
									newPoint[2] += 2*pi;
								p_s[2] = s * newPoint[2] + (1.0 - s) * newAngle;

								//push the points into the local path vector
								localpath.push_back(p_s);

							}
							//check collision for each point of the rewired path
							for(int c=0; c<localpath.size(); c++)
							{
								simFloat pLocalPoint[3];
								pLocalPoint[0]=localpath[c](0);
								pLocalPoint[1]=localpath[c](1);
								pLocalPoint[2]=height;

								simFloat oLocalPoint[3];
								oLocalPoint[0]=rotationAlpha;
								oLocalPoint[1]=localpath[c](2);
								oLocalPoint[2]=rotationGamma;

								for(int i=0; i<objects.size(); i++)
								{
									simSetObjectPosition(hCopiedCube, -1, pLocalPoint );
									simSetObjectOrientation(hCopiedCube, -1, oLocalPoint );
									if(simCheckCollision(hCopiedCube, objects[i])==1)
									{
										collision=true;
										break;
									}
								}
							}

							//if there is not collision
							if(!collision)
							{
								//Get the cost from the c-th near point
								float nearPointCost=0;
								int localNode=TREEnearPointsIndex[c];
								float distanceFromParent = TREEdistancesFromParent[localNode];
								while(localNode!=rootIndex)
								{
									nearPointCost+= distanceFromParent;
									localNode=TREEparentsIndexes[localNode];
									distanceFromParent = TREEdistancesFromParent[localNode];
								}

								//get the distance between new point and c-th near point
								Eigen::Vector3f difference = TREEpoints[TREEnearPointsIndex[c]]-newPoint;
								float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

								//if the cost of the new point+ the distance is less than the cost of the near point
								//0.000001 is a guard distance for avoiding the case of equality
								if(newPointCost+module+0.000001<nearPointCost)
								{

									//add the child to the new point(REWIRING)
									TREEchildrenIndexes[TREEpoints.size()-1].push_back(TREEnearPointsIndex[c]);

									//remove the child from the parent of the near point
									for( int i=0; i<TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].size(); i++)
										if ( TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]][i] == TREEnearPointsIndex[c] )
											TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].erase(TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].begin()+i);

									//replace the parent of the c-th near point with the new point (REWIRING)
									TREEparentsIndexes[TREEnearPointsIndex[c]]=TREEpoints.size()-1;

									//and change the distance from this parent
									TREEdistancesFromParent[TREEnearPointsIndex[c]]=module;

									TREEpoints[TREEnearPointsIndex[c]][2]=atan2(difference[0],difference[1]);

									rewiredLinksToBePlotted.push_back(TREEnearPointsIndex[c]);
									rewiringIndexesPoint.push_back(TREEpoints.size()-1);
								}
							}

						}


						//distance between new point and the goal
						Eigen::Vector3f difference = newPoint-GOAL;
						float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

						PATHpointsNew.clear();
						PATHpointsIndexesNew.clear();


						int parent=GOALindex;
						float newCost =0;
						//other points of the path

						while(parent!=rootIndex)
						{
							newCost += TREEdistancesFromParent[parent];
							PATHpointsNew.push_back(TREEpoints[parent]);
							PATHpointsIndexesNew.push_back(parent);
							parent=TREEparentsIndexes[parent];
						}
						PATHpointsNew.push_back(TREEpoints[parent]);
						PATHpointsIndexesNew.push_back(parent);

						if(newCost!=finalCost)
						{
							finalCost=newCost;

							PATHpointsList.push_back(PATHpointsNew);
							PATHpointsIndexesList.push_back(PATHpointsIndexesNew);

							PATHcosts.push_back(finalCost+previousCost);
							PATHtimes.push_back(simGetSimulationTime());

							simAddStatusbarMessage("Path improved");
							solutionUpdated=true;
						}


					}

				}
			}
			
		}
		
		if ( !timeBudgetReached | goalReached)
		{
			if(RRT==2)
				removeTreePlots();

			simFloat pastPathColor[3];
			pastPathColor[0] = 1;
			pastPathColor[1] = 0;
			pastPathColor[2] = 0;

			if(PATHpointsList.size()>numOfStep & RRT==2)
			for(int i=0; i<numOfStep; i++)
			{
				simSetShapeColor(plottedPathPoints[plottedPathPoints.size()-1-i],NULL,0,pastPathColor);
				simSetShapeColor(plottedPathLinks[plottedPathPoints.size()-1-i],NULL,0,pastPathColor);
			}

			for(int i=0; i<PATHcosts.size(); i++)
			std::cout << "cost n° " << i << " =  " << PATHcosts[i] << " at time: " << PATHtimes[i] << "\n";

			simAddStatusbarMessage("Thread stopped");
			pthread_exit(NULL);
			break;
		}

	}
}


void *planningThread(void *thread_id) {
	long tid = (long)thread_id;
	PlanningDuringExecution();
	return NULL;
}




void Planning(){
	//RRT star algorithm
	restart=false;

	//get initial position of the youBot
	simInt hCube = simGetObjectHandle("youBot");

	//generate a random point in the map
	float newX = (drand48() * MAP_WIDTH)-(MAP_WIDTH/2);
	float newY = (drand48() * MAP_HEIGHT)-(MAP_HEIGHT/2);

	//position of new point
	Eigen::Vector3f newPoint;
	newPoint(0)=newX;
	newPoint(1)=newY;
	newPoint(2)=NULL; //it will be replaced

	//range for near points
	float maxDistance = MAP_WIDTH+MAP_HEIGHT;
	int nearestPointIndex = -1;

	//find nearest point
	for(int c=0; c<TREEpoints.size(); c++)
	{
		if(TREEpoints[c]!=nullPoint)
		{
			//distance from 2 points
			Eigen::Vector3f difference = newPoint-TREEpoints[c];
			float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

			//if the new point is the nearest...
			if(module<maxDistance)
			{
				maxDistance=module;
				nearestPointIndex=c;
			}
		}
	}

	
	//range for near points
	float range = (MAP_WIDTH+MAP_HEIGHT)/20;

	//stepsize
	float stepsize = range * 0.5;

	//distance from 2 points
	Eigen::Vector3f difference = newPoint-TREEpoints[nearestPointIndex];
	float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

	if(module>stepsize)
	{
		float stepsizeModuleRatio = stepsize/module;
		difference = difference*stepsizeModuleRatio;

		//generate new point
		newPoint = TREEpoints[nearestPointIndex] + difference;
	}
	
	
	if(RRT>=1)
	{
		//vector of near points
		std::vector<int> TREEnearPointsIndex;

		//find near points
		for(int c=0; c<TREEpoints.size(); c++)
		{
			if(TREEpoints[c]!=nullPoint)
			{
				//distance from 2 points
				Eigen::Vector3f difference = newPoint-TREEpoints[c];
				float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

				//if the new point is near...
				if(module<range)
				{
					TREEnearPointsIndex.push_back(c); //..add it into the vector of near points
				}
			}
		}

		//if there are near points
		if(TREEnearPointsIndex.size()!=0)
		{
			//minimum cost (initialized too big in order to be replaced)
			float minCost=MAP_WIDTH*MAP_HEIGHT*200000;

			//index of costless point
			int costlessPointIndex;
			float distanceFromCostlessPoint;

			//for all near points..
			for(int c=0; c<TREEnearPointsIndex.size(); c++)
			{
				//..get the cost
				float cost=0;
				int localNode=TREEnearPointsIndex[c];
				float distanceFromParent = TREEdistancesFromParent[localNode];
				while(localNode!=rootIndex)
				{
					cost+= distanceFromParent;
					localNode=TREEparentsIndexes[localNode];
					distanceFromParent = TREEdistancesFromParent[localNode];
				}

				//distance from 2 points
				Eigen::Vector3f difference = newPoint-TREEpoints[TREEnearPointsIndex[c]];
				float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

				cost=cost+module;

				//store the costless node
				if(cost<minCost)
				{
					minCost=cost;
					costlessPointIndex=TREEnearPointsIndex[c];
					distanceFromCostlessPoint=module;

					//set the orientation
					newPoint[2]= atan2(difference[0],difference[1]);
				}
			}


			//cost to go
			//Get the cost from the c-th near point
			float nearPointCost=0;
			int localNode=costlessPointIndex;
			float distanceFromParent = TREEdistancesFromParent[localNode];
			while(localNode!=rootIndex)
			{
				nearPointCost+= distanceFromParent;
				localNode=TREEparentsIndexes[localNode];
				distanceFromParent = TREEdistancesFromParent[localNode];
			}

			//get the distance between new point and c-th near point
			Eigen::Vector3f difference = TREEpoints[costlessPointIndex]-newPoint;
			float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

			//Branch and bound
			if(solutionFound)
			{
				Eigen::Vector3f differenceFromGoal = TREEpoints[GOALindex]-newPoint;
				float moduleFromGoal = sqrt((differenceFromGoal[0]*differenceFromGoal[0])+(differenceFromGoal[1]*differenceFromGoal[1]));
				if(nearPointCost+module+moduleFromGoal>finalCost+0.000001)
					restart=true;
			}


			if(!restart)
			{
				//collision detection
				bool collision = false;
				//check collision also for all the points of the internal path
				std::vector<Eigen::Vector3f> localpath;	
				int n = 50;
				float s = 0.0;
				float s_inc = 1.0 / (float)n; 
				Eigen::Vector3f p_s;

				for(int i = 1; i < n+1; i++)
				{
					//interpolation variable
					s = i * s_inc;
					//interpolate points
					p_s[0] = s * newPoint[0]  + (1.0 - s) * TREEpoints[costlessPointIndex][0];
					p_s[1] = s * newPoint[1]  + (1.0 - s) * TREEpoints[costlessPointIndex][1];

					if(newPoint[2] - TREEpoints[costlessPointIndex][2] > pi)
							newPoint[2] -= 2*pi;
						else if(newPoint[2] - TREEpoints[costlessPointIndex][2] < -pi)
							newPoint[2] += 2*pi;
					p_s[2] = s * newPoint[2]  + (1.0 - s) * TREEpoints[costlessPointIndex][2];

					//push the points into the local path vector
					localpath.push_back(p_s);

				}

				//check collision for the internal path
				for(int c=0; c<localpath.size(); c++)
				{
					//position of the local point
					simFloat pLocalPoint[3];
					pLocalPoint[0]=localpath[c](0);
					pLocalPoint[1]=localpath[c](1);
					pLocalPoint[2]=height;

					//orientation of the local point
					simFloat oLocalPoint[3];
					oLocalPoint[0]=rotationAlpha;
					oLocalPoint[1]=localpath[c](2);
					oLocalPoint[2]=rotationGamma;

					//check collision for the current point
					for(int i=0; i<objects.size(); i++)
					{
						simSetObjectPosition(hCube, -1, pLocalPoint );
						simSetObjectOrientation(hCube, -1, oLocalPoint );

						if(simCheckCollision(hCube, objects[i])==1)
						{
							collision=true;
							break;
						}
					}
				}

				//if there is not collision
				if (!collision)
				{
					//insert the new point, his cost and his parent in the tree
					TREEpoints.push_back(newPoint);
					TREEparentsIndexes.push_back(costlessPointIndex);

					std::vector<int> emptyVector;
					emptyVector.clear();
					TREEchildrenIndexes.push_back(emptyVector);
					TREEchildrenIndexes[costlessPointIndex].push_back(TREEpoints.size()-1);

					TREEdistancesFromParent.push_back(distanceFromCostlessPoint);

					//cost of the new point
					float newPointCost = minCost;

					//Plot new point of the tree
					simFloat size[3];
					size[0]=0.05;
					size[1]=0.05;
					size[2]=0.0;
					simFloat newPosition[3];
					simFloat newOrientation[3];

					simFloat treeColor[3];
					treeColor[0] = 0;
					treeColor[1] = 0.8;
					treeColor[2] = 0;

					simInt hNew;
					hNew= simCreatePureShape(2,16,size,0,NULL);
					newPosition[0]=TREEpoints[TREEpoints.size()-1](0);
					newPosition[1]=TREEpoints[TREEpoints.size()-1](1);
					newPosition[2]=0;
					simSetObjectPosition(hNew, -1, newPosition);
					simSetObjectInt32Parameter(hNew, 3003, 1);
					simSetObjectInt32Parameter(hNew, 10, 1);
					simSetShapeColor(hNew,NULL,0,treeColor);	
					plottedTreePoints.push_back(hNew);

					//Plot new link of the tree
					size[0]=0.001;
					size[1]=0.02;
					size[2]=distanceFromCostlessPoint;

					hNew= simCreatePureShape(0,16,size,0,NULL);
					newPosition[0]=( TREEpoints[TREEpoints.size()-1](0) + TREEpoints[TREEparentsIndexes[TREEpoints.size()-1]](0) )/2;
					newPosition[1]=( TREEpoints[TREEpoints.size()-1](1) + TREEpoints[TREEparentsIndexes[TREEpoints.size()-1]](1) )/2;
					newPosition[2]=0;
					simSetObjectPosition(hNew, -1, newPosition);


					newOrientation[0]= rotationAlpha;
					newOrientation[1]= TREEpoints[TREEpoints.size()-1](2);
					newOrientation[2]= rotationGamma;
					simSetObjectOrientation(hNew, -1, newOrientation);

					simSetObjectInt32Parameter(hNew, 3003, 1);
					simSetObjectInt32Parameter(hNew, 10, 1);
					simSetShapeColor(hNew,NULL,0,treeColor);	
					plottedTreeLinks.push_back(hNew);

					//REWIRING OF THE TREE:
					//find near points again, becouse new point now is different

					//check collision for the rewired path
					for(int c=0; c<TREEnearPointsIndex.size(); c++)
					{
						//collision detection between new point and c-th near point
						collision = false;
						//check collision for all the points of the internal path
						localpath.clear();	

						//difference
						float newAngle = atan2(difference[0],difference[1]);

						for(int i = 1; i < n; i++){
							//interpolation variable
							s = i * s_inc;

							//position
							p_s[0] = s * newPoint[0] + (1.0 - s) * TREEpoints[TREEnearPointsIndex[c]][0];
							p_s[1] = s * newPoint[1] + (1.0 - s) * TREEpoints[TREEnearPointsIndex[c]][1];

							//orientation
							if(newPoint[2] - newAngle > pi)
								newPoint[2] -= 2*pi;
							else if(newPoint[2] - newAngle < -pi)
								newPoint[2] += 2*pi;
							p_s[2] = s * newPoint[2] + (1.0 - s) * newAngle;

							//push the points into the local path vector
							localpath.push_back(p_s);

						}
						//check collision for each point of the rewired path
						for(int c=0; c<localpath.size(); c++)
						{
							simFloat pLocalPoint[3];
							pLocalPoint[0]=localpath[c](0);
							pLocalPoint[1]=localpath[c](1);
							pLocalPoint[2]=height;

							simFloat oLocalPoint[3];
							oLocalPoint[0]=rotationAlpha;
							oLocalPoint[1]=localpath[c](2);
							oLocalPoint[2]=rotationGamma;

							for(int i=0; i<objects.size(); i++)
							{
								simSetObjectPosition(hCube, -1, pLocalPoint );
								simSetObjectOrientation(hCube, -1, oLocalPoint );
								if(simCheckCollision(hCube, objects[i])==1)
								{
									collision=true;
									break;
								}
							}
						}


						//if there is not collision
						if(!collision)
						{
							//Get the cost from the c-th near point
							float nearPointCost=0;
							int localNode=TREEnearPointsIndex[c];
							float distanceFromParent = TREEdistancesFromParent[localNode];
							while(localNode!=rootIndex)
							{
								nearPointCost+= distanceFromParent;
								localNode=TREEparentsIndexes[localNode];
								distanceFromParent = TREEdistancesFromParent[localNode];
							}

							//get the distance between new point and c-th near point
							Eigen::Vector3f difference = TREEpoints[TREEnearPointsIndex[c]]-newPoint;
							float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));

							//if the cost of the new point+ the distance is less than the cost of the near point
							//0.000001 is a guard distance for avoiding the case of equality
							if(newPointCost+module+0.000001<nearPointCost)
							{	
								//add the child to the new point(REWIRING)
								TREEchildrenIndexes[TREEpoints.size()-1].push_back(TREEnearPointsIndex[c]);

								int childToBeRemovedIndex = NULL;
								//remove the child from the parent of the near point (REWIRING)
								for( int i=0; i<TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].size(); i++)
									if ( TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]][i] == TREEnearPointsIndex[c] )
									{
										TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].erase(TREEchildrenIndexes[TREEparentsIndexes[TREEnearPointsIndex[c]]].begin()+i);
									}

								//replace the parent of the c-th near point with the new point (REWIRING)
								TREEparentsIndexes[TREEnearPointsIndex[c]]=TREEpoints.size()-1;

								//and change the distance from this parent
								TREEdistancesFromParent[TREEnearPointsIndex[c]]=module;

								TREEpoints[TREEnearPointsIndex[c]][2]=atan2(difference[0],difference[1]);

								//change the link
								size[0]=0.001;
								size[1]=0.02;
								size[2]=module;

								hNew = simCreatePureShape(0,16,size,0,NULL);

								newPosition[0]=( TREEpoints[TREEnearPointsIndex[c]](0) + newPoint[0] )/2;
								newPosition[1]=( TREEpoints[TREEnearPointsIndex[c]](1) + newPoint[1] )/2;
								newPosition[2]=0.0001;
								simSetObjectPosition(hNew, -1, newPosition);

								newOrientation[0]= rotationAlpha;
								newOrientation[1]= TREEpoints[TREEnearPointsIndex[c]](2);
								newOrientation[2]= rotationGamma;
								simSetObjectOrientation(hNew, -1, newOrientation);

								simSetObjectInt32Parameter(hNew, 3003, 1);
								simSetObjectInt32Parameter(hNew, 10, 1);
								simSetShapeColor(hNew,NULL,0,treeColor);	

								simRemoveObject(plottedTreeLinks[TREEnearPointsIndex[c]]);
								plottedTreeLinks[TREEnearPointsIndex[c]] = hNew;
							}
						}

					}


					//distance between new point and the goal
					Eigen::Vector3f difference = newPoint-GOAL;
					float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));


					//if the new point reaches the goal, set RRT_IsRunning to false to stop the while, and build the path
					if (module<0.25 & !solutionFound)
					{
						PATHpoints.clear();
						PATHpointsIndexes.clear();

						//compute the cost
						int parent=TREEparentsIndexes.size()-1;

						//other points of the path
						while(parent!=rootIndex)
						{
							finalCost += TREEdistancesFromParent[parent];
							PATHpoints.push_back(TREEpoints[parent]);
							PATHpointsIndexes.push_back(parent);
							parent=TREEparentsIndexes[parent];
						}


						PATHpoints.push_back(TREEpoints[parent]);
						PATHpointsIndexes.push_back(parent);

						PATHpointsList.push_back(PATHpoints);
						PATHpointsIndexesList.push_back(PATHpointsIndexes);


						solutionFound=true;
						GOALindex=TREEpoints.size()-1;
						PATHcosts.push_back(finalCost);
						PATHtimes.push_back(simGetSimulationTime());
						simAddStatusbarMessage("Solution found!");

						pointsToBePruned.clear();
						for(int i=0; i<TREEpoints.size(); i++)
						{
							//cost to go
							//Get the cost from the c-th near point
							float costFromRoot=0;
							int localNode=i;
							float distanceFromParent = TREEdistancesFromParent[localNode];
							while(localNode!=rootIndex)
							{
								costFromRoot+= distanceFromParent;
								localNode=TREEparentsIndexes[localNode];
								distanceFromParent = TREEdistancesFromParent[localNode];
							}

							//branch and bound
							Eigen::Vector3f differenceFromGoal = TREEpoints[GOALindex]-TREEpoints[i];
							float moduleFromGoal = sqrt((differenceFromGoal[0]*differenceFromGoal[0])+(differenceFromGoal[1]*differenceFromGoal[1]));
							if(costFromRoot+moduleFromGoal>finalCost+0.000001)
							{
								simSetModelProperty(plottedTreePoints[i],sim_modelproperty_not_visible);
								simSetModelProperty(plottedTreeLinks[i],sim_modelproperty_not_visible);
								pointsToBePruned.push_back(i);
							}
						}

						if(pointsToBePruned.size()>0)
							for(int i=0; i<pointsToBePruned.size(); i++)
							{
								TREEpoints[pointsToBePruned[i]] = nullPoint;
								TREEparentsIndexes[pointsToBePruned[i]]=-1;
								TREEchildrenIndexes[pointsToBePruned[i]].clear();
								TREEdistancesFromParent[pointsToBePruned[i]]= -1;
							}
						pointsToBePruned.clear();

						solutionUpdated=true;
					}
					else if(solutionFound)
					{
						PATHpoints.clear();
						PATHpointsIndexes.clear();


						int parent=GOALindex;
						float newCost =0;
						//other points of the path
						while(parent!=rootIndex)
						{
							newCost += TREEdistancesFromParent[parent];
							PATHpoints.push_back(TREEpoints[parent]);
							PATHpointsIndexes.push_back(parent);
							parent=TREEparentsIndexes[parent];
						}
						PATHpoints.push_back(TREEpoints[parent]);
						PATHpointsIndexes.push_back(parent);

						PATHpointsList.push_back(PATHpoints);
						PATHpointsIndexesList.push_back(PATHpointsIndexes);

						if(newCost!=finalCost)
						{
							finalCost=newCost;
							PATHcosts.push_back(finalCost);
							PATHtimes.push_back(simGetSimulationTime());

							simAddStatusbarMessage("Path improved");
							solutionUpdated=true;
						}
					}
				}


				if(solutionUpdated)
				{
					if(plottedPathPoints.size()>0)
					{
						for(int i=0; i<plottedPathPoints.size(); i++)
							simRemoveObject(plottedPathPoints[i]);
					}
					plottedPathPoints.clear();
					if(plottedPathLinks.size()>0)
						for(int i=0; i<plottedPathLinks.size(); i++)
							simRemoveObject(plottedPathLinks[i]);

					plottedPathLinks.clear();


					simFloat size[3];
					size[0]=0.05;
					size[1]=0.05;
					size[2]=0.0;
					simFloat newPosition[3];

					simFloat treeColor[3];
					treeColor[0] = 0;
					treeColor[1] = 0.8;
					treeColor[2] = 0;

					//Plot path points
					simFloat pathColor[3];
					pathColor[0] = 0;
					pathColor[1] = 0;
					pathColor[2] = 1;

					for(int i=0; i<PATHpoints.size()-1; i++)
					{
						simInt hNew;
						hNew= simCreatePureShape(2,16,size,0,NULL);
						newPosition[0]=PATHpoints[i](0);
						newPosition[1]=PATHpoints[i](1);
						newPosition[2]=0.0015;
						simSetObjectPosition(hNew, -1, newPosition);
						simSetObjectInt32Parameter(hNew, 3003, 1);
						simSetObjectInt32Parameter(hNew, 10, 1);
						simSetShapeColor(hNew,NULL,0,pathColor);

						plottedPathPoints.push_back(hNew);
					}
					//Plot path links

					simFloat sizeLink[3];
					sizeLink[0]=0.001;
					sizeLink[1]=0.02;

					simFloat newOrientation[3];

					for(int i=0; i<PATHpointsIndexes.size(); i++)
					{
						//Plot new link of the tree
						sizeLink[2]=TREEdistancesFromParent[PATHpointsIndexes[i]];

						simInt hLink;
						hLink= simCreatePureShape(0,16,sizeLink,0,NULL);
						newPosition[0]=( TREEpoints[PATHpointsIndexes[i]](0) + TREEpoints[TREEparentsIndexes[PATHpointsIndexes[i]]](0) )/2;
						newPosition[1]=( TREEpoints[PATHpointsIndexes[i]](1) + TREEpoints[TREEparentsIndexes[PATHpointsIndexes[i]]](1) )/2;
						newPosition[2]=0.0015;
						simSetObjectPosition(hLink, -1, newPosition);

						newOrientation[0]= rotationAlpha;
						newOrientation[1]= TREEpoints[PATHpointsIndexes[i]](2);
						newOrientation[2]= rotationGamma;
						simSetObjectOrientation(hLink, -1, newOrientation);

						simSetObjectInt32Parameter(hLink, 3003, 1);
						simSetObjectInt32Parameter(hLink, 10, 1);
						simSetShapeColor(hLink,NULL,0,pathColor);	

						plottedPathLinks.push_back(hLink);
					}

					//************************************************************************


					solutionUpdated=false;
				}

				simFloat pRoot[3];
				pRoot[0]=ROOT(0);
				pRoot[1]=ROOT(1);
				pRoot[2]=height;

				simFloat oRoot[3];
				oRoot[0]=rotationAlpha;
				oRoot[1]=ROOT(2);
				oRoot[2]=rotationGamma;

				simSetObjectPosition(hCube, -1, pRoot );
				simSetObjectOrientation(hCube, -1, oRoot );

			}

			stop = clock();


			if(simGetSimulationTime()>30)
			{
				RRT_IsRunning=false;	//stop the RRT* algorithm
				timeBudgetReached=true;
				pthread_create(&planning_thread, NULL, planningThread, (void *)1);

			}

		}
	}
	else
	{
		//distance from 2 points
		Eigen::Vector3f difference = newPoint-TREEpoints[nearestPointIndex];
		float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));
		newPoint[2]= atan2(difference[0],difference[1]);

		//collision detection
		bool collision = false;
		//check collision also for all the points of the internal path
		std::vector<Eigen::Vector3f> localpath;	
		int n = 50;
		float s = 0.0;
		float s_inc = 1.0 / (float)n; 
		Eigen::Vector3f p_s;

		for(int i = 1; i < n+1; i++)
		{
			//interpolation variable
			s = i * s_inc;
			//interpolate points
			p_s[0] = s * newPoint[0]  + (1.0 - s) * TREEpoints[nearestPointIndex][0];
			p_s[1] = s * newPoint[1]  + (1.0 - s) * TREEpoints[nearestPointIndex][1];

			if(newPoint[2] - TREEpoints[nearestPointIndex][2] > pi)
					newPoint[2] -= 2*pi;
				else if(newPoint[2] - TREEpoints[nearestPointIndex][2] < -pi)
					newPoint[2] += 2*pi;
			p_s[2] = s * newPoint[2]  + (1.0 - s) * TREEpoints[nearestPointIndex][2];

			//push the points into the local path vector
			localpath.push_back(p_s);

		}

		//check collision for the internal path
		for(int c=0; c<localpath.size(); c++)
		{
			//position of the local point
			simFloat pLocalPoint[3];
			pLocalPoint[0]=localpath[c](0);
			pLocalPoint[1]=localpath[c](1);
			pLocalPoint[2]=height;

			//orientation of the local point
			simFloat oLocalPoint[3];
			oLocalPoint[0]=rotationAlpha;
			oLocalPoint[1]=localpath[c](2);
			oLocalPoint[2]=rotationGamma;

			//check collision for the current point
			for(int i=0; i<objects.size(); i++)
			{
				simSetObjectPosition(hCube, -1, pLocalPoint );
				simSetObjectOrientation(hCube, -1, oLocalPoint );

				if(simCheckCollision(hCube, objects[i])==1)
				{
					collision=true;
					break;
				}
			}
		}

		//if there is not collision
		if (!collision)
		{
			//insert the new point, his cost and his parent in the tree
			TREEpoints.push_back(newPoint);
			TREEparentsIndexes.push_back(nearestPointIndex);

			std::vector<int> emptyVector;
			emptyVector.clear();
			TREEchildrenIndexes.push_back(emptyVector);
			TREEchildrenIndexes[nearestPointIndex].push_back(TREEpoints.size()-1);

			TREEdistancesFromParent.push_back(module);

			if(!solutionFound)
			{
				//Plot new point of the tree
				simFloat size[3];
				size[0]=0.05;
				size[1]=0.05;
				size[2]=0.0;
				simFloat newPosition[3];
				simFloat newOrientation[3];

				simFloat treeColor[3];
				treeColor[0] = 0;
				treeColor[1] = 0.8;
				treeColor[2] = 0;

				simInt hNew;
				hNew= simCreatePureShape(2,16,size,0,NULL);
				newPosition[0]=TREEpoints[TREEpoints.size()-1](0);
				newPosition[1]=TREEpoints[TREEpoints.size()-1](1);
				newPosition[2]=0;
				simSetObjectPosition(hNew, -1, newPosition);
				simSetObjectInt32Parameter(hNew, 3003, 1);
				simSetObjectInt32Parameter(hNew, 10, 1);
				simSetShapeColor(hNew,NULL,0,treeColor);	
				plottedTreePoints.push_back(hNew);

				//Plot new link of the tree
				size[0]=0.001;
				size[1]=0.02;
				size[2]=module;

				hNew= simCreatePureShape(0,16,size,0,NULL);
				newPosition[0]=( TREEpoints[TREEpoints.size()-1](0) + TREEpoints[TREEparentsIndexes[TREEpoints.size()-1]](0) )/2;
				newPosition[1]=( TREEpoints[TREEpoints.size()-1](1) + TREEpoints[TREEparentsIndexes[TREEpoints.size()-1]](1) )/2;
				newPosition[2]=0;
				simSetObjectPosition(hNew, -1, newPosition);


				newOrientation[0]= rotationAlpha;
				newOrientation[1]= TREEpoints[TREEpoints.size()-1](2);
				newOrientation[2]= rotationGamma;
				simSetObjectOrientation(hNew, -1, newOrientation);

				simSetObjectInt32Parameter(hNew, 3003, 1);
				simSetObjectInt32Parameter(hNew, 10, 1);
				simSetShapeColor(hNew,NULL,0,treeColor);	
				plottedTreeLinks.push_back(hNew);
			}


			//distance between new point and the goal
			Eigen::Vector3f difference = newPoint-GOAL;
			float module = sqrt((difference[0]*difference[0])+(difference[1]*difference[1]));


			//if the new point reaches the goal, set RRT_IsRunning to false to stop the while, and build the path
			if (module<0.25 & !solutionFound)
			{
				PATHpoints.clear();
				PATHpointsIndexes.clear();

				//compute the cost
				int parent=TREEparentsIndexes.size()-1;

				//other points of the path
				while(parent!=rootIndex)
				{
					finalCost += TREEdistancesFromParent[parent];
					PATHpoints.push_back(TREEpoints[parent]);
					PATHpointsIndexes.push_back(parent);
					parent=TREEparentsIndexes[parent];
				}


				PATHpoints.push_back(TREEpoints[parent]);
				PATHpointsIndexes.push_back(parent);

				PATHpointsList.push_back(PATHpoints);
				PATHpointsIndexesList.push_back(PATHpointsIndexes);


				solutionFound=true;
				GOALindex=TREEpoints.size()-1;
				PATHcosts.push_back(finalCost);
				PATHtimes.push_back(simGetSimulationTime());
				simAddStatusbarMessage("Solution found!");


				solutionUpdated=true;
			}
			else if(solutionFound)
			{
				PATHpoints.clear();
				PATHpointsIndexes.clear();


				int parent=GOALindex;
				float newCost =0;
				//other points of the path
				while(parent!=rootIndex)
				{
					newCost += TREEdistancesFromParent[parent];
					PATHpoints.push_back(TREEpoints[parent]);
					PATHpointsIndexes.push_back(parent);
					parent=TREEparentsIndexes[parent];
				}
				PATHpoints.push_back(TREEpoints[parent]);
				PATHpointsIndexes.push_back(parent);

				PATHpointsList.push_back(PATHpoints);
				PATHpointsIndexesList.push_back(PATHpointsIndexes);

				if(newCost!=finalCost)
				{
					finalCost=newCost;
					PATHcosts.push_back(finalCost);
					PATHtimes.push_back(simGetSimulationTime());

					simAddStatusbarMessage("Path improved");
					solutionUpdated=true;
				}
			}
		}
	

		simFloat pRoot[3];
		pRoot[0]=ROOT(0);
		pRoot[1]=ROOT(1);
		pRoot[2]=height;

		simFloat oRoot[3];
		oRoot[0]=rotationAlpha;
		oRoot[1]=ROOT(2);
		oRoot[2]=rotationGamma;

		simSetObjectPosition(hCube, -1, pRoot );
		simSetObjectOrientation(hCube, -1, oRoot );

	

		if( simGetSimulationTime()>30)
		{
			RRT_IsRunning=false;	//stop the RRT* algorithm
			timeBudgetReached=true;
			pthread_create(&planning_thread, NULL, planningThread, (void *)1);

		}

	}
}




void Execution()
{
	if(RRT>=1)
		prunePointsAndLinks();

	if (solutionFound & !goalReached & !pathInterpolation)
	{
		Eigen::Vector3f p_curr;		

		simFloat pYouBot[3];
		simFloat oYouBot[3];
		simInt hCube = simGetObjectHandle("youBot");


		if(i_path < path.size() & !pathInterpolation)
		{
			p_curr = path.at(i_path);

			pYouBot[0] = p_curr(0);
			pYouBot[1] = p_curr(1);
			pYouBot[2] = height;

			oYouBot[0] = rotationAlpha;
			oYouBot[1] = p_curr(2);
			oYouBot[2] = rotationGamma;

			simSetObjectPosition(hCube, -1, pYouBot);
			simSetObjectOrientation(hCube, -1, oYouBot);
			i_path++;
		}
		else 
		{


			if (rootIndex == GOALindex)
			{
				goalReached=true;
			}

			else if(!pathInterpolation )
			{	
				pathInterpolation=true;

				while(pathInterpolation)
				{
					if(pathInterpolation2)
						break;
				}
				plotPointsAndLinks();
				pathInterpolation2=false;
			}

		}
	}
}

// This is the plugin start routine (called just once, just after the plugin was loaded):

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt) {
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    GetModuleFileName(NULL, curDirAndFile, 1023);
    PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof (curDirAndFile));
#endif
    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp += "\\v_rep.dll";
#elif defined (__linux)
    temp += "/libv_rep.so";
#elif defined (__APPLE__)
    temp += "/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL) {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0) {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    simLockInterface(1);

    // Here you could handle various initializations
    // Here you could also register custom Lua functions or custom Lua constants
    // etc.

    return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):

VREP_DLLEXPORT void v_repEnd() {
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData) { // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 6 lines at the beginning and unchanged:
    simLockInterface(1);
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal = NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected) { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass) { // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched = ((flags & 64) != 0);

        if (instanceSwitched) {
            // React to an instance switch here!!
        }

        if (sceneContentChanged) { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled) { // The main script is about to be run (only called while a simulation is running (and not paused!))
		
    }

    if (message == sim_message_eventcallback_simulationabouttostart) { // Simulation is about to start
		
		
		//pthread_create(&planning_thread, NULL, planningThread, (void *)1);

    }

    if (message == sim_message_eventcallback_simulationended) { // Simulation just ended
		
		//pthread_join(planning_thread, NULL);
		std::cout << "Simulation Concluded" << std::endl;
	
    }

    if (message == sim_message_eventcallback_moduleopen) { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle) { // A script called simHandleModule (by default the main script). Is only called during simulation.
	
		if(timeBudgetReached)
		{
			Execution();
		}
		else if(!timeBudgetReached & !RRT_IsRunning)
		{
			Planning_init();
			//pthread_create(&planning_thread, NULL, planningThread, (void *)1);
			//pthread_join(planning_thread, NULL);
		}
		else
		{
			Planning();
			//pthread_join(planning_thread, 0);
			//Planning();
		}
		
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose) { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch) { // Here the user switched the scene. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)

    }

    if (message == sim_message_eventcallback_broadcast) { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message == sim_message_eventcallback_scenesave) { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag) { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }
   
	// Play button is pressed
    if (simGetSimulationState()==17){  
		
		
    }
	// Stop button is pressed
	else if(simGetSimulationState()==sim_simulation_stopped){    
		
		RRT_IsRunning=false;
		solutionFound=false;
		timeBudgetReached=false;
		pointsToBePlotted.clear();
		rewiredLinksToBePlotted.clear();
		rewiringIndexesPoint.clear();
	}  


    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    simLockInterface(0);
    return retVal;
}
