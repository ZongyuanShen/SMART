/*******************************************
* Author: Zongyuan Shen. 
* All Rights Reserved. 
*******************************************/
#include "SMART.h"

int main(int argc, char *argv[])
{
	int trialIndex = 1;
	int dynObsNum = 15;
	double dynObsSpeed = 4;
	double robotSpeed = 4;
	int sceneIndex = 1;

	vector<vector<double>> dynObsPosition
    {
        {16, 6},
        {26, 6},
        {16, 16},
		{26, 16},
		{16, 26},
		{26, 26},
		{6, 6},
		{6, 16},
		{6, 26},
		{6, 21},
		{16, 11},
		{26, 11},
		{6, 11},
		{16, 21},
		{26, 21}
    };
	ofstream outfile_dynObsInfo("dynObsInfo.txt");
	ofstream outfile_treeInfo("treeInfo.txt");
	ofstream outfile_pathInfo("pathInfo.txt");
	ofstream outfile_footPrintInfo("footPrintInfo.txt");
	
	std::cout<<"Trial index = "<<trialIndex<<endl;
	std::cout<<"obstacle speed = "<<dynObsSpeed<<endl;
	std::cout<<"scene index = "<<sceneIndex<<endl;
	std::cout<<"dynObsNum = "<<dynObsNum<<endl;
	SMART SMART(trialIndex, dynObsSpeed, robotSpeed, sceneIndex, dynObsNum, dynObsPosition);
	while(true)
	{
		SMART.mainFunction();
		if (SMART.simulationSuccess == true) break;
		else if (SMART.simulationFail == true) break;
	}
	if (outfile_dynObsInfo.is_open())
	{
		for(vector<vector<double> >::size_type row = 0; row < SMART.dynObsInfo.size(); row++)
		{
			for (vector<double>::size_type column = 0; column < SMART.dynObsInfo[row].size(); column++)
			{
				outfile_dynObsInfo<<SMART.dynObsInfo[row][column]<<' ';
			}
			outfile_dynObsInfo<<endl;
		}
	}
	outfile_dynObsInfo.close();

	if (outfile_treeInfo.is_open())
	{
		for(vector<vector<double> >::size_type row = 0; row < SMART.treeInfo.size(); row++)
		{
			for (vector<double>::size_type column = 0; column < SMART.treeInfo[row].size(); column++)
			{
				outfile_treeInfo<<SMART.treeInfo[row][column]<<' ';
			}
			outfile_treeInfo<<endl;
		}
	}
	outfile_treeInfo.close();

	if (outfile_pathInfo.is_open())
	{
		for(vector<vector<double> >::size_type row = 0; row < SMART.pathInfo.size(); row++)
		{
			for (vector<double>::size_type column = 0; column < SMART.pathInfo[row].size(); column++)
			{
				outfile_pathInfo<<SMART.pathInfo[row][column]<<' ';
			}
			outfile_pathInfo<<endl;
		}
	}
	outfile_pathInfo.close();

	if (outfile_footPrintInfo.is_open())
	{
		for(vector<vector<double> >::size_type row = 0; row < SMART.footPrintInfo.size(); row++)
		{
			for (vector<double>::size_type column = 0; column < SMART.footPrintInfo[row].size(); column++)
			{
				outfile_footPrintInfo<<SMART.footPrintInfo[row][column]<<' ';
			}
			outfile_footPrintInfo<<endl;
		}
	}
	outfile_footPrintInfo.close();	
	return 0;
}