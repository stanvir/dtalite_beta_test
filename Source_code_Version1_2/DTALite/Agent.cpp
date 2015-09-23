#include "stdafx.h"
#include <stdlib.h>
#include <crtdbg.h>

#include "DTALite.h"


#include "Geometry.h"
#include "GlobalData.h"
#include "CSVParser.h"
#include "SafetyPlanning.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

using namespace std;

void g_AddVehicleID2ListBasedonDepartureTime(DTAVehicle * pVehicle)
{

	if (pVehicle->m_AgentID == 19)
	{

		TRACE("");
	}
	int simulation_time_no = (int)(pVehicle->m_DepartureTime * 10);
	g_VehicleTDListMap[simulation_time_no].m_AgentIDVector.push_back(pVehicle->m_AgentID);


}
void g_AllocateDynamicArrayForVehicles()
{
	if (g_TDOVehicleArray == NULL)  // has not allocated memory yet
	{
		_proxy_ABM_log(0, "Allocate memory for %d zones and %d SP calculation intervals.\n",
			
			g_ZoneMap.size(), g_NumberOfSPCalculationPeriods);

		g_TDOVehicleArray = AllocateDynamicArray<VehicleArrayForOriginDepartrureTimeInterval>(g_ZoneMap.size(), g_NumberOfSPCalculationPeriods);

	}
}

vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}


vector<float> ParseLineToFloat(string line)
{
	vector<float> SeperatedValues;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedValues.push_back(integer);
	}
	return SeperatedValues;
}

void g_ReadDSPVehicleFile(string file_name)
{
	if (g_TrafficFlowModelFlag == tfm_BPR)  //BRP  // static assignment parameters
	{
		g_AggregationTimetInterval = 60;
	}

	g_AllocateDynamicArrayForVehicles();


	FILE* st = NULL;

	fopen_s(&st, file_name.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << file_name << " ..." << endl;
		g_LogFile << "Reading file " << file_name << endl;
		int count = 0;

		//# of vehicles in the file 
		g_read_integer(st);
		// Max # of stops 
		g_read_integer(st);
		float total_number_of_vehicles_to_be_generated = 0;

		int i = 0;
		int line_no = 1;
		while (true)
		{

			line_no += 2;

			if (line_no % 1000 == 0)
				cout << "loading " << line_no / 1000 << " k lines" << endl;

			// #
			int agent_id = g_read_integer(st);

			if (agent_id < 0)
				break;

			DTAVehicle* pVehicle = 0;

			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			pVehicle->m_AgentID = i;
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			// 	usec 

			int origin_node_number = g_read_integer(st);
			pVehicle->m_OriginNodeID = g_NodeNametoIDMap[origin_node_number];

			//dsec  

			g_read_integer(st);

			//stime 

			pVehicle->m_DepartureTime = g_read_float(st);


			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: agent " << agent_id << " in file " << file_name << " has a departure time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please check!";
				g_ProgramStop();
			}



			//vehicle class
			pVehicle->m_DemandType = g_read_integer(st);

			pVehicle->m_DemandType = pVehicle->m_DemandType;
			//vehicle type 

			pVehicle->m_VehicleType = g_read_integer(st);
			//information class  
			pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType - 1].PCE;

			pVehicle->m_InformationClass = g_read_integer(st);


			//#ONode 
			g_read_integer(st);
			// #IntDe 
			g_read_integer(st);
			//info 
			g_read_integer(st);

			//ribf  
			g_read_float(st);
			//comp 
			g_read_float(st);
			//izone 

			pVehicle->m_OriginZoneID = g_read_integer(st);

			//Evac
			float evac_value = g_read_float(st);
			//InitPos 
			g_read_float(st);
			//VoT 
			pVehicle->m_VOT = g_read_float(st);
			//tFlag 
			g_read_float(st);
			//pArrTime 
			float PATvalue = g_read_float(st);
			//TP 
			float TP_value = g_read_float(st);
			//IniGas
			float value = g_read_float(st);

			pVehicle->m_DestinationZoneID = g_read_integer(st);




			// stop time?
			float travel_time_value = g_read_float(st);


			pVehicle->m_DestinationNodeID = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_AgentID % 100) / 100.0f);;

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}


			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;

			pVehicle->m_NodeSize = 0;

			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;

			if (pVehicle->m_OriginZoneID == pVehicle->m_DestinationZoneID)
			{  // do not simulate intra zone traffic
				continue;
			}
			if (g_DemandGlobalMultiplier<0.9999)
			{
				double random_value = g_GetRandomRatio();
				if (random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				{

					delete pVehicle;
					continue;
				}
			}

			g_VehicleVector.push_back(pVehicle);
			g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
			g_VehicleMap[i] = pVehicle;

			int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

			g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);


			i++;

		}

	}
	else
	{
		cout << "File " << file_name << " cannot be opened. Please check." << endl;
		g_ProgramStop();

	}

}

bool AddPathToVehicle(DTAVehicle * pVehicle, std::vector<int> path_node_sequence, CString FileName)
{

	if (pVehicle->m_NodeSize >= 1 && pVehicle->m_LinkAry != NULL)
	{
		delete pVehicle->m_LinkAry;
	}

	pVehicle->m_NodeSize = path_node_sequence.size();

	if (pVehicle->m_NodeSize >= 1)  // in case reading error
	{
		pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];
		pVehicle->m_NodeNumberSum = 0;

		pVehicle->m_Distance = 0;  // reset distanace when there are new paths assigned. 
		for (int i = 0; i < pVehicle->m_NodeSize; i++)
		{

			int node_id;
			float event_time_stamp, travel_time, emissions;

			pVehicle->m_NodeNumberSum += path_node_sequence[i];

			if (i == 0)
				pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

			if (i == pVehicle->m_NodeSize - 1)
				pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

			if (i >= 1)
			{
				DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
				if (pLink == NULL && FileName.GetLength() > 0)
				{
					CString msg;
					msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], pVehicle->m_AgentID, FileName);
					cout << msg << endl;

					return false;
				}

				pVehicle->m_Distance += pLink->m_Length;

				pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
			}


		}

	}
	return true;
}


void g_UseExternalPath(DTAVehicle* pVehicle)
{

	if (g_ODPathSetVector == NULL)
		return;

	int OrgZoneSequentialNo = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
	int DestZoneSequentialNo = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

	float random_value = pVehicle->GetRandomRatio();

	int information_type = pVehicle->m_InformationClass;

	if (information_type >= 2) // for enroute and pretrip infor users, we do not have information yet, so we default their paths to the learning from the previous day
		information_type = 1; 

	// loop through the path set
	if (g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size() >= 1)
	{
		int i = 0;
		for (i = 0; i < g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size(); i++)
		{

			if (random_value <= g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].CumulativeRatio)
			{
				break;
			}

		}

		if (i < 0)
			i = 0;

		if (i == g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size())
			i = g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet.size() - 1;

		AddPathToVehicle(pVehicle, g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo][information_type].PathSet[i].m_NodeNumberArray, NULL);

	}

}

bool g_ReadTripCSVFile(string file_name, bool bOutputLogFlag)
{
	int	LineCount = 0;

	bool bOutputDebugLogFile = true;

	bool bUpdatePath = false;

	g_AllocateDynamicArrayForVehicles();
	float start_time_value = -100;

	CCSVParser parser_agent;

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name, false))
	{

		if (bOutputLogFlag)
		{
			cout << "reading file " << file_name << endl;

		}
		if (bOutputDebugLogFile)
			fprintf(g_DebugLogFile, "reading file %s\n", file_name.c_str());

		int line_no = 1;

		int i = 0;

		int count = 0;

		int count_for_sameOD = 0;
		int count_for_not_defined_zones = 0;

		while (parser_agent.ReadRecord())
		{

			if ((count + 1) % 1000 == 0 && bOutputLogFlag)
			{
				cout << "reading " << count + 1 << " records..." << endl;

			}
			count++;

			int agent_id = 0;

			parser_agent.GetValueByFieldNameRequired("agent_id", agent_id);

			_proxy_ABM_log(0, "--step 1: read agent id = %d \n", agent_id);

			DTAVehicle* pVehicle = 0;

			bool bCreateNewAgent = false;

			if (g_VehicleMap.find(agent_id) != g_VehicleMap.end())
			{
				_proxy_ABM_log(0, "--step 2: agent id =%d found in memory, will update data\n", agent_id);

				pVehicle = g_VehicleMap[agent_id];
				bCreateNewAgent = false;
			}
			else
			{

				pVehicle = new (std::nothrow) DTAVehicle;
				if (pVehicle == NULL)
				{
					cout << "Insufficient memory...";
					getchar();
					exit(0);

				}
				pVehicle->m_AgentID = agent_id;
				_proxy_ABM_log(0, "--step 2: new agent id = %d, the current size of agent vector= %d \n", agent_id, g_VehicleVector.size());

				bCreateNewAgent = true;

			}

			
			// additional error checking for updating agent data
			int ExternalTourID = 0;
			parser_agent.GetValueByFieldNameRequired("tour_id", ExternalTourID);

			_proxy_ABM_log(0, "--step 3: read tour_id id = %d \n", ExternalTourID);

			if (ExternalTourID >= 0)
			{
				pVehicle->m_ExternalTourID = ExternalTourID;
			}


			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			int from_zone_id = pVehicle->m_OriginZoneID;
			int to_zone_id = pVehicle->m_DestinationZoneID;

			parser_agent.GetValueByFieldNameRequired("from_zone_id", from_zone_id);
			parser_agent.GetValueByFieldNameRequired("to_zone_id", to_zone_id);

			if (pVehicle->m_OriginZoneID == -1)  // new vehicle
				pVehicle->m_OriginZoneID = from_zone_id;

			if (pVehicle->m_DestinationZoneID == -1) //new vehicle
				pVehicle->m_DestinationZoneID = to_zone_id;


			_proxy_ABM_log(0, "--step 4: read from_zone_id = %d, to_zone_id=%d \n", from_zone_id, to_zone_id);

			if (g_ZoneMap.find(from_zone_id) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;
				_proxy_ABM_log(0, "--step 4.1: from_zone_id = %d not defined, exit.\n", from_zone_id);

				continue;
			}

			if (g_ZoneMap.find(to_zone_id) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;
				_proxy_ABM_log(0, "--step 4.1: to_zone_id=%d not defined, exit\n", to_zone_id);

				continue;
			}
			// to do: update origin only when vehicle has not departed yet
			if (pVehicle->m_OriginZoneID != from_zone_id)
			{
				g_LogFile << " UPDATE Agent Data: origin zone =  " << pVehicle->m_OriginZoneID << "-> " << from_zone_id << endl;
			
				_proxy_ABM_log(0, "--step 4.2: update from_zone_id = %d->%d \n", pVehicle->m_OriginZoneID, from_zone_id);
				pVehicle->m_OriginZoneID = from_zone_id;
			}

			//to do: update destination only when vehicle has not reached the destination
			if (pVehicle->m_DestinationZoneID != to_zone_id)
			{
				g_LogFile << " UPDATE Agent Data: destination zone =  " << pVehicle->m_DestinationZoneID << "-> " << to_zone_id << endl;
				_proxy_ABM_log(0, "--step 4.2: update to_zone_id = %d->%d \n", pVehicle->m_DestinationZoneID, to_zone_id);
				pVehicle->m_DestinationZoneID = to_zone_id;
			}
			int origin_node_id = -1;
			int origin_node_number = -1;

			parser_agent.GetValueByFieldNameRequired("from_origin_node_id", origin_node_number);

			if (g_NodeNametoIDMap.find(origin_node_number) != g_NodeNametoIDMap.end())  // convert node number to internal node id
			{
				origin_node_id = g_NodeNametoIDMap[origin_node_number];
			}

			int destination_node_id = -1;
			int destination_node_number = -1;
			parser_agent.GetValueByFieldNameRequired("to_destination_node_id", destination_node_number);

			if (g_NodeNametoIDMap.find(destination_node_number) != g_NodeNametoIDMap.end()) // convert node number to internal node id
			{
				destination_node_id = g_NodeNametoIDMap[destination_node_number];
			}

			_proxy_ABM_log(0, "--step 5: read origin_node_id = %d, destination_node_id=%d \n", 
				origin_node_number, destination_node_number);

			if (origin_node_id == -1)  // no default origin node value, re-generate origin node
			{
				origin_node_id = g_ZoneMap[pVehicle->m_OriginZoneID].GetRandomOriginNodeIDInZone((pVehicle->m_AgentID % 100) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily
			}
			if (destination_node_id == -1)// no default destination node value, re-destination origin node
				destination_node_id = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_AgentID % 100) / 100.0f);

			if (pVehicle->m_OriginNodeID !=-1 && pVehicle->m_OriginNodeID != origin_node_id)
			{
				g_LogFile << " UPDATE Agent Data: origin node =  " << pVehicle->m_OriginNodeID << "-> " << origin_node_id << endl;


				_proxy_ABM_log(0, "--step 5.2 update origin_node_id = %d->%d \n",
					pVehicle->m_OriginNodeID, origin_node_id);

				bUpdatePath = true;
			}


			if (pVehicle->m_DestinationNodeID != -1 && pVehicle->m_DestinationNodeID != destination_node_id)
			{
				g_LogFile << " UPDATE Agent Data: destination node =  " << pVehicle->m_DestinationNodeID << "-> " << destination_node_id << endl;

				_proxy_ABM_log(0, "--step 5.2: update destination_node_id = %d->%d \n",
					pVehicle->m_DestinationNodeID, destination_node_id);

				bUpdatePath = true;

			}

			// for input or update data or not, we all reset the origin_node_id and destination_node_id
			pVehicle->m_OriginNodeID = origin_node_id;
			pVehicle->m_DestinationNodeID = destination_node_id;




			if (origin_node_id == destination_node_id)
			{  // do not simulate intra zone traffic
				_proxy_ABM_log(0, "--step 5.3: found intrazone traffic %d->%d, not simulated\n",
					origin_node_id, destination_node_id);

				count_for_sameOD++;
				continue;
			}

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}

			float departure_time = 0;
			if (parser_agent.GetValueByFieldNameRequired("departure_time_in_min", departure_time) == true)
			{
				_proxy_ABM_log(0, "--step 6: read departure_time = %.2f\n",
					departure_time);

				if (start_time_value < 0)  // set first value
					start_time_value = departure_time;
				else if (start_time_value > departure_time + 0.00001)  // check if the departure times are sequential
				{
					departure_time = start_time_value; // use a larger value 
					start_time_value = departure_time;
				}

				if (pVehicle->m_DepartureTime <= -1)
				{
					pVehicle->m_DepartureTime = departure_time;  // new vehicle

				}
				else if (fabs(pVehicle->m_DepartureTime - departure_time) > 0.5)
				{
					g_LogFile << " UPDATE Agent Data: departure time=  " << pVehicle->m_DepartureTime << "-> " << departure_time << endl;
					_proxy_ABM_log(0, "--step 6.2: update departure_time = %.2f->%.2f\n",
						pVehicle->m_DepartureTime , departure_time);

					bUpdatePath = true;

					//remove vehicle id for the old departure time slot

					int int_to_remove = pVehicle->m_AgentID;
					g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.erase(std::remove(g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.begin(), g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.end(), int_to_remove), g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.end());


					//add vehicle id for the new departure time slot
					g_VehicleTDListMap[departure_time * 10].m_AgentIDVector.push_back(pVehicle->m_AgentID);

					pVehicle->m_DepartureTime = departure_time;
					pVehicle->m_PreferredDepartureTime = departure_time;


				}

			}






			int beginning_departure_time = departure_time;

			ASSERT(pVehicle->m_DepartureTime < 4000);

			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: agent_id " << agent_id << " in file " << file_name << " has a start time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt";
				g_ProgramStop();
			}

			int demand_type = pVehicle->m_DemandType;
			if (parser_agent.GetValueByFieldName("demand_type", demand_type) == true)
			{
				if (pVehicle->m_DemandType != demand_type)
					pVehicle->m_DemandType = demand_type;

				_proxy_ABM_log(0, "--step 7: read demand_type = %d\n",
					demand_type);

				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);

				// if there are values in the file, then update the related attributes; 
				int VOT = 0;
				int DemandType = 0;
				int VehicleType = 0;



				parser_agent.GetValueByFieldNameRequired("vehicle_type", VehicleType);

				_proxy_ABM_log(0, "--step 8: read vehicle_type = %d\n",
					VehicleType);

				if (VehicleType >= 1)
				{

					pVehicle->m_VehicleType = VehicleType;
				}

				int information_type = pVehicle->m_InformationClass;

				parser_agent.GetValueByFieldNameRequired("information_type", information_type); //default is 0;

				_proxy_ABM_log(0, "--step 8: read information_type = %d\n",
					information_type);
				if (information_type != pVehicle->m_InformationClass)
				{
					_proxy_ABM_log(0, "--step 8.2: update information_type = %d->%d\n",
						pVehicle->m_InformationClass, information_type);

					g_LogFile << " UPDATE Agent Data: information type =  " << pVehicle->m_InformationClass << "-> " << information_type << endl;

				}

				if (pVehicle->m_InformationClass >= 3)  // enroute info
				{

					double time_to_start_information_retrieval = -1.0;
					parser_agent.GetValueByFieldName("time_to_start_information_retrieval", time_to_start_information_retrieval); //default is -1;

					if (time_to_start_information_retrieval >= 0)
					{
						pVehicle->m_TimeToRetrieveInfo = time_to_start_information_retrieval;
					}

					pVehicle->m_EnrouteInformationUpdatingTimeIntervalInMin = g_information_updating_interval_in_min;

					double information_updating_interval_in_min = -1.0;
					parser_agent.GetValueByFieldName("information_updating_interval_in_min", information_updating_interval_in_min); //default is -1;

					if (information_updating_interval_in_min >= 0)
					{
						pVehicle->m_EnrouteInformationUpdatingTimeIntervalInMin = information_updating_interval_in_min;
					}

				}

				parser_agent.GetValueByFieldNameRequired("value_of_time", VOT);

				_proxy_ABM_log(0, "--step 9: read value_of_time = %d\n", VOT);


				if (VOT >= 1)  // only with valid value
					pVehicle->m_VOT = VOT;

				parser_agent.GetValueByFieldNameRequired("vehicle_age", pVehicle->m_Age);

				_proxy_ABM_log(0, "--step 10: read vehicle_age = %d\n", pVehicle->m_Age);

			}
			else
			{


			}

			if (bCreateNewAgent == true)
			{

				pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
				pVehicle->m_ArrivalTime = 0;
				pVehicle->m_bComplete = false;
				pVehicle->m_bLoaded = false;
				pVehicle->m_TollDollarCost = 0;
				pVehicle->m_Emissions = 0;
				pVehicle->m_Distance = 0;

				pVehicle->m_NodeSize = 0;

				pVehicle->m_NodeNumberSum = 0;
				pVehicle->m_Distance = 0;

			}

			std::vector<int> path_node_sequence;
				string path_node_sequence_str;
				parser_agent.GetValueByFieldNameRequired("path_node_sequence", path_node_sequence_str);

				path_node_sequence = ParseLineToIntegers(path_node_sequence_str);
				if (path_node_sequence.size() >= 2)
				{
					_proxy_ABM_log(0, "--step 11: read path_node_sequence = %s\n", path_node_sequence_str.c_str());
					AddPathToVehicle(pVehicle, path_node_sequence, file_name.c_str());

				}
				else
				{
					_proxy_ABM_log(0, "--step 11: no input for path_node_sequence, create shortest path\n");

				}


			std::vector<int> detour_node_sequence;
			string detour_node_sequence_str;
			if (parser_agent.GetValueByFieldName("detour_node_sequence", detour_node_sequence_str) == true)
			{


				detour_node_sequence = ParseLineToIntegers(detour_node_sequence_str);

				g_UpdateAgentPathBasedOnDetour(pVehicle->m_AgentID, detour_node_sequence);
			}

			if (bUpdatePath)
			{
				g_UpdateAgentPathBasedOnNewDestinationOrDepartureTime(pVehicle->m_AgentID);

			}


			int number_of_agents = 1;

			float ending_departure_time = 0;

			if (bCreateNewAgent == true)
			{

				g_VehicleVector.push_back(pVehicle);

				if (pVehicle->m_AgentID == 19)
				{
					TRACE("");

				}
				g_VehicleTDListMap[(int)pVehicle->m_DepartureTime * 10].m_AgentIDVector.push_back(pVehicle->m_AgentID);

				if (bOutputDebugLogFile)
				{
					fprintf(g_DebugLogFile, "adding vehicle: total size =%d\n", g_VehicleVector.size());
				}
				g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

				int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

				ASSERT(pVehicle->m_OriginZoneID <= g_ODZoneNumberSize);

				g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);

			}
			i++;
		}


		line_no++;



		if (bOutputLogFlag)
		{

			cout << count << " records have been read from file " << file_name << endl;

			cout << i << " agents have been read from file " << file_name << endl;

			if (count_for_sameOD >= 1)
				cout << "there are " << count_for_sameOD << " agents with the same from_zone_id and to_zone_id, which will not be simulated. " << endl;


			if (count_for_not_defined_zones >= 1)
				cout << "there are " << count_for_not_defined_zones << " agents with zones not being defined in input_zone.csv file, which will not be simulated. " << endl;

		}
		LineCount = count;
	}
	else
	{
		cout << "Waiting for file " << file_name << "... " << endl;

		return false;
	}

	return true;

}
bool g_ReadTRANSIMSTripFile(string file_name, bool bOutputLogFlag)
{
	g_AllocateDynamicArrayForVehicles();

	CCSVParser parser_agent;


	parser_agent.Delimiter = '\t';

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name, false))
	{

		if (bOutputLogFlag)
		{
			cout << "reading file " << file_name << endl;
		}
		int line_no = 1;

		int i = 0;

		int count = 0;

		int count_for_sameOD = 0;
		int count_for_not_defined_zones = 0;

		while (parser_agent.ReadRecord())
		{

			if ((count + 1) % 1000 == 0 && bOutputLogFlag)
			{
				cout << "reading " << count + 1 << " records..." << endl;

			}
			count++;

			if (g_DemandGlobalMultiplier<0.9999)
			{
				double random_value = g_GetRandomRatio();
				if (random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				{

					continue;
				}
			}

			int trip_id = 0;

			DTA_vhc_simple vhc;

			parser_agent.GetValueByFieldNameRequired("ORIGIN", vhc.m_OriginZoneID);
			parser_agent.GetValueByFieldNameRequired("DESTINATION", vhc.m_DestinationZoneID);

			if (g_ZoneMap.find(vhc.m_OriginZoneID) == g_ZoneMap.end() || g_ZoneMap.find(vhc.m_DestinationZoneID) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;

				continue;
			}



			if (vhc.m_OriginZoneID == vhc.m_DestinationZoneID)
			{  // do not simulate intra zone traffic

				count_for_sameOD++;
				continue;
			}

			if (g_ZoneMap.find(vhc.m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[vhc.m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[vhc.m_OriginZoneID].m_OriginVehicleSize += 1;

			}


			std::string START_TIME;
			parser_agent.GetValueByFieldNameRequired("START", START_TIME);



			if (START_TIME.find(":") != std::string::npos)
			{
				int hour, min, second;
				sscanf(START_TIME.c_str(), "%d:%d:%d", &hour, &min, &second);
				vhc.m_DepartureTime = hour * 60 + min + second / 60.0;
			}
			else
			{
				float min;

				sscanf(START_TIME.c_str(), "%f", &min);

				vhc.m_DepartureTime = min;

			}


			if (vhc.m_DepartureTime < g_DemandLoadingStartTimeInMin || vhc.m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				//cout << "Error: trip_id " <<  trip_id << " in file " << file_name << " has a start time of " << vhc.m_DepartureTimeIndex  << ", which is out of the demand loading range: " << 
				//	g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt" ;

				continue;
			}

			//parser_agent.GetValueByFieldName("demand_type",pVehicle->m_DemandType);

			//parser_agent.GetValueByFieldName("vehicle_type",pVehicle->m_VehicleType);
			//parser_agent.GetValueByFieldName("information_type",pVehicle->m_InformationClass);
			//parser_agent.GetValueByFieldName("value_of_time",pVehicle->m_VOT);
			//parser_agent.GetValueByFieldName("vehicle_age",pVehicle->m_Age );


			/*		int number_of_nodes = 0;
			parser_agent.GetValueByFieldName("number_of_nodes",number_of_nodes );

			std::vector<int> path_node_sequence;
			if(number_of_nodes >=2)
			{
			string path_node_sequence_str;
			parser_agent.GetValueByFieldName("path_node_sequence",path_node_sequence_str);

			path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

			AddPathToVehicle(pVehicle, path_node_sequence,file_name.c_str ());
			}*/

			//pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			//pVehicle->m_ArrivalTime  = 0;
			//pVehicle->m_bComplete = false;
			//pVehicle->m_bLoaded  = false;
			//pVehicle->m_TollDollarCost = 0;
			//pVehicle->m_Emissions  = 0;
			//pVehicle->m_Distance = 0;

			//pVehicle->m_NodeSize = 0;

			//pVehicle->m_NodeNumberSum =0;
			//pVehicle->m_Distance =0;

			line_no++;

			//

			int demand_type = 1;
			vhc.m_DemandType = demand_type;


			g_GetVehicleAttributes(vhc.m_DemandType, vhc.m_VehicleType, vhc.m_InformationClass, vhc.m_VOT, vhc.m_Age);

			g_simple_vector_vehicles.push_back(vhc);

			//// debug info
			//if(g_simple_vector_vehicles.size() == 100000)
			//	break;



		}
		if (bOutputLogFlag)
		{

			cout << count << " records have been read from file " << file_name << endl;

			cout << i << " agents have been read from file " << file_name << endl;

			if (count_for_sameOD >= 1)
				cout << "there are " << count_for_sameOD << " agents with the same from_zone_id and to_zone_id, which will not be simulated. " << endl;


			if (count_for_not_defined_zones >= 1)
				cout << "there are " << count_for_not_defined_zones << " agents with zones not being defined in input_zone.csv file, which will not be simulated. " << endl;

		}

	}
	else
	{
		cout << "Waiting for file " << file_name << "... " << endl;

		return false;
	}

	return true;

}


void g_ReadScenarioFilesUnderAgentBinaryMode()
{
	CCSVParser parser_demand_type;

	if (parser_demand_type.OpenCSVFile("Scenario_Demand_Type.csv", false))
	{
		g_DemandTypeVector.clear();

		float cumulative_demand_type_percentage = 0;

		while (parser_demand_type.ReadRecord())
		{


			DemandType element;
			int demand_type = 1;
			float average_VOT = 10;

			if (parser_demand_type.GetValueByFieldName("demand_type", demand_type) == false)
				break;

			float demand_type_percentage = 0;
			if (parser_demand_type.GetValueByFieldName("demand_type_percentage", element.demand_type_percentage) == false)
			{
				cout << "Field demand_type_percentage is missing in Scenario_Demand_Type.csv. Please check." << endl;
				g_ProgramStop();
				break;

			}


			if (parser_demand_type.GetValueByFieldName("vehicle_trip_multiplier_factor", element.vehicle_trip_multiplier_factor) == false)
			{
				cout << "Field vehicle_trip_multiplier_factor is missing in Scenario_Demand_Type.csv. Please check." << endl;
				g_ProgramStop();
				break;

			}




			cumulative_demand_type_percentage += element.demand_type_percentage;

			element.cumulative_demand_type_percentage = cumulative_demand_type_percentage;

			float ratio_pretrip = 0;
			float ratio_enroute = 0;
			float ratio_personalized_info = 0;
			float ratio_eco_so_info = 0;

			parser_demand_type.GetValueByFieldName("percentage_of_pretrip_info", ratio_pretrip);
			parser_demand_type.GetValueByFieldName("percentage_of_enroute_info", ratio_enroute);
			parser_demand_type.GetValueByFieldName("percentage_of_personalized_info", ratio_personalized_info);
			parser_demand_type.GetValueByFieldName("percentage_of_eco_so_info", ratio_eco_so_info);

			if (ratio_eco_so_info >= 1)
			{
				g_EmissionDataOutputFlag = 2;  //enable emission output at each iteration

			}

			element.demand_type = demand_type;

			parser_demand_type.GetValueByFieldName("demand_type_name", element.demand_type_name);
			element.info_class_percentage[1] = 0;  //learning 
			element.info_class_percentage[2] = ratio_pretrip;
			element.info_class_percentage[3] = ratio_enroute;
			element.info_class_percentage[4] = ratio_personalized_info;
			element.info_class_percentage[5] = ratio_eco_so_info;


			if (ratio_pretrip > 0 || ratio_enroute > 0 )
			{
				if (g_NumberOfIterations >= 1)
				{
					cout << "Please use 1 iteration when pre-trip and en route information is enabled." << endl;
					g_ProgramStop();
				}
			}

			element.info_class_percentage[0] = 100 - ratio_enroute - ratio_pretrip - ratio_personalized_info - ratio_eco_so_info;

			for (int ic = 0; ic < MAX_INFO_CLASS_SIZE; ic++)
			{
				element.cumulative_info_class_percentage[ic] = element.cumulative_info_class_percentage[ic - 1] + element.info_class_percentage[ic];
			}
			for (int i = 0; i < g_VehicleTypeVector.size(); i++)
			{
				std::ostringstream  str_percentage_of_vehicle_type;
				str_percentage_of_vehicle_type << "percentage_of_vehicle_type" << i + 1;

				float percentage_vehicle_type = 0;
				if (parser_demand_type.GetValueByFieldName(str_percentage_of_vehicle_type.str(), percentage_vehicle_type) == false)
				{
					cout << "Error: Field percentage_of_vehicle_type " << i + 1 << " cannot be found in the Scenario_Demand_Type.csv file.";
					cout << "In file Scenario_Vehicle_Type.csv, " << g_VehicleTypeVector.size() << " have been defined, so Scenario_Demand_Type.csv should percentage_of_vehicle_type for all vehicle types.";


					g_ProgramStop();
					return;
				}
				else
				{
					element.vehicle_type_percentage[i + 1] = percentage_vehicle_type;

					element.cumulative_type_percentage[i + 1] = element.cumulative_type_percentage[i] + percentage_vehicle_type;

				}
			}


			g_DemandTypeVector.push_back(element);

		}

		if (cumulative_demand_type_percentage >= 99 && cumulative_demand_type_percentage <= 101)
			cumulative_demand_type_percentage = 100;
		else
		{
			cout << "Error: Sum of demand_type_percentage =  " << cumulative_demand_type_percentage << " which should be 100 in the Scenario_Demand_Type.csv file.";

		}

	}
	else
	{

		cout << "Error: File Scenario_Demand_Type.csv cannot be opened.\nThis file is required when setting format_type = agent_bin_with_updated_demand_vehicle_type_info in file input_demand_meta_data.csv." << endl;
		g_ProgramStop();
	}


	// reading updated vehicle type 

	CCSVParser parser_vehicle_type;

	if (parser_vehicle_type.OpenCSVFile("Scenario_Vehicle_Type.csv", false))
	{
		g_VehicleTypeVector.clear();
		while (parser_vehicle_type.ReadRecord())
		{
			int vehicle_type = 0;
			if (parser_vehicle_type.GetValueByFieldName("vehicle_type", vehicle_type) == false)
				break;

			string vehicle_type_name;
			parser_vehicle_type.GetValueByFieldName("vehicle_type_name", vehicle_type_name);


			DTAVehicleType element;
			element.vehicle_type = vehicle_type;
			element.vehicle_type_name = vehicle_type_name;

			parser_vehicle_type.GetValueByFieldName("rolling_term_a", element.rollingTermA);
			parser_vehicle_type.GetValueByFieldName("rotating_term_b", element.rotatingTermB);
			parser_vehicle_type.GetValueByFieldName("drag_term_c", element.dragTermC);
			parser_vehicle_type.GetValueByFieldName("source_mass", element.sourceMass);


			float percentage_of_age = 0;

			int age = 0;

			// initialize age vector from 0 year to 30 year
			for (age = 0; age <= 30; age++)
			{
				element.percentage_age_vector.push_back(0);
			}

			for (age = 0; age <= 30; age++)
			{
				CString str_age;
				str_age.Format("percentage_of_age_%d", age);

				CT2CA pszConvertedAnsiString(str_age);
				// construct a std::string using the LPCSTR input
				std::string strStd(pszConvertedAnsiString);

				if (parser_vehicle_type.GetValueByFieldName(strStd, percentage_of_age) == true) // with data
				{
					element.percentage_age_vector[age] = percentage_of_age;

				}

			}


			g_VehicleTypeVector.push_back(element);

		}

	}
	else
	{
		cout << "Sceanrio_Vehicle_Type.csv cannot be opened.\nThis file is required when setting format_type = agent_bin_with_updated_demand_vehicle_type_info in file input_demand_meta_data.csv." << endl;
		g_ProgramStop();

	}

	// reading VOT

	CCSVParser parser_VOT;

	double cumulative_percentage = 0;

	if (parser_VOT.OpenCSVFile("Scenario_VOT.csv", false))
	{
		int i = 0;
		int old_demand_type = 0;
		while (parser_VOT.ReadRecord())
		{
			int demand_type = 0;

			if (parser_VOT.GetValueByFieldName("demand_type", demand_type) == false)
				break;

			if (demand_type != old_demand_type)
				cumulative_percentage = 0;   //switch vehicle type, reset cumulative percentage


			int VOT;
			if (parser_VOT.GetValueByFieldName("VOT_dollar_per_hour", VOT) == false)
				break;

			float percentage;
			if (parser_VOT.GetValueByFieldName("percentage", percentage) == false)
				break;

			old_demand_type = demand_type;
			VOTDistribution element;
			element.demand_type = demand_type;
			element.percentage = percentage;
			element.VOT = VOT;
			element.cumulative_percentage_LB = cumulative_percentage;
			cumulative_percentage += percentage;
			element.cumulative_percentage_UB = cumulative_percentage;

			g_VOTDistributionVector.push_back(element);

		}

	}
	else
	{
		cout << "Scenario_VOT.csv cannot be opened." << endl;
		g_ProgramStop();
	}

}
bool g_ReadAgentBinFile(string file_name, bool b_with_updated_demand_type_info)
{

	cout << "Reading Agent Bin File..." << endl;
	g_VehicleLoadingMode = vehicle_binary_file_mode;

	g_DetermineDemandLoadingPeriod();

	if (b_with_updated_demand_type_info)
	{
		g_ReadScenarioFilesUnderAgentBinaryMode();
	}


	int path_node_sequence[MAX_NODE_SIZE_IN_A_PATH];

	g_AllocateDynamicArrayForVehicles();

	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "rb");
	if (st != NULL)
	{
		struct_VehicleInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_VehicleInfo_Header), 1, st);

			if (header.vehicle_id < 0)
				break;

			if (header.vehicle_id == 28)
				TRACE("Vehicle ID = %d\n", header.vehicle_id);


			if (header.number_of_nodes != 12)
			{
				TRACE("");

			}

			if (result != 1)  // read end of file
				break;

			DTAVehicle* pVehicle = 0;
			//try
			//{
			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			//if(header.departure_time >= 420)
			//	break;

			////}
			////catch (std::bad_alloc& exc)
			////{
			////	cout << "Insufficient memory...";
			////	getchar();
			////	exit(0);

			////}

			pVehicle->m_AgentID = header.vehicle_id;
			pVehicle->m_RandomSeed = pVehicle->m_AgentID;

			pVehicle->m_OriginZoneID = header.from_zone_id;
			pVehicle->m_DestinationZoneID = header.to_zone_id;

			g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
			g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;


			pVehicle->m_DepartureTime = header.departure_time;

			if (g_DemandLoadingEndTimeInMin < pVehicle->m_DepartureTime)
				g_DemandLoadingEndTimeInMin = pVehicle->m_DepartureTime;

			if (g_DemandLoadingStartTimeInMin > pVehicle->m_DepartureTime)
				g_DemandLoadingStartTimeInMin = pVehicle->m_DepartureTime;

			pVehicle->m_PreferredDepartureTime = header.departure_time;
			pVehicle->m_ArrivalTime = header.arrival_time;

			pVehicle->m_TripTime = header.trip_time;

			pVehicle->m_DemandType = header.demand_type;


			if (pVehicle->m_DemandType == 0) // unknown type
				pVehicle->m_DemandType = 1;

			pVehicle->m_VehicleType = header.vehicle_type;
			pVehicle->m_PCE = g_VehicleTypeVector[pVehicle->m_VehicleType - 1].PCE;
			pVehicle->m_InformationClass = header.information_type;
			pVehicle->m_VOT = header.value_of_time;
			pVehicle->m_Age = header.age;


			//
			float vehicle_trip_multiplier_factor = 1.0f;

			if (b_with_updated_demand_type_info)
			{
				int demand_type = -1;
				double RandomPercentage = g_GetRandomRatio() * 100;

				double previous_cumulative_percentage = 0;

				for (int type_no = 0; type_no < g_DemandTypeVector.size(); type_no++)
				{


					double cumulative_percentage = g_DemandTypeVector[type_no].cumulative_demand_type_percentage;
					if (RandomPercentage >= previous_cumulative_percentage && RandomPercentage <= cumulative_percentage)
					{
						vehicle_trip_multiplier_factor = g_DemandTypeVector[type_no].vehicle_trip_multiplier_factor;
						demand_type = type_no;

						previous_cumulative_percentage = cumulative_percentage;

					}
				}

				//	TRACE("vehicle id = %d, demand type = %d\n ", header.vehicle_id, demand_type);

				if (demand_type == -1)
				{
					cout << "Error: demand_type = -1" << endl;
					g_ProgramStop();

				}

				pVehicle->m_DemandType = demand_type;

				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);


			}



			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;

			pVehicle->m_NodeSize = header.number_of_nodes;

			if (header.number_of_nodes >= 1999)

			{
				cout << "Error in reading agent file: header.number_of_node = " << header.number_of_nodes << endl;
				g_ProgramStop();
			}
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;
			pVehicle->m_NodeNumberSum = 0;

			int time_interval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

			if (g_ODEstimationFlag == 1) // having hist od only unde ODME mode
			{
				g_SystemDemand.AddValue(pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, time_interval, 1); // to store the initial table as hist database
			}

			if (pVehicle->m_NodeSize >= 1)  // in case reading error
			{
				pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];

				pVehicle->m_NodeNumberSum = 0;
				int i;
				for (i = 0; i < pVehicle->m_NodeSize; i++)
				{

					int node_id;
					float event_time_stamp, travel_time, emissions;

					struct_Vehicle_Node node_element;
					fread(&node_element, sizeof(node_element), 1, st);

					path_node_sequence[i] = node_element.NodeName;
					pVehicle->m_NodeNumberSum += path_node_sequence[i];

					if (i == 0)
						pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

					if (i == pVehicle->m_NodeSize - 1)
						pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

					if (i >= 1)
					{
						DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
						if (pLink == NULL)
						{
							CString msg;
							msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], header.vehicle_id, file_name.c_str());
							cout << msg << endl;
							continue;
						}

						if (pLink->GetNumberOfLanes() < 0.01)  // this is a blocked link by work zone
						{
							pVehicle->m_bForcedSwitchAtFirstIteration = true;

						}

						pVehicle->m_Distance += pLink->m_Length;

						pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
					}


				}


				//if(g_DemandGlobalMultiplier<0.9999)
				//{
				//	double random_value = g_GetRandomRatio();
				//	if(random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				//	{

				//		delete pVehicle;
				//		continue;
				//	}
				//}


				if (vehicle_trip_multiplier_factor < 0.9999 && b_with_updated_demand_type_info == true)  // we have to run a random number to decide if the vehicles should be added into the simulation or not.
				{

					double RandomRatio = g_GetRandomRatio();
					if (RandomRatio < vehicle_trip_multiplier_factor)
					{

						delete pVehicle;
						pVehicle = NULL;
						continue;  // do not proceed to the remaining steps

					}

				}
				if (i >= pVehicle->m_NodeSize)
				{

					g_VehicleVector.push_back(pVehicle);
					g_AddVehicleID2ListBasedonDepartureTime(pVehicle);
					g_VehicleMap[pVehicle->m_AgentID] = pVehicle;

					int AssignmentInterval = g_FindAssignmentIntervalIndexFromTime(pVehicle->m_DepartureTime);

					g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_AgentID);

					count++;
				}
				if (count % 10000 == 0)
					cout << "reading " << count / 1000 << "K agents from binary file " << file_name << endl;
			}
		}
		g_ResetVehicleAttributeUsingDemandType();

		if (g_use_global_path_set_flag == 1)
			g_BuildGlobalPathSet();


		fclose(st);
		return true;

	}
	else
	{
		cout << "File agent.bin cannot be found. Please check." << endl;
		g_ProgramStop();



	}
	return false;
}

void g_ResetVehicleType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;
		pVehicle->m_InformationClass = info_hist_based_on_routing_policy;
		double RandomPercentage = g_GetRandomRatio() * 100;
		for (int in = 0; in < MAX_INFO_CLASS_SIZE; in++)
		{
			int demand_type_no = pVehicle->m_DemandType - 1;

			if (RandomPercentage >= g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[in - 1] &&
				RandomPercentage < g_DemandTypeVector[demand_type_no].cumulative_info_class_percentage[in])
				pVehicle->m_InformationClass = in + 1; // return pretrip as 2 or enoute as 3
		}
	}

}

void g_ResetVehicleAttributeUsingDemandType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;

		g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);

	}
}


class DTAPathNodeSequence{
public:

	std::vector<int> m_node_sequence;

};

void g_AccessibilityMatrixGenerationForAllDemandTypes(string FileName, bool bTimeDependentFlag, double CurrentTime)
{

	CString file_name;
	file_name.Format(FileName.c_str());

	CString str_output_file_in_summary;
	str_output_file_in_summary.Format("Output file =,%s\n", file_name);
	g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);

	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int DemandLoadingStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int DemandLoadingEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		DemandLoadingStartTimeInMin = CurrentTime;
		DemandLoadingEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculate time interval " << DemandLoadingStartTimeInMin << " -> " << DemandLoadingEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;


	int total_demand_type = g_DemandTypeVector.size();
	cout << "------00---------" << endl;
	int StatisticsIntervalSize = max(1, (DemandLoadingEndTimeInMin - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval);

	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float**** ODTravelTime = NULL;
	//ODTravelTime = Allocate4DDynamicArray<float>(1000, 1000, 1000, 1000);
	ODTravelTime = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDistance = NULL;
	ODDistance = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDollarCost = NULL;
	ODDollarCost = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	DTAPathNodeSequence**** ODPathNodeSequence = NULL;
	ODPathNodeSequence = Allocate4DDynamicArray<DTAPathNodeSequence>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	int number_of_travel_time_budget_intervals = 36;  // 5 min interval

	int**** OAccessibilityMatrix = NULL;
	int**** DAccessibilityMatrix = NULL;
	OAccessibilityMatrix = Allocate4DDynamicArray<int>(total_demand_type + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize, number_of_travel_time_budget_intervals);
	DAccessibilityMatrix = Allocate4DDynamicArray<int>(total_demand_type + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize, number_of_travel_time_budget_intervals);


	cout << "------05---------" << endl;
	for (int d = 1; d <= total_demand_type; d++)
	for (int i = 1; i <= g_ODZoneIDSize; i++)
	for (int j = 1; j <= g_ODZoneIDSize; j++)
	for (int t = 0; t < StatisticsIntervalSize; t++)
	{   
		if (i == j)
		{
			ODTravelTime[d][i][j][t] = 0.5;
			ODDistance[d][i][j][t] = 0.5;
			ODDollarCost[d][i][j][t] = 0.0;
		}
		else
		{  
			ODTravelTime[d][i][j][t] = -1;
			ODDistance[d][i][j][t] = -1;
			ODDollarCost[d][i][j][t] = -1;
		}
	}
	cout << "------07---------" << endl;

	for (int d = 1; d <= total_demand_type; d++)
	for (int i = 1; i <= g_ODZoneIDSize; i++)
	for (int t = 1; t < StatisticsIntervalSize; t++)
	for (int ttb = 0; ttb < number_of_travel_time_budget_intervals; ttb++)
	{
		OAccessibilityMatrix[d][i][t][ttb] = 0;
		DAccessibilityMatrix[d][i][t][ttb] = 0;
	}

	if (bTimeDependentFlag)
		cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
	else
		cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


	//#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{

				// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0


		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

		if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
		{
			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
		}

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{

						bDistanceCost = false;
						for (int departure_time_index = 0; departure_time_index < StatisticsIntervalSize; departure_time_index++)
						{

							int departure_time = DemandLoadingStartTimeInMin + departure_time_index* g_AggregationTimetInterval;

							if (bTimeDependentFlag == false)  // real time version
							{
								if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
									continue;
							}
							g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, iterZone->first, departure_time, demand_type, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo

							// to each destination zone
							for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
							{

								int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
								if (dest_node_index >= 0 && (iterZone->first != iterZone2->first)) // convert node number to internal node id
								{

									int time_interval_no = departure_time_index;

									float TravelTime = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
									ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];

									ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];
									ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDollarCostAry[dest_node_index];

									
									///////////////////////////////////////////////////
									// fetch node sequence


										int NodeSize = 0;
										int PredNode = g_TimeDependentNetwork_MP[id].NodePredAry[dest_node_index];

										int node_number = g_NodeVector[dest_node_index].m_NodeNumber;
										ODPathNodeSequence[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no].m_node_sequence.push_back(node_number);

										while (PredNode != -1) // scan backward in the predessor array of the shortest path calculation results
										{
											if (NodeSize >= MAX_NODE_SIZE_IN_A_PATH - 1)
											{

												break;
											}
											node_number = g_NodeVector[PredNode].m_NodeNumber;
											ODPathNodeSequence[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no].m_node_sequence.push_back(node_number);

											PredNode = g_TimeDependentNetwork_MP[id].NodePredAry[PredNode];

								}
								//end of fetch shortest path

							///////////////////////////////////////////////////


									

									int TTB_interval = min(number_of_travel_time_budget_intervals, (TravelTime+4.99) / 5);
									//mark accessibility for TTB index less than TravelTime
									for (int ttb = TTB_interval; ttb < number_of_travel_time_budget_intervals; ttb++)
									{
										OAccessibilityMatrix[demand_type][iterZone->second.m_ZoneSequentialNo][time_interval_no][ttb] += 1;
										DAccessibilityMatrix[demand_type][iterZone2->second.m_ZoneSequentialNo][time_interval_no][ttb] += 1;
									}
								}

							} //for each destination zone
						}  // departure time
					}  // with origin node numbers 
				} // current thread	

			}  // origin zone

		} // each demand type 

	}  // multiple threads


	FILE* st = NULL;
	fopen_s(&st, file_name, "w");
	if (st != NULL)
	{
		//write header:
		fprintf(st, "from_zone_id,to_zone_id,departure_time_in_min,");
		CString str;

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{
			str.Format("DT%d_TT_in_min,", demand_type);
			fprintf(st, str);
		}

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{
			str.Format("DT%d_Distance,", demand_type);
			fprintf(st, str);
		}

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{
			str.Format("DT%d_Toll_Cost,", demand_type);
			fprintf(st, str);
		}


		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{
			str.Format("DT%d_path_node_sequence,", demand_type);
			fprintf(st, str);
		}
		//str.Format("demand_type_%d_generalized_travel_time_diff,demand_type_%d_distance_diff,demand_type_%d_dollar_cost_diff,", total_demand_type, total_demand_type, total_demand_type);


		fprintf(st, "\n");

		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{
			// to each destination zone
			for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
			{
				int origin_zone = iterZone->first;
				int destination_zone = iterZone2->first;


				if (origin_zone != destination_zone && origin_zone >= 1 && destination_zone >=1)
				{
					for (int departure_time_index = 0; departure_time_index < StatisticsIntervalSize; departure_time_index++)
					{

						int departure_time = DemandLoadingStartTimeInMin + departure_time_index* g_AggregationTimetInterval;
						int time_interval_no = departure_time_index;

						if (bTimeDependentFlag == false)  // real time version
						{
							if (fabs(CurrentTime - departure_time) > 10)  // not in the current time interval, skip outputing 
								continue;
						}

						fprintf(st, "%d,%d,%d,",
							origin_zone,
							destination_zone, 

							departure_time);

						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							fprintf(st, "%4.2f,",	ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							fprintf(st, "%4.2f,", ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
							fprintf(st, "%4.2f,", ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

						for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
						{
							std::vector<int> node_sequence =
								ODPathNodeSequence[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no].m_node_sequence;

							for (int ni = node_sequence.size() - 1; ni >= 0; ni--)
							{
								fprintf(st, "%d;", node_sequence[ni]);

							}
							fprintf(st, ",");

						}


						fprintf(st, "\n");

					}

				}  // each department type
			}


		}
		fclose(st);

	}
	else
	{
		cout << "File " << file_name << " cannot be opened." << endl;
		getchar();
		exit(0);

	}


	//// csv file
	//fopen_s(&st, "output_accessibility.csv", "w");
	//str_output_file_in_summary.Format("Output file =,output_accessibility.csv\n");
	//g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);


	//if (st != NULL)
	//{
	//	fprintf(st, "zone_id,departure_time_in_min,demand, demand_type,");

	//	for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//		fprintf(st, "OTTB_%d_min,", ttb*5);

	//	for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//		fprintf(st, "DTTB_%d_min,", ttb * 5);

	//	for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//		fprintf(st, "WOTTB_%d_min,", ttb * 5);

	//	for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//		fprintf(st, "WDTTB_%d_min,", ttb * 5);



	//	fprintf(st, "x,y,geometry,geometry\n");


	//	for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	//	{

	//		
	//		for (int t = 0; t < StatisticsIntervalSize; t++)
	//		{
	//			int demand_time = t + g_DemandLoadingStartTimeInMin / g_AggregationTimetInterval;


	//			for (int demand_type = 1; demand_type <= g_DemandTypeVector.size(); demand_type++)
	//			{
	//				int departure_time = demand_time*g_AggregationTimetInterval;

	//				fprintf(st, "%d,%d,%.1f,%d,",
	//					iterZone->second.m_ZoneNumber,
	//					departure_time,
	//					iterZone->second.m_Demand,
	//					demand_type);

	//				for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//					fprintf(st, "%d,", OAccessibilityMatrix[demand_type][iterZone->second.m_ZoneSequentialNo][t][ttb]);

	//				for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//					fprintf(st, "%d,", DAccessibilityMatrix[demand_type][iterZone->second.m_ZoneSequentialNo][t][ttb]);

	//				for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//					fprintf(st, "%.2f,", iterZone->second.m_Demand*OAccessibilityMatrix[demand_type][iterZone->second.m_ZoneSequentialNo][t][ttb]);

	//				for (int ttb = 1; ttb < number_of_travel_time_budget_intervals; ttb++)
	//					fprintf(st, "%.2f,", iterZone->second.m_Demand*DAccessibilityMatrix[demand_type][iterZone->second.m_ZoneSequentialNo][t][ttb]);
	//				
	//				int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

	//				if (origin_node_indx >= 0)
	//				{
	//					float x = g_NodeVector[origin_node_indx].m_pt.x;
	//					float y = g_NodeVector[origin_node_indx].m_pt.y;
	//					fprintf(st, "%f,%f,\"<Point><coordinates>%f,%f</coordinates></Point>\",",
	//						x,y,x, y);
	//				}
	//				else
	//				{
	//					fprintf(st, ",,,");
	//				}

	//				fprintf(st, "\"<Polygon><outerBoundaryIs><LinearRing><coordinates>");
	//				for (unsigned int si = 0; si< iterZone->second.m_ShapePoints.size(); si++)
	//				{
	//					fprintf(st, "%f,%f,0.0", iterZone->second.m_ShapePoints[si].x, iterZone->second.m_ShapePoints[si].y);

	//					if (si != iterZone->second.m_ShapePoints.size() - 1)
	//						fprintf(st, " ");
	//				}
	//				fprintf(st, "\</coordinates></LinearRing></outerBoundaryIs></Polygon>\"");

	//				fprintf(st, "\n");

	//			}

	//		}

	//	}

	//	fclose(st);
	//}
	//else
	//{
	//	cout << "File output_accessibility.css cannot be opened." << endl;
	//	getchar();
	//	exit(0);


	//}
	//for (int demand_type = 1; demand_type <= g_DemandTypeVector.size(); demand_type++)
	//{

	//	float base_rate = 1;


	//	for (int t = 0; t < StatisticsIntervalSize; t++)
	//	{
	//		CString File_name;
	//		File_name.Format("output_accessibility_demand_type_%d_min%d.kml", demand_type, g_DemandLoadingStartTimeInMin + t*g_AggregationTimetInterval);

	//		CString str_output_file_in_summary;
	//		str_output_file_in_summary.Format("Output file =,%s\n", File_name);
	//		g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);


	//		FILE* st;
	//		fopen_s(&st, File_name, "w");

	//		if (st != NULL)
	//		{
	//			fprintf(st, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
	//			fprintf(st, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
	//			fprintf(st, "<Document>\n");

	//			fprintf(st, "<name>accessibility</name>\n");

	//			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
	//			{

	//				for (int i = 0; i < iterZone->second.m_OriginActivityVector.size(); i++)
	//				{
	//					int node_id = iterZone->second.m_OriginActivityVector[i];

	//					int count = 0;

	//					int demand_time = t + g_DemandLoadingStartTimeInMin / g_AggregationTimetInterval;
	//					if (demand_time <MAX_DEMAND_TIME_SIZE)
	//						count = iterZone->second.m_AccessibilityCountMatrix[demand_type][t];

	//					count = count / 10;

	//					float x = g_NodeVector[node_id].m_pt.x;
	//					float y = g_NodeVector[node_id].m_pt.y;
	//					for (int k = 0; k < count; k++)
	//					{

	//						fprintf(st, "\t<Placemark>\n");
	//						fprintf(st, "\t <name>%d</name>\n", k + 1);
	//						fprintf(st, "\t <description>zone%d</description>\n", iterZone->second.m_ZoneNumber);
	//						fprintf(st, "\t <Point><coordinates>%f,%f,0</coordinates></Point>\n", x, y);
	//						fprintf(st, "\t</Placemark>\n");

	//					}  // for each count
	//				}

	//			}

	//		}

	//		fprintf(st, "</Document>\n");
	//		fprintf(st, "</kml>\n");
	//		fclose(st);
	//	}  // for each departure time
	//}




	//Deallocate4DDynamicArray(ODTravelTime, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	//Deallocate4DDynamicArray(ODDistance, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	//Deallocate4DDynamicArray(ODDollarCost, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	//Deallocate4DDynamicArray(ODPathNodeSequence, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	//Deallocate4DDynamicArray(OAccessibilityMatrix, total_demand_type + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);
	//Deallocate4DDynamicArray(DAccessibilityMatrix, total_demand_type + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);


}


void g_AgentBasedAccessibilityMatrixGeneration(bool bTimeDependentFlag, int DemandType, double CurrentTime)
{
	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int ComputationStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int ComputationEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		ComputationStartTimeInMin = CurrentTime;
		ComputationEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculation interval " << ComputationStartTimeInMin << " -> " << ComputationEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;


	int StatisticsIntervalSize = max(1, (ComputationEndTimeInMin - ComputationStartTimeInMin) / g_AggregationTimetInterval);



	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float*** ODTravelTime = NULL;
	ODTravelTime = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float*** ODDistance = NULL;
	ODDistance = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	for (int i = 0; i <= g_ODZoneIDSize; i++)
	for (int j = 0; j <= g_ODZoneIDSize; j++)
	for (int t = 0; t < StatisticsIntervalSize; t++)
	{

		if (i == j)
		{
			ODTravelTime[i][j][t] = 0.5;
			ODDistance[i][j][t] = 0.5;
		}
		else
		{
			ODTravelTime[i][j][t] = 0;
			ODDistance[i][j][t] = 0;
		}
	}


	if (bTimeDependentFlag)
		cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
	else
		cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{


		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0


		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

		if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
		{
			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
		}

		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{

			if ((iterZone->first%number_of_threads) == ProcessID)
			{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

				int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

				if (origin_node_indx >= 0) // convert node number to internal node id
				{

					bDistanceCost = false;
					for (int departure_time = ComputationStartTimeInMin; departure_time < ComputationEndTimeInMin; departure_time += g_AggregationTimetInterval)
					{
						g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, iterZone->first, departure_time, DemandType, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo


						// to each destination zone
						for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
						{

							int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
							if (dest_node_index >= 0) // convert node number to internal node id
							{

								int time_interval_no = (departure_time - ComputationStartTimeInMin) / g_AggregationTimetInterval;
								ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
								ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];

							}

						} //for each demand type
					}  // departure time
				}  // with origin node numbers 
			} // current thread	

		}  // origin zone

	}  // multiple threads


	if (bTimeDependentFlag == true) // time-dependent skim files
	{
		for (int departure_time = ComputationStartTimeInMin; departure_time < ComputationEndTimeInMin; departure_time += g_AggregationTimetInterval)
		{

			CString str;
			int time_interval_no = departure_time / g_AggregationTimetInterval;

			str.Format("output_skim_demand_type_%d_min%d.csv", DemandType, time_interval_no * 15);

			CString str_output_file_in_summary;
			str_output_file_in_summary.Format("Output file =,%s\n", str);
			g_SummaryStatFile.WriteTextLabel(str_output_file_in_summary);



			FILE* st = NULL;
			fopen_s(&st, str, "w");
			if (st != NULL)
			{

				fprintf(st, "origin_zone,destination_zone,demand_type,travel_time_in_min,travel_distance,travel_cost\n");


				// from each origin zone
				for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
				{
					// to each destination zone
					for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
					{

						int time_interval_no = (departure_time - ComputationStartTimeInMin) / g_AggregationTimetInterval;
						fprintf(st, "%d,%d,%4.2f,%4.2f\n",
							iterZone->first,
							iterZone2->first,
							ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

					}
				}


				fclose(st);
			}

		}

	}
	else // real time skim file
	{

		// step w: calculate experienced travel time for complete vehile trips

		ODStatistics** ODMOEArray = NULL;

		int total_number_of_zones = g_ZoneMap.size();

		ODMOEArray = AllocateDynamicArray<ODStatistics>(total_number_of_zones, total_number_of_zones);

		int i, j;
		for (i = 0; i < total_number_of_zones; i++)
		for (j = 0; j < total_number_of_zones; j++)
		{

			ODMOEArray[i][j].OriginZoneNumber = 0;
			ODMOEArray[i][j].DestinationZoneNumber = 0;
			ODMOEArray[i][j].TotalVehicleSize = 0;
			ODMOEArray[i][j].TotalCompleteVehicleSize = 0;
			ODMOEArray[i][j].TotalTravelTime = 0;
			ODMOEArray[i][j].TotalDistance = 0;

		}

		std::map<int, DTAVehicle*>::iterator iterVM;
		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
		{

			DTAVehicle* pVehicle = iterVM->second;

			int origin_zone_no = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
			int destination_zone_no = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

			ODMOEArray[origin_zone_no][destination_zone_no].TotalVehicleSize += 1;
			int arrival_time_window_begin_time_in_min = CurrentTime - g_AggregationTimetInterval;
			if (/*pVehicle->m_DemandType == DemandType && */pVehicle->m_NodeSize >= 2 && pVehicle->m_bComplete && pVehicle->m_ArrivalTime >= arrival_time_window_begin_time_in_min)  // with physical path in the network
			{


				ODMOEArray[origin_zone_no][destination_zone_no].OriginZoneNumber = pVehicle->m_OriginZoneID;
				ODMOEArray[origin_zone_no][destination_zone_no].DestinationZoneNumber = pVehicle->m_DestinationZoneID;


				ODMOEArray[origin_zone_no][destination_zone_no].TotalCompleteVehicleSize += 1;
				ODMOEArray[origin_zone_no][destination_zone_no].TotalTravelTime += pVehicle->m_TripTime;
				ODMOEArray[origin_zone_no][destination_zone_no].TotalDistance += pVehicle->m_Distance;
			}
		}


		for (i = 0; i < total_number_of_zones; i++)
		for (j = 0; j < total_number_of_zones; j++)
		{

			if (ODMOEArray[i][j].TotalCompleteVehicleSize >= 1)
			{

				ODTravelTime[i][j][0]
					= ODMOEArray[i][j].TotalTravelTime / max(1, ODMOEArray[i][j].TotalCompleteVehicleSize);

				ODDistance[i][j][0]
					= ODMOEArray[i][j].TotalDistance / max(1, ODMOEArray[i][j].TotalCompleteVehicleSize);
			}
		}

		//

		FILE* st = NULL;
		fopen_s(&st, "OD_real_time_skim.csv", "w");

		if (st != NULL)
		{
			g_LogFile << " output ODMOE data to file OD_real_time_skim.csv" << endl;

			fprintf(st, "from_zone_id,to_zone_id,number_of_agents,trip_time_in_min,trip_distance_in_mile\n");

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{
				// to each destination zone
				for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
				{

					for (int departure_time = ComputationStartTimeInMin; departure_time < ComputationEndTimeInMin; departure_time += g_AggregationTimetInterval)
					{
						int time_interval_no = (departure_time - ComputationStartTimeInMin) / g_AggregationTimetInterval;
						fprintf(st, "%d,%d,%d,%4.2f,%4.2f\n",
							iterZone->first,
							iterZone2->first,
							ODMOEArray[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo].TotalCompleteVehicleSize,
							ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]
							);

					}

				}
			}

			fclose(st);

			if (ODMOEArray != NULL)
				DeallocateDynamicArray<ODStatistics>(ODMOEArray, total_number_of_zones, total_number_of_zones);

		}
		else
		{
			cout << "File OD_real_time_skim.csv cannot be opened. Please check." << endl;
			g_ProgramStop();
		}



	}

	if (ODTravelTime != NULL)
		Deallocate3DDynamicArray<float>(ODTravelTime, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

	if (ODDistance != NULL)
		Deallocate3DDynamicArray<float>(ODDistance, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

}

void g_AgentBasedAccessibilityMatrixGenerationExtendedSingleFile(string file_name, double CurrentTime)
{

	bool bDistanceCost = true;

	bool bRebuildNetwork = true;

	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float*** ODTravelTime = NULL;
	ODTravelTime = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, g_DemandTypeVector.size() + 1);

	float*** ODDistance = NULL;
	ODDistance = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, g_DemandTypeVector.size() + 1);

	for (int i = 0; i <= g_ODZoneIDSize; i++)
	for (int j = 0; j <= g_ODZoneIDSize; j++)
	for (int t = 1; t <= g_DemandTypeVector.size(); t++)
	{

		if (i == j)
		{
			ODTravelTime[i][j][t] = 0.5;
			ODDistance[i][j][t] = 0.5;
		}
		else
		{
			ODTravelTime[i][j][t] = 0;
			ODDistance[i][j][t] = 0;
		}
	}

	cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;

	for (int demand_type = 1; demand_type <= g_DemandTypeVector.size(); demand_type++)
	{

#pragma omp parallel for
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{


			// create network for shortest path calculation at this processor
			int	id = omp_get_thread_num();  // starting from 0


			//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_AgentID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{

						bDistanceCost = false;
						g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, iterZone->first, CurrentTime, demand_type, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo


						// to each destination zone
						for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
						{

							int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
							if (dest_node_index >= 0) // convert node number to internal node id
							{

								ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][demand_type] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
								ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][demand_type] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];


							}

						} //for each destination zone
					}  // with origin node numbers 
				} // current thread	

			}  // origin zone

		}  // multiple threads

	}
	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "w");
	if (st != NULL)
	{


		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{
			// to each destination zone
			for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
			{

				for (int t = 1; t <= g_DemandTypeVector.size(); t++)
				{

					fprintf(st, "%d,%d,%4.2f,%4.2f\n",
						iterZone->first,
						iterZone2->first,
						ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][t],
						ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][t]);
				}
			}
		}


		fclose(st);
	}
	if (ODTravelTime != NULL)
		Deallocate3DDynamicArray<float>(ODTravelTime, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

	if (ODDistance != NULL)
		Deallocate3DDynamicArray<float>(ODDistance, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);


}

