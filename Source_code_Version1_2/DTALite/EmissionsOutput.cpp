//  Portions Copyright 2010 Xuesong Zhou, Hao Lei

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of DTALite.

//    DTALite is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    DTALite is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with DTALite.  If not, see <http://www.gnu.org/licenses/>.

// DTALite.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "DTALite.h"
#include "GlobalData.h"
#include "CSVParser.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>


using namespace std;

const int NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND = 10;
#define _MAXIMUM_OPERATING_MODE_SIZE 41
#define _MAXIMUM_AGE_SIZE 31

int OperatingModeMap[MAX_SPEED_BIN][MAX_VSP_BIN] = {-1};

class CEmissionRate 
{
public:
	bool bInitialized;
	float meanBaseRate_TotalEnergy;
	float meanBaseRate_CO2;
	float meanBaseRate_NOX;
	float meanBaseRate_CO;
	float meanBaseRate_HC;
	int Age;

	CEmissionRate()
	{
		bInitialized = false;
		meanBaseRate_TotalEnergy = meanBaseRate_CO2 = meanBaseRate_NOX = meanBaseRate_CO = meanBaseRate_HC = 0;
		Age = 0;
	}
};

CEmissionRate EmissionRateData[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_OPERATING_MODE_SIZE][_MAXIMUM_AGE_SIZE];

class CCycleAverageEmissionFactor
{
public:
	float emissionFactor_CO2;
	float emissionFactor_NOX;
	float emissionFactor_CO;
	float emissionFactor_HC;
	float average_cycle_speed;

	CCycleAverageEmissionFactor()
	{
		this->emissionFactor_CO2 = this->emissionFactor_NOX = this->emissionFactor_CO = this->emissionFactor_HC = 0.0f;
	}
	CCycleAverageEmissionFactor(float emissionFactor_CO2, float emissionFactor_NOX, float emissionFactor_CO, float emissionFactor_HC)
	{
		this->emissionFactor_CO2 = emissionFactor_CO2;
		this->emissionFactor_NOX = emissionFactor_NOX;
		this->emissionFactor_CO = emissionFactor_CO;
		this->emissionFactor_HC = emissionFactor_HC;
	}
};

CCycleAverageEmissionFactor CycleAverageEmissionFactorMatrix[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE];

class CVehicleEmissionResult
{
public:
	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;

	static void CalculateEmissions(int vehicle_type, std::map<int, int>& OperatingModeCount, int age, CVehicleEmissionResult& emissionResult)
	{
		emissionResult.Energy = 0;
		emissionResult.CO2 = 0;
		emissionResult.NOX = 0;
		emissionResult.CO = 0;
		emissionResult.HC = 0;

		for(std::map<int, int>::iterator iterOP  = OperatingModeCount.begin(); iterOP != OperatingModeCount.end (); iterOP++)
		{
			int OpModeID = iterOP->first; int count = iterOP->second;

			emissionResult.Energy+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_TotalEnergy*count/3600;
			emissionResult.CO2+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO2*count/3600;
			emissionResult.NOX+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_NOX*count/3600;
			emissionResult.CO+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO*count/3600;
			emissionResult.HC+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_HC*count/3600;
		}
	}
};

SPEED_BIN GetSpeedBinNo(float speed_mph)
{
	//the precision of speed output is 2
	//if the speed is 49.995, it would round up to 50
	//if the speed is 49.994, it would round to 49.99
	int speed = floor(speed_mph + 0.005);

	if(speed <= 25)
		return VSP_0_25mph;

	if(speed < 50)
		return VSP_25_50mph;
	else 
		return VSP_GT50mph;

}

VSP_BIN GetVSPBinNo(float vsp, float speed_mph)
{
	//if(speed_mph < 50)
	//{
	if (vsp < 0) return VSP_LT0;

	if (vsp < 3) return VSP_0_3;

	if (vsp < 6) return VSP_3_6;

	if (vsp < 9) return VSP_6_9;

	if (vsp < 12) return VSP_9_12;

	if (vsp < 18) return VSP_12_18;

	if (vsp < 24) return VSP_18_24;

	if (vsp < 30) return VSP_24_30;

	return VSP_GT30;

	//}
	//else  // greate than 50
	//{
	//	if(vsp < 6) 
	//		return VSP_LT6;
	//	if(vsp < 12) 
	//		return VSP_6_12;
	//	if (vsp < 18)
	//		return VSP_12_18;
	//	if (vsp < 24)
	//		return VSP_18_24;
	//	if (vsp < 30)
	//		return VSP_24_30;
	//	else
	//		return VSP_GT30;
	//}
}

void ReadInputEmissionRateFile()
{
	CCSVParser parser_emission;

	int line=1;
	if (parser_emission.OpenCSVFile("input_vehicle_emission_rate.csv"))
	{

		while(parser_emission.ReadRecord())
		{ line++;

		if(line == 256)
			TRACE("");


			int vehicle_type;
			int opModeID;

			if(parser_emission.GetValueByFieldName("vehicle_type",vehicle_type) == false)
				break;
			if(parser_emission.GetValueByFieldName("OpModeID",opModeID) == false)
				break;

			CEmissionRate element;
			if(parser_emission.GetValueByFieldName("meanBaseRate_TotalEnergy_(KJ/hr)",element.meanBaseRate_TotalEnergy) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_CO2_(g/hr)",element.meanBaseRate_CO2) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_NOX_(g/hr)",element.meanBaseRate_NOX) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_CO_(g/hr)",element.meanBaseRate_CO) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_HC_(g/hr)",element.meanBaseRate_HC) == false)
				break;


			if(parser_emission.GetValueByFieldName("age",element.Age) == false)
			{
				break;
			}


			if(element.Age< _MAXIMUM_AGE_SIZE && opModeID < _MAXIMUM_OPERATING_MODE_SIZE)
			{

			EmissionRateData[vehicle_type][opModeID][element.Age] = element;
			EmissionRateData[vehicle_type][opModeID][element.Age].bInitialized = true;
			}else
			{
			cout << "Reading error at line "  << line <<" at input_vehicle_emission_rate.csv" <<endl;
			g_ProgramStop();
			
			}

		}
	}
	else
	{
		cout << "Error: File input_vehicle_emission_rate.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	cout << "Read " << line << " lines from file input_vehicle_emission_rate.csv."<< endl;
}

//typedef struct
//{
//	int second;
//	float speed;
//	float position;
//} SecondData;
//
//class DrivingCycleSample
//{
//public:
//	int driving_cycle_id;
//	vector<SecondData> dataVector;
//};
//	//string s1 = "Cycle_PC_PT_LCT.csv";
//	//string s2 = "CYCLE_LHT.csv";
//	//string s3 = "Cycle_SST.csv";
////map<int, 
//void GenerateDrivingCycleSamples(int vehicle_type, string drivingCycleFileName)
//{
//	FILE* pFile;
//	fopen_s(&pFile, drivingCycleFileName.c_str(),"r");
//	vector<DrivingCycleSample> drivingCycleSamples;
//	if (pFile)
//	{
//		int curr_cycle_id = -1;
//		int cycle_id, second;
//		float speed;
//		DrivingCycleSample samples;
//		while (!feof(pFile))
//		{
//			fscanf(pFile,"%d,%d,%f\n",&cycle_id, &second, &speed);
//			if (cycle_id != curr_cycle_id)
//			{
//				samples.driving_cycle_id = cycle_id;
//			}
//			SecondData data;
//			data.second = second;
//			data.speed = speed;
//			samples.dataVector.push_back();
//		}
//	}
//}

enum Pollutants {EMISSION_CO2,EMISSION_NOX,EMISSION_CO,EMISSION_HC,MAX_POLLUTANT};

float BaseCycleFractionOfOperatingMode[MAX_VEHICLE_TYPE_SIZE][41] = {0.0f};
float BaseCycleEmissionRate[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE][MAX_POLLUTANT] = {0.0f};

void ReadFractionOfOperatingModeForBaseCycle()
{
	CCSVParser parser_emission_factor;
	if (parser_emission_factor.OpenCSVFile("input_base_cycle_fraction_of_OpMode.csv"))
	{
		int vehicle_type;
		float value;

		while(parser_emission_factor.ReadRecord())
		{			
			if (parser_emission_factor.GetValueByFieldName("vehicle_type",vehicle_type) == false)
			{
				break;
			}
			else
			{
				if (vehicle_type >= MAX_VEHICLE_TYPE_SIZE)
				{
					cout << "Warning: unknown vehicle_type " << vehicle_type << " !\n";
					continue;
				}

				if (parser_emission_factor.GetValueByFieldName("0", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][0] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("1", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][1] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("11", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][11] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("12", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][12] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("13", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][13] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("14", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][14] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("15", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][15] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("16", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][16] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("21", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][21] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("22", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][22] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("23", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][23] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("24", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][24] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("25", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][25] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("27", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][27] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("28", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][28] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("29", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][29] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("30", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][30] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("33", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][33] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("35", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][35] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("37", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][37] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("38", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][38] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("39", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][39] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("40", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][40] = value;
				}
			}
		}
	}
	else
	{
		cout << "Error: File input_base_cycle_fraction_of_OpMode.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}

	for (int vType = 0; vType < MAX_VEHICLE_TYPE_SIZE; vType++)
	{
		for (int vAge = 0; vAge < _MAXIMUM_AGE_SIZE; vAge++)
		{
			for (int opMode = 0; opMode < 41;opMode++)
			{
				if (EmissionRateData[vType][opMode][vAge].bInitialized)
				{
					BaseCycleEmissionRate[vType][vAge][EMISSION_CO2] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_CO2;
					BaseCycleEmissionRate[vType][vAge][EMISSION_NOX] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_NOX;
					BaseCycleEmissionRate[vType][vAge][EMISSION_CO] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_CO;
					BaseCycleEmissionRate[vType][vAge][EMISSION_HC] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_HC;
				}
			}
		}
	}
	cout << "Reading file input_base_cycle_fraction_of_OpMode.csv..."<< endl;
}

typedef struct
{
	int second;
	float speed;
	float position;
} SecondData;

class DrivingCycleSample
{
public:
	int driving_cycle_id;
	int defaultTimeSpan;
	vector<SecondData> dataVector;
	DrivingCycleSample(int cycle_id, int timeSpan = 300)
	{
		driving_cycle_id = cycle_id;
		defaultTimeSpan = timeSpan;
	}

	void SetSegmentTimeSpan(int timeSpan)
	{
		defaultTimeSpan = timeSpan;
	}

	float GetSpeedByTime(unsigned int segmentID, int time)
	{
		if (segmentID * defaultTimeSpan + time < dataVector.size())
		{
			return dataVector[segmentID * defaultTimeSpan + time].speed;
		}
		else
		{
			return 0.0f;
		}
	}

	float GetPositionByTime(unsigned int segmentID, int time)
	{
		if (segmentID * defaultTimeSpan + time >= dataVector.size())
		{
			return -1.0f;
		}

		float position_offset;
		if (segmentID == 0)
		{
			position_offset = 0.0f;
		}
		else
		{
			position_offset = dataVector[segmentID * defaultTimeSpan].position;
		}

		return max(0.0f, dataVector[segmentID * defaultTimeSpan + time].position - position_offset);
	}
};

typedef struct
{
	int cycle_id;
	int segment_id;
} DrivingCycleIdentifier;

class TrajectoryMatrix
{
public:
	int timeWindowSize;
	int spaceWindowSize;
private:
	vector<DrivingCycleIdentifier>** pMatrix;
public:
	TrajectoryMatrix(int n = 300, int m = 800)
	{
		timeWindowSize = n;
		spaceWindowSize = m;
		pMatrix = new vector<DrivingCycleIdentifier>*[n];
		for (int i=0;i<n;i++)
		{
			pMatrix[i] = new vector<DrivingCycleIdentifier>[m];
		}
	}

	void AddToMatrix(DrivingCycleSample& sample, unsigned int segment_id)
	{
		for (int i=0;i<sample.defaultTimeSpan;i++)
		{		
			int posIdx = (int)sample.GetPositionByTime(segment_id,i);
			if (posIdx < 0) break;
			DrivingCycleIdentifier identifier;
			identifier.cycle_id = sample.driving_cycle_id;
			identifier.segment_id = segment_id;
			if (i < timeWindowSize && posIdx < spaceWindowSize)
			{
				pMatrix[i][posIdx].push_back(identifier);
			}
		}
	}

	vector<DrivingCycleIdentifier>* GetIdentifier(int n, int m) const
	{
		if (n < timeWindowSize && m < spaceWindowSize)
		{
			return &pMatrix[n][m];
		}

		return NULL;
	}

	~TrajectoryMatrix()
	{
		for (int i=0;i<timeWindowSize;i++)
		{
			delete [] pMatrix[i];
		}

		delete [] pMatrix;
	}
};

//TrajectoryMatrix trajectoryMatrixBin[MAX_VEHICLE_TYPE_SIZE][7];

int GetDrivingCycleVehicleSpeedBin(float speed)
{
	if (speed < 15.0f) return 0;
	if (speed < 30.0f) return 1;
	if (speed < 40.0f) return 2;
	if (speed < 50.0f) return 3;
	if (speed < 60.0f) return 4;
	if (speed < 70.0f) return 5;

	return 6;
}
map<int,vector<DrivingCycleSample*>> drivingCycleMap;

void ReadInputCycleAverageEmissionFactors()
{
	CCSVParser parser_emission_factor;
	if (parser_emission_factor.OpenCSVFile("input_cycle_emission_factor.csv"))
	{
		int vehicle_type;
		float emissionFactor_CO2;
		float emissionFactor_NOX;
		float emissionFactor_CO;
		float emissionFactor_HC;
		float cycle_average_speed;
		int age;

		while (parser_emission_factor.ReadRecord())
		{
			if (parser_emission_factor.GetValueByFieldName("vehicle_type", vehicle_type) == false
				|| parser_emission_factor.GetValueByFieldName("CO2_emission_factor_(g/mi)", emissionFactor_CO2) == false
				|| parser_emission_factor.GetValueByFieldName("NOX_emission_factor_(g/mi)", emissionFactor_NOX) == false
				|| parser_emission_factor.GetValueByFieldName("CO_emission_factor_(g/mi)", emissionFactor_CO) == false
				|| parser_emission_factor.GetValueByFieldName("HC_emission_factor_(g/mi)", emissionFactor_HC) == false
				|| parser_emission_factor.GetValueByFieldName("age", age) == false
				)
			{
				break;
			}
			else
			{
				CycleAverageEmissionFactorMatrix[vehicle_type][age] = CCycleAverageEmissionFactor(emissionFactor_CO2, emissionFactor_NOX, emissionFactor_CO, emissionFactor_HC);
			}
		}
	}
	else
	{
		cout << "Error: File input_cycle_emission_factor.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		g_ProgramStop();
	}

	cout << "Reading file input_cycle_emission_factor.csv..." << endl;
}

//void ReadDrivingCycleSamplesByVehicleType(int vehicle_type, char* fileName, vector<DrivingCycleSample*>& drivingCycleSamples)
//{
//	FILE* pFile;
//	fopen_s(&pFile,fileName, "r");
//	int curr_cycle_id = -1;
//	int cycle_id, second;
//	float speed, position;
//	DrivingCycleSample* pDrivingCycleSample;
//	while (!feof(pFile))
//	{
//		fscanf_s(pFile,"%d,%d,%f\n",&cycle_id, &second, &speed);
//		if (curr_cycle_id != cycle_id)
//		{
//			curr_cycle_id = cycle_id;
//			position = 0.0f;
//			pDrivingCycleSample = new DrivingCycleSample(cycle_id);
//			drivingCycleSamples.push_back(pDrivingCycleSample);
//		}
//
//		SecondData data;
//		data.second = second;
//		data.speed = speed;
//		if (second == 0)
//		{
//			position = 0.0f;
//		}
//		else
//		{
//			position += speed * 5280.0f / 3600.0f * 0.3048f;
//		}
//		data.position = position;
//		pDrivingCycleSample->dataVector.push_back(data);
//	}
//	fclose(pFile);
//
//	if (vehicle_type < MAX_VEHICLE_TYPE_SIZE)
//	{
//		for (int i=0;i<drivingCycleSamples.size();i++)
//		{
//			int segment_id = 0;
//			for (int j=0;j<drivingCycleSamples[i]->dataVector.size();j=j+drivingCycleSamples[i]->defaultTimeSpan)
//			{
//				float initial_speed = drivingCycleSamples[i]->dataVector[i].speed;
//				int speed_bin = GetDrivingCycleVehicleSpeedBin(initial_speed);
//				trajectoryMatrixBin[vehicle_type][speed_bin].AddToMatrix(*drivingCycleSamples[i],segment_id);
//				segment_id++;
//			}
//		}
//	}
//}

int SmoothVehicleAcceleration(float link_ff_speed, float link_length, float initial_speed, int start_time, int end_time, float* position, int size)
{
	const float Acceleration_Rate = 7.5f; //8mph/s
	const float Deceleration_Rate = 5.0f; //3.3mph/s
	float speed_above_ff_speed = 10.0; //mph

	//t1: acceleration time: from initial_speed to link_ff_speed + speed_above_ff_speed
	//t2: constant cruising time 
	//t3: deceleration time: from link_ff_speed + speed_above_ff_speed to link_ff_speed

	float t1 = (link_ff_speed + speed_above_ff_speed - initial_speed) / Acceleration_Rate; // in second
	float t3 = speed_above_ff_speed / Deceleration_Rate; // in second
	float t2 = (link_ff_speed * 5280.0f / 3600.0f * (t1 + t3) - (initial_speed * 5280.0f / 3600.0f * t1 + 0.5f * Acceleration_Rate * 5280.0f / 3600.0f * t1 * t1 + (link_ff_speed + speed_above_ff_speed) * (5280.0f / 3600.0f) * t3 - 0.5f * Deceleration_Rate * (5280.0f / 3600.0f) * t3 * t3)) / (speed_above_ff_speed * 5280.0f / 3600.0f);

	if ((int)((t1 + t2 + t3) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND) <= (end_time - start_time))
	{
		int end_of_duration_1 = start_time + (int)(t1 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		int end_of_duration_2 = end_of_duration_1 + (int)(t2 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		int end_of_duration_3 = end_of_duration_2 + (int)(t3 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		for (int t = start_time; t < end_of_duration_1; t++)
		{
			position[t % size] = initial_speed * 5280.0f / 3600.0f + 0.5f * Acceleration_Rate * 5280.0f / 3600.0f * (t - start_time + 1) * (t - start_time) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		}

		for (int t = end_of_duration_1; t < end_of_duration_2; t++)
		{
			position[t % size] = position[end_of_duration_1 -1] + (link_ff_speed + speed_above_ff_speed) * 5280.0f / 3600.0f * (t - end_of_duration_1 + 1) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
		}

		for (int t = end_of_duration_2; t < end_of_duration_3; t++)
		{
			position[t % size] = position[end_of_duration_2 - 1] + (link_ff_speed + speed_above_ff_speed) * 5280.0f / 3600.0f * (t - end_of_duration_2 + 1) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND - 0.5f * Deceleration_Rate * 5280.0f / 3600.0f * (t - end_of_duration_2) * (t - end_of_duration_2) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		}

		return (t1+t2+t3);
	}
	else
	{
		return 0;
	}
}

void SetupOperatingModeVector()
{
	OperatingModeMap[VSP_0_25mph][VSP_LT0] = 11;
	OperatingModeMap[VSP_0_25mph][VSP_0_3] = 12;
	OperatingModeMap[VSP_0_25mph][VSP_3_6] = 13;
	OperatingModeMap[VSP_0_25mph][VSP_6_9] = 14;
	OperatingModeMap[VSP_0_25mph][VSP_9_12] = 15;
	OperatingModeMap[VSP_0_25mph][VSP_12_18] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_18_24] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_24_30] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_GT30] = 16;

	OperatingModeMap[VSP_25_50mph][VSP_LT0] = 21;
	OperatingModeMap[VSP_25_50mph][VSP_0_3] = 22;
	OperatingModeMap[VSP_25_50mph][VSP_3_6] = 23;
	OperatingModeMap[VSP_25_50mph][VSP_6_9] = 24;
	OperatingModeMap[VSP_25_50mph][VSP_9_12] = 25;
	OperatingModeMap[VSP_25_50mph][VSP_12_18] = 27;
	OperatingModeMap[VSP_25_50mph][VSP_18_24] = 28;
	OperatingModeMap[VSP_25_50mph][VSP_24_30] = 29;
	OperatingModeMap[VSP_25_50mph][VSP_GT30] = 30;

	OperatingModeMap[VSP_GT50mph][VSP_LT0] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_0_3] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_3_6] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_6_9] = 35;
	OperatingModeMap[VSP_GT50mph][VSP_9_12] = 35;
	OperatingModeMap[VSP_GT50mph][VSP_12_18] = 37;
	OperatingModeMap[VSP_GT50mph][VSP_18_24] = 38;
	OperatingModeMap[VSP_GT50mph][VSP_24_30] = 39;
	OperatingModeMap[VSP_GT50mph][VSP_GT30] = 40;
}

int GetOperatingMode(float vsp, float s_mph)
{
	SPEED_BIN SpeedBinNo = GetSpeedBinNo(s_mph);
	VSP_BIN VSPBinNo =  GetVSPBinNo(vsp, s_mph);

	int OPBin =  OperatingModeMap[SpeedBinNo][VSPBinNo];
	return OPBin;
}

int ComputeOperatingModeFromSpeed(float &vsp, float v /*meter per second*/,float a, float grade = 0, int vehicle_type =1)
{
	int vehicle_type_no = vehicle_type -1; // start from 0
	double TermA = g_VehicleTypeVector[vehicle_type_no].rollingTermA ;
	double TermB = g_VehicleTypeVector[vehicle_type_no].rotatingTermB ;
	double TermC = g_VehicleTypeVector[vehicle_type_no].dragTermC  ;
	double Mass =  g_VehicleTypeVector[vehicle_type_no].sourceMass;
	vsp = (TermA*v + TermB*v*v +  TermC*v*v*v + v*a*Mass)/Mass;
	//vsp = TermA/Mass*v + TermB/Mass*v*v+  TermC/Mass*v*v*v;
	float speed_mph = v * 2.23693629f;  //3600 seconds / 1606 meters per hour

	int OpMode = GetOperatingMode(vsp,speed_mph);

	return OpMode; 
}


bool vehicleCF_sort_function (VehicleCFData i,VehicleCFData j) 
{ 
	return (i.StartTime_in_SimulationInterval < j.StartTime_in_SimulationInterval );
}
bool matchMicroTrip(int vehicle_type, int start_time, float start_speed, float start_position, vector<float>& speedArray, vector<float>& position, 
					int arraySize, DrivingCycleIdentifier& matchedDrivingCycleIdentifier, int& exactSecond)
{
	//int speed_bin = GetDrivingCycleVehicleSpeedBin(start_speed);
	//TrajectoryMatrix& trajMatrix = trajectoryMatrixBin[speed_bin][vehicle_type];
	//bool isMatch = false;

	//int t = 0;
	//for (int n = start_time; n < arraySize && t < trajMatrix.timeWindowSize; n++,t++)
	//{
	//	int upperBound = min((int)(position[n] - start_position) + 10, trajMatrix.spaceWindowSize);
	//	int lowerBound = max(0, upperBound - 20);

	//	for (int m = lowerBound; m < upperBound; m++)
	//	{
	//		vector<DrivingCycleIdentifier>* pDrivingCycleIdentifierVector = trajMatrix.GetIdentifier(t, m);

	//		if (pDrivingCycleIdentifierVector && pDrivingCycleIdentifierVector->size() > 0)
	//		{
	//			float targetSpeed = speedArray[n+1];
	//			for (int k = 0; k < pDrivingCycleIdentifierVector->size();k++)
	//			{
	//				float speed = drivingCycleMap[vehicle_type][(*pDrivingCycleIdentifierVector)[k].cycle_id -1]->GetSpeedByTime((*pDrivingCycleIdentifierVector)[k].segment_id,n - start_time);
	//				if (abs(speed - targetSpeed) < 5)
	//				{
	//					matchedDrivingCycleIdentifier = (*pDrivingCycleIdentifierVector)[0];
	//					exactSecond = n - start_time;
	//					return true;
	//				}
	//			}
	//		}
	//	}

	//}

	//return false;
	///*for (int i=0;i<trajMatrix.timeWindowSize;i++)
	//{
	//	for (int j=0;j<trajMatrix.spaceWindowSize;j++)
	//	{

	//	}
	//}*/
	return true;
}
void ReconstructTrajectory(int vehicle_type, std::map<int, VehicleSpeedProfileData>& speedProfile)
{
	std::map<int, VehicleSpeedProfileData>::iterator iter = speedProfile.begin();
	int size = speedProfile.size();
	vector<float> speedArray;
	vector<float> position;
	position.push_back(0.0f);
	bool isUpdated = false;

	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		speedArray.push_back(iter->second.Speed);
		if (i == 0)
		{
			position[0] = 0.0f;
		}
		else
		{
			position.push_back(position[i-1] + iter->second.Speed * 0.44704f);
		}
	}

	vector<int> transitionPoints;
	for (int i=1;i<size;i++)
	{
		if (abs(speedArray[i] - speedArray[i-1]) > 10)
		{
			transitionPoints.push_back(i);
		}
	}

	if (transitionPoints.size() > 0)
	{
		for (int i=0;i<transitionPoints.size();i++)
		{
			int t = transitionPoints[i];
			int exactSecond;
			DrivingCycleIdentifier drivingCycleIdentifier;
			if (matchMicroTrip(vehicle_type,t,speedArray[t-1], position[t-1],speedArray,position, size, drivingCycleIdentifier, exactSecond))
			{
				std::cout << "==================Microtrip matched!====================\n";
				if (i != transitionPoints.size() - 1)
				{
					if (t + exactSecond < transitionPoints[i+1])
					{						
						for (int n=0;n<exactSecond;n++)
						{
							speedArray[i-1] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.cycle_id]->GetSpeedByTime(drivingCycleIdentifier.segment_id,n);
						}
						isUpdated = true;
					}
				}
				else
				{
					if (t + exactSecond < size)
					{
						for (int n=0;n<exactSecond;n++)
						{
							speedArray[i-1] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.cycle_id]->GetSpeedByTime(drivingCycleIdentifier.segment_id,n);
						}

						isUpdated = true;
					}
				}
			}
			else
			{

			}
		}
	}

	if (isUpdated)
	{
		iter = speedProfile.begin();
		for (int i=0;iter != speedProfile.end();iter++,i++)
		{
			iter->second.Speed = speedArray[i];
		}
	}

}
enum TransitionType {ACC, DEC};
typedef struct
{
	int t;
	TransitionType type;

} TransitionPoint;


/* Acceleration and Deceleration Range
0-1			-2		0.6
1-5			-3.7	2.4
5-10		-5.1	5.6
10-20		-4.5	5
20-30		-3.7	3.6
30-40		-2.9	2.7
40-50		-1.6	1.7
50-60		-1		1
60-70		-1		1
70 and up	-1		1

*/

float maxAcceleration = (0.6f, 2.4f, 5.6f, 5.0f, 3.6f, 2.7f, 1.7f, 1.0f, 1.0f, 1.0f);
float maxDeceleration = (-2.0f, -3.7f, -5.1f, -4.5f, -3.7f, -2.9f, -1.6f, -1.0f, -1.0f, -1.0f);

float maxAccelerationBySpeed(float speedInMiles)
{
	if (speedInMiles < 1) return 0.6f;
	if (speedInMiles < 5) return 2.4f;
	if (speedInMiles < 10) return 5.6f;
	if (speedInMiles < 20) return 5.0f;
	if (speedInMiles < 30) return 3.6f;
	if (speedInMiles < 40) return 2.7f;
	if (speedInMiles < 50) return 1.7f;
	
	return 1.0f;
}

float maxDecelerationBySpeed(float speedInMiles)
{
	if (speedInMiles < 1) return -2.0f;
	if (speedInMiles < 5) return -3.7f;
	if (speedInMiles < 10) return -5.1f;
	if (speedInMiles < 20) return -4.5f;
	if (speedInMiles < 30) return -3.7f;
	if (speedInMiles < 40) return -2.9f;
	if (speedInMiles < 50) return -1.6f;
	
	return -1.0f;
}

void MovingAverage(std::vector<float>& p)
{
	size_t sz = p.size();

	if (sz >= 7)
	{
		p[0] = p[0];
		p[1] = (p[0] + p[1] + p[2]) / 3.0f;
		p[2] = (p[0] + p[1] + p[2] + p[3] + p[4]) / 5.0f;		

		for (int i = 3; i <= sz - 4; i++)
		{
			p[i] = (p[i - 3] + p[i - 2] + p[i - 1] + p[i] + p[i + 1] + p[i + 2] + p[i + 3]) / 7.0f;
		}

		p[sz - 3] = (p[sz - 5] + p[sz - 4] + p[sz - 3] + p[sz - 2] + p[sz - 1] ) / 5.0f;
		p[sz - 2] = (p[sz - 3] + p[sz - 2] + p[sz - 1]) / 3.0f;
		p[sz - 1] = p[sz - 1];
	}

	return;
}

void MovingAverage(float* p, int sz)
{
	if (p == NULL || sz == 0)
	{
		return;
	}

	if (sz >= 7)
	{
		p[0] = p[0];
		p[1] = (p[0] + p[1] + p[2]) / 3.0f;
		p[2] = (p[0] + p[1] + p[2] + p[3] + p[4]) / 5.0f;		

		for (int i = 3; i <= sz - 4; i++)
		{
			p[i] = (p[i - 3] + p[i - 2] + p[i - 1] + p[i] + p[i + 1] + p[i + 2] + p[i + 3]) / 7.0f;
		}

		p[sz - 3] = (p[sz - 5] + p[sz - 4] + p[sz - 3] + p[sz - 2] + p[sz - 1] ) / 5.0f;
		p[sz - 2] = (p[sz - 3] + p[sz - 2] + p[sz - 1]) / 3.0f;
		p[sz - 1] = p[sz - 1];
	}

	return;
}

vector<float> SpeedUpTo(int vehicle_id, float initial_speed, float targetSpeed)
{
	
	float speed = initial_speed;
	vector<float> speedVector;
	//if (initial_speed > targetSpeed) return speedVector;

	srand(vehicle_id);

	float currAccRate = maxAccelerationBySpeed(initial_speed) * (float)(rand() / double(RAND_MAX));

	while (speed + currAccRate < targetSpeed)
	{
		speed += currAccRate;
		speedVector.push_back(speed);
		currAccRate = maxAccelerationBySpeed(speed) * (float)(rand() / double(RAND_MAX));
	}
	speedVector.push_back(targetSpeed);
	MovingAverage(speedVector);
	return speedVector;
}

vector<float> SlowDownTo(int vehicle_id, float initial_speed, float targetSpeed)
{
	float speed = initial_speed;
	vector<float> speedVector;
	//if (initial_speed > targetSpeed) return speedVector;

	srand(vehicle_id);
	float maxDec = maxDecelerationBySpeed(initial_speed);
	float currDecRate = maxDec * (float)(rand() / double(RAND_MAX));

	while (speed + currDecRate > targetSpeed)
	{
		speed += currDecRate;
		speedVector.push_back(speed);
		maxDec = maxDecelerationBySpeed(initial_speed);
		currDecRate = maxDec * (float)(rand() / double(RAND_MAX));

	}
	speedVector.push_back(targetSpeed);
	MovingAverage(speedVector);
	return speedVector;
}




void SmoothVehicleTrajectory(int vehicle_id, std::map<int, VehicleSpeedProfileData>& speedProfile, int maxRun = 11)
{
	std::map<int, VehicleSpeedProfileData>::iterator iter = speedProfile.begin();
	int size = speedProfile.size();
	vector<float> speedArray;

	//speedArray.push_back(0.0f);
	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		speedArray.push_back(iter->second.Speed);
	}
	speedArray[speedArray.size() - 1] = 0.0f;

	//Find the transition points
	vector<TransitionPoint> transitionPoints;
	for (int i=1;i<size;i++)
	{
		if (speedArray[i] - speedArray[i-1] > 5.6 || speedArray[i] - speedArray[i-1] < -5.1)
		{
			if (i != size -1)
			{
				if (abs(speedArray[i] - speedArray[i-1]) == abs(speedArray[i] - speedArray[i+1]))
				{
					speedArray[i] = speedArray[i-1];
					i++;
					continue;
				}
			}

			TransitionPoint point;
			point.t = i;
			point.type = speedArray[i] - speedArray[i-1] > 0 ? ACC : DEC;
			transitionPoints.push_back(point);
		}
	}

	MovingAverage(speedArray);

	if (transitionPoints.size() <= 0) return;

	//Do the smoothing based on accelertion or deceleration
	for (int i=0;i<transitionPoints.size();i++)
	{
		TransitionPoint currPoint = transitionPoints[i];
		//if (i > 0)
		//{
		//	TransitionPoint prevPoint = transitionPoints[i-1];
		//	if (currPoint.t - prevPoint.t < 5)
		//	{
		//		continue;
		//	}
		//}
		float initialSpeed = speedArray[currPoint.t -1];
		float targetSpeed = speedArray[currPoint.t];
		vector<float> speedVector;
		if (currPoint.type == ACC)
		{
			speedVector = SpeedUpTo(vehicle_id, initialSpeed, targetSpeed);
			if (i != transitionPoints.size() - 1)
			{
				TransitionPoint nextPoint = transitionPoints[i+1];

				if (speedVector.size() > 0 /*&& nextPoint.t - currPoint.t > speedVector.size()*/)
				{
					if (currPoint.t >= speedVector.size())
					{
						for (int n = 0; n < speedVector.size(); n++)
						{
							speedArray[currPoint.t - speedVector.size() + 1 + n] = speedVector[n];
						}
					}
					else
					{
						for (int n = 0; n < speedVector.size(); n++)
						{
							speedArray[currPoint.t + n] = speedVector[n];
						}

					}
				}
			}
		}
		else
		{
			speedVector = SlowDownTo(vehicle_id, initialSpeed, targetSpeed);
			if (speedVector.size() > 0 && currPoint.t >= speedVector.size())
			{
				for (int n = 0; n < speedVector.size(); n++)
				{
					speedArray[currPoint.t - speedVector.size() + 1 + n] = speedVector[n];
				}
			}
		}
		
	}

	MovingAverage(speedArray);

	iter = speedProfile.begin();


	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		iter->second.Speed = speedArray[i];
		if (i == 0)
		{
			iter->second.Acceleration = 0.0f;
		}
		else
		{
			iter->second.Acceleration = speedArray[i] - speedArray[i - 1];
			//if (iter->second.Acceleration > 0 && iter->second.Acceleration 
		}
	}

	if (maxRun > 0)
	{
		SmoothVehicleTrajectory(vehicle_id, speedProfile, --maxRun);
	}
		
}
void g_CalculateEmissionMOE()
{
#ifdef _large_memory_usage

	std::set<DTALink*>::iterator iterLink;
	int count = 0;
	// step 1: generate data for different lanes
	for (unsigned li = 0; li< g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		pLink->m_VehicleDataVector.clear();
		for (int LaneNo = 0; LaneNo < pLink->m_OutflowNumLanes; LaneNo++)
		{
			LaneVehicleCFData element(g_PlanningHorizon + 1);

			pLink->m_VehicleDataVector.push_back(element);
		}
	}

	// step 2: collect all vehicles passing each link

	int totalSamples = (int)(g_VehicleMap.size() * g_OutputSecondBySecondEmissionDataPercentage);
	int sampling_interval = (int)(g_VehicleMap.size() / max(1, totalSamples));
	int vehicleCounts = 0;

	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;

		pVehicle->m_OperatingModeCount.clear();

		if (pVehicle->m_NodeSize >= 2)
		{
			for (int i = 0; i< pVehicle->m_NodeSize - 1; i++)
			{
				if (pVehicle->m_bComplete)  // for vehicles finish the trips
				{
					VehicleCFData element;
					element.VehicleID = pVehicle->m_AgentID;
					int LinkNo = pVehicle->m_LinkAry[i].LinkNo;
					element.SequentialLinkNo = i;

					element.FreeflowDistance_per_SimulationInterval = g_LinkVector[LinkNo]->m_SpeedLimit* 1609.344f / 3600 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					//1609.344f: mile to meters ; 3600: # of seconds per hour
					//τ = 1/(wkj)
					// δ = 1/kj
					element.TimeLag_in_SimulationInterval = (int)(3600 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / (g_LinkVector[LinkNo]->m_BackwardWaveSpeed * g_LinkVector[LinkNo]->m_KJam) + 0.5);
					element.CriticalSpacing_in_meter = 1609.344f / g_LinkVector[LinkNo]->m_KJam;

					if (i == 0)
					{
						element.StartTime_in_SimulationInterval = pVehicle->m_DepartureTime * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					}
					else
					{
						element.StartTime_in_SimulationInterval = pVehicle->m_LinkAry[i - 1].AbsArrivalTimeOnDSN * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					}

					element.EndTime_in_SimulationInterval = pVehicle->m_LinkAry[i].AbsArrivalTimeOnDSN * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					element.LaneNo = element.VehicleID%g_LinkVector[LinkNo]->m_OutflowNumLanes;

					g_LinkVector[LinkNo]->m_VehicleDataVector[element.LaneNo].LaneData.push_back(element);
				}
			}

			if (pVehicle->m_bComplete)
			{
				vehicleCounts++;
				if (vehicleCounts % sampling_interval == 0)
				{
					pVehicle->m_bDetailedEmissionOutput = true;
				}
			}
			// for each vehicle
		}
	}


	// calclate link based VSP
	// step 3: collect all vehicle passing each link
	for (int li = 0; li< g_LinkVector.size(); li++)
	{
		TRACE("\n compute emissions for link %d", li);
		//		g_LogFile << "compute emissions for link " << li << endl;
		cout << "compute emissions for link " << li << endl;
		g_LinkVector[li]->ComputeVSP();
		//g_LinkVector[li]->ThreeDetectorVehicleTrajectorySimulation();
	}

	float Energy = 0;
	float CO2 = 0;
	float NOX = 0;
	float CO = 0;
	float HC = 0;

	// aggregation for each vehicle 
	CVehicleEmissionResult emissionResult;

	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;
		if (pVehicle->m_NodeSize >= 2 && pVehicle->m_OperatingModeCount.size() > 0)  // with physical path in the network
		{
			CVehicleEmissionResult::CalculateEmissions(pVehicle->m_VehicleType, pVehicle->m_OperatingModeCount, pVehicle->m_Age, emissionResult);

			pVehicle->m_Emissions = emissionResult.CO2;
			pVehicle->Energy = emissionResult.Energy;
			pVehicle->CO2 = emissionResult.CO2;
			pVehicle->NOX = emissionResult.NOX;
			pVehicle->CO = emissionResult.CO;
			pVehicle->HC = emissionResult.HC;

			Energy += emissionResult.Energy;
			CO2 += emissionResult.CO2;
			NOX += emissionResult.NOX;
			CO += emissionResult.CO;
			HC += emissionResult.HC;
		}
	}

	g_SimulationResult.Energy = Energy;
	g_SimulationResult.CO2 = CO2;
	g_SimulationResult.NOX = NOX;
	g_SimulationResult.CO = CO;
	g_SimulationResult.HC = HC;


	// aggregation for each link 
	for (unsigned li = 0; li< g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		int TimeSize = min(pLink->m_LinkMOEAry.size(), pLink->m_VehicleDataVector[0].m_LaneEmissionVector.size());
		for (int t = 0; t< TimeSize; t++)
		{
			for (int LaneNo = 0; LaneNo < pLink->m_OutflowNumLanes; LaneNo++)
			{
				pLink->m_LinkMOEAry[t].Energy += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].Energy;
				pLink->m_LinkMOEAry[t].CO2 += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO2;
				pLink->m_LinkMOEAry[t].NOX += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].NOX;
				pLink->m_LinkMOEAry[t].CO += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO;
				pLink->m_LinkMOEAry[t].HC += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].HC;

				pLink->TotalEnergy += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].Energy;
				pLink->TotalCO2 += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO2;
				pLink->TotalNOX += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].NOX;
				pLink->TotalCO += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO;
				pLink->TotalHC += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].HC;
			}
		}
	}
#endif 
}
void OutputEmissionData()
{
#ifdef _large_memory_usage
	if(g_TrafficFlowModelFlag ==  tfm_BPR)
		return; // no emission output for BPR function

	//ReadDrivingCycleSamplesByVehicleType(1,"input_Cycle_PC_PT_LCT.csv",drivingCycleMap[1]);
	//ReadDrivingCycleSamplesByVehicleType(4,"input_Cycle_SST.csv",drivingCycleMap[4]);
	//ReadDrivingCycleSamplesByVehicleType(5,"input_CYCLE_LHT.csv",drivingCycleMap[5]);

	g_CalculateEmissionMOE();

	// output speed data second by second
	if(g_OutputEmissionOperatingModeData)
	{
		FILE* st2;
		fopen_s(&st2,"output_vehicle_operating_mode.csv","w");

		FILE* fpSampledEmissionsFile = NULL;
		FILE* fpSampledCycleEmissionRateFile = NULL;

		int numOfSamples = 0;
		//int numOfVehicles = 0;

		if (g_OutputSecondBySecondEmissionData)
		{
			fopen_s(&fpSampledEmissionsFile, "output_sampled_vehicle_operating_mode.csv","w");
			fopen_s(&fpSampledCycleEmissionRateFile, "output_sampled_vehicle_cycle_emission_rate.csv","w");
			if (fpSampledEmissionsFile)
			{
				fprintf(fpSampledEmissionsFile, "Vehicle_id,Vehicle_Type,Vehicle_Age,Hour,Min,FromNodeNumber,ToNodeNumber,LinkType,TimeOnThisLinkInSecond,TripDurationInSecond,Speed(mph),Acceleration(mph/s),VSP,VSP_bin,Speed_bin,OperatingMode,Energy,CO2,NOX,CO,HC,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)\n");
			}

			if (fpSampledCycleEmissionRateFile)
			{
				fprintf(fpSampledCycleEmissionRateFile, "Vehicle_id,Vehicle_Type,Vehicle_Age,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)\n");
			}
		}

		if(st2 != NULL)
		{
			cout << "writing output_sampled_vehicle_operating_mode.csv..." << endl;
			std::map<int, DTAVehicle*>::iterator iterVM;
			//calculate the toll cost and emission cost
			for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
			{
				DTAVehicle* pVehicle = iterVM->second;

				/// print out second by second profile
				if(g_OutputSecondBySecondEmissionData)
				{
					if (pVehicle->m_bDetailedEmissionOutput == false)
					{
						continue;
					}

					numOfSamples++;

					if(numOfSamples % 500 == 0)
					{
						cout << " " << numOfSamples << " emission samples have been generated ..." << endl;
					}

					int VehicleSimulationStartTime;
					int prevFromNodeNumber = -1;
					int prevToNodeNumber = -1;
					int prevLinkType = -1;
					int currLinkType = -1;
					bool isNewLink = false;

					//ReconstructTrajectory(pVehicle->m_VehicleType, pVehicle->m_SpeedProfile);
					int speedProfileSize = pVehicle->m_SpeedProfile.size();
					int speedProfileCounter = 0;
					float prev_speed = 0.0f;

					float prev_acceleration = -99999.0f; //in meter/sec^2
					float prev_prev_acceleration = -99999.0f; //in meter/sec^2

					float totalCO2 = 0.0f;
					float totalNOX = 0.0f;
					float totalCO = 0.0f;
					float totalHC = 0.0f;

					float AverageSpeed = 0.0f;
					if (g_EmissionSmoothVehicleTrajectory)
					{
						SmoothVehicleTrajectory(pVehicle->m_AgentID, pVehicle->m_SpeedProfile);
					}

					for(std::map<int, VehicleSpeedProfileData>::iterator iter_s  =  pVehicle->m_SpeedProfile.begin();
						iter_s != pVehicle->m_SpeedProfile.end();
						iter_s++, speedProfileCounter++)
					{  
						VehicleSpeedProfileData element = iter_s->second;

						if (fpSampledEmissionsFile)
						{
							//Get the link type by from and to node numbers
							if (prevFromNodeNumber != element.FromNodeNumber || prevToNodeNumber != element.ToNodeNumber)
							{
								if(g_LinkMap.find(GetLinkStringID(element.FromNodeNumber,element.ToNodeNumber)) != g_LinkMap.end())
								{
									DTALink* pLink = g_LinkMap[GetLinkStringID(element.FromNodeNumber,element.ToNodeNumber)];
									currLinkType = prevLinkType = pLink->m_link_type;
								}
								else
								{
									currLinkType = prevLinkType = -1;
								}
								//isNewLink = true;

								prevFromNodeNumber = element.FromNodeNumber;
								prevToNodeNumber = element.ToNodeNumber;
							}
							else
							{
								isNewLink = false;								
							}

							// If it is the first second on the first link or the last second on the last link, recalculate the acceleration rate, VSP and emissions
							//if (iter_s == pVehicle->m_SpeedProfile.begin() || speedProfileCounter + 1 == speedProfileSize) 
							//{
							//	isNewLink = false;

							VehicleSimulationStartTime = pVehicle->m_SpeedProfile.begin()->first;

							float acceleration = iter_s->second.Acceleration * 0.44704f;
								//if (iter_s == pVehicle->m_SpeedProfile.begin())
								//{
								//	acceleration = iter_s->second.Speed * 0.44704f; //mph to meter/sec^2
								//}

								//if (speedProfileCounter + 1 == speedProfileSize)
								//{
								//	acceleration = -iter_s->second.Speed * 0.44704f;
								//	element.Speed = iter_s->second.Speed = 0.0f;
								//}

								float vsp = 0;
								int OperatingMode = ComputeOperatingModeFromSpeed(vsp,iter_s->second.Speed * 0.44704f, acceleration,0,pVehicle->m_VehicleType);

								if (acceleration <= -2)
								{
									OperatingMode = 0;
								}
								else
								{
									if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
									{
										if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
										{
											OperatingMode = 0;
										}
									}
								}

								if (iter_s->second.Speed >= -1 && iter_s->second.Speed < 1)
								{
									OperatingMode = 1;
								}

								pVehicle->m_OperatingModeCount[element.OperationMode]--;
								pVehicle->m_OperatingModeCount[OperatingMode]++;							

								int vehicle_type = pVehicle->m_VehicleType;
								int age = pVehicle->m_Age;
								float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600;
								float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600;
								float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600;
								float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600;
								float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600;

								element.Acceleration = iter_s->second.Acceleration; //meter/sec^2 to feet/sec^2
								element.VSP = vsp;
								element.OperationMode = OperatingMode;
								element.VSPBinNo = GetVSPBinNo(vsp, element.Speed);
								element.SpeedBinNo = GetSpeedBinNo(element.Speed);
								element.Energy = Energy;
								element.CO2 = CO2;
								element.NOX = NOX;
								element.CO = CO;
								element.HC = HC;
							//}

							// First second on the new Link, recalculate the acceleration rate, VSP and emissions
							//if (isNewLink)
							//{
							//	std::map<int, VehicleSpeedProfileData>::iterator prev_iter_s = --iter_s;
							//	iter_s++;
							//	float acceleration = (iter_s->second.Speed - prev_speed) * 0.44704f;
							//	float vsp = 0;
							//	int OperatingMode = ComputeOperatingModeFromSpeed(vsp,iter_s->second.Speed * 0.44704f, acceleration,0,pVehicle->m_VehicleType);
							//	if (acceleration <= -2)
							//	{
							//		OperatingMode = 0;
							//	}
							//	else
							//	{
							//		if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
							//		{
							//			if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
							//			{
							//				OperatingMode = 0;
							//			}
							//		}
							//	}

							//	if (iter_s->second.Speed >= -1 && iter_s->second.Speed < 1)
							//	{
							//		OperatingMode = 1;
							//	}

							//	pVehicle->m_OperatingModeCount[element.OperationMode]--;
							//	pVehicle->m_OperatingModeCount[OperatingMode]++;							

							//	int vehicle_type = pVehicle->m_VehicleType;
							//	int age = pVehicle->m_Age;
							//	float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600;
							//	float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600;
							//	float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600;
							//	float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600;
							//	float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600;

							//	element.Acceleration = acceleration * 3.28084f; // meter/sec^2 to feet/sec^2
							//	element.VSP = vsp;
							//	element.OperationMode = OperatingMode;
							//	element.VSPBinNo = GetVSPBinNo(vsp, element.Speed);
							//	element.SpeedBinNo = GetSpeedBinNo(element.Speed);
							//	element.Energy = Energy;
							//	element.CO2 = CO2;
							//	element.NOX = NOX;
							//	element.CO = CO;
							//	element.HC = HC;
							//}

							fprintf(fpSampledEmissionsFile, "%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%5.2f",
								pVehicle->m_AgentID,
								pVehicle->m_VehicleType,
								pVehicle->m_Age,
								(int)(iter_s->first / 3600 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),
								((int)(iter_s->first / 60 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)) % 60,								
								element.FromNodeNumber,
								element.ToNodeNumber,
								(currLinkType == -1) ? "Unknown":g_LinkTypeMap[currLinkType].link_type_name.c_str(),
								element.TimeOnThisLinkInSecond,
								(int)((iter_s->first - VehicleSimulationStartTime) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),
								element.Speed
								);

							if (iter_s != pVehicle->m_SpeedProfile.begin())
							{
								totalCO2 += element.CO2;
								totalNOX += element.NOX;
								totalCO += element.CO;
								totalHC += element.HC;

								fprintf(fpSampledEmissionsFile,",%5.2f,%f,%s,%s,%d,%f,%f,%f,%f,%f",
									element.Acceleration,
									element.VSP,
									element.GetVSPBinNoString(),
									element.GetSpeedBinNoString(),
									element.OperationMode,
									element.Energy,
									element.CO2, 
									element.NOX, 
									element.CO,
									element.HC
									);
							}

							AverageSpeed += element.Speed;

							if (speedProfileCounter + 1 == speedProfileSize)
							{
								fprintf(fpSampledCycleEmissionRateFile, "%d,%d,%d", 
									pVehicle->m_AgentID,
									pVehicle->m_VehicleType,
									pVehicle->m_Age
									);

								float average_speed =  AverageSpeed / speedProfileSize /*pVehicle->m_Distance / (pVehicle->m_ArrivalTime - pVehicle->m_DepartureTime) * 60.0f*/;
								float base_average_speed;
								//Average speed for base cycle is 21.2 mph
								CCycleAverageEmissionFactor BaseCaseEmissionsFactors = CycleAverageEmissionFactorMatrix[pVehicle->m_VehicleType][pVehicle->m_Age];
								float speed_ratio = 21.2 / average_speed;

								float correctedCO2 = BaseCaseEmissionsFactors.emissionFactor_CO2 * (totalCO2 / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_CO2] * speed_ratio;
								float correctedNOX = BaseCaseEmissionsFactors.emissionFactor_NOX * (totalNOX / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_NOX] * speed_ratio;
								float correctedCO = BaseCaseEmissionsFactors.emissionFactor_CO * (totalCO / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_CO] * speed_ratio;
								float correctedHC = BaseCaseEmissionsFactors.emissionFactor_HC * (totalHC / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_HC] * speed_ratio;

								fprintf(fpSampledEmissionsFile, ",%f,%f,%f,%f,%f", average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);
								fprintf(fpSampledCycleEmissionRateFile, ",%f,%f,%f,%f,%f", average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);


								fprintf(fpSampledCycleEmissionRateFile,"\n");
							}
							fprintf(fpSampledEmissionsFile,"\n");

							prev_speed = element.Speed;
							prev_prev_acceleration = prev_acceleration;
							prev_acceleration = element.Acceleration / 3.28084f;
						}
					}
				}

				fprintf(st2, "Vehicle=,%d,Type=,%d\n", pVehicle->m_AgentID,pVehicle->m_VehicleType);
				fprintf(st2, "# of operating mode data points =,%d\n", pVehicle->m_OperatingModeCount.size());
				fprintf(st2, "Energy:, %f\n", pVehicle->Energy);
				fprintf(st2, "CO2:, %f\n", pVehicle->CO2 );
				fprintf(st2, "CO:, %f\n", pVehicle->CO );
				fprintf(st2, "HC:, %f\n", pVehicle->HC );
				fprintf(st2, "NOX:, %f\n", pVehicle->NOX);

				for(std::map<int, int>::iterator iterOP  =  pVehicle->m_OperatingModeCount.begin(); iterOP != pVehicle->m_OperatingModeCount.end (); iterOP++)
				{
					fprintf(st2, "op=,%d,count=,%d\n", iterOP->first ,iterOP->second );
				}

				for(std::map<int, int>::iterator iter_speed  =  pVehicle->m_SpeedCount.begin(); iter_speed != pVehicle->m_SpeedCount.end (); iter_speed++)
				{
					fprintf(st2, "speed=,%d,count=,%d\n", iter_speed->first ,iter_speed->second );
				}

				fprintf(st2,"\n");
			}

			if (fpSampledEmissionsFile)
			{
				fclose(fpSampledEmissionsFile);
			}

			if (fpSampledCycleEmissionRateFile)
			{
				fclose(fpSampledCycleEmissionRateFile);
			}

			fclose(st2);

			cout << numOfSamples << " vehicles are sampled to output_sampled_vehicle_operating_mode.csv." << endl;
		}

	}
#endif 
}

//void SmoothCumulativeCounts(float* countArray, int size)
//{
//    if (countArray != NULL && size > 0)
//    {
//        float value = countArray[0];
//        int start_idx = 0;
//        int end_index = 0;
//        for (int n = 1; n < size; n++)
//        {
//            if (countArray[n] == value)
//            {
//                countArray[n] = 0;
//                end_index++;
//            }
//            else
//            {
//                if (start_idx < end_index)
//                {
//                    float increment = (countArray[n] - value) / (end_index - start_idx + 1);
//                    for (int i = start_idx + 1; i <= end_index; i++)
//                    {
//                        countArray[i] = countArray[i - 1] + increment;
//                    }
//                }
//                value = countArray[n];
//                start_idx = end_index = n;
//            }
//        }
//    }
//}

//void DTALink::ThreeDetectorVehicleTrajectorySimulation()
//{
//	int initialTime = -1;
//	int endTime = -1;
//
//
//	for (int LaneNo = 0; LaneNo < m_OutflowNumLanes; LaneNo++)
//	{
//		sort(m_VehicleDataVector[LaneNo].LaneData.begin(), m_VehicleDataVector[LaneNo].LaneData.end(), vehicleCF_sort_function);
//		initialTime = m_VehicleDataVector[LaneNo].LaneData[0].StartTime_in_SimulationInterval;
//		endTime = m_VehicleDataVector[LaneNo].LaneData[m_VehicleDataVector[LaneNo].LaneData.size() -1].EndTime_in_SimulationInterval;
//		int countArraySize = endTime - initialTime + 1;
//		float* downstreamLaneCounts = new float[countArraySize];
//		memset(downstreamLaneCounts, 0, sizeof(float) * countArraySize);
//
//		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
//		{
//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;
//			downstreamLaneCounts[end_time - initialTime]++;
//		}
//
//		for (int i=1; i < countArraySize; i++)
//		{
//			downstreamLaneCounts[i] += downstreamLaneCounts[i -1];
//		}
//
//		SmoothCumulativeCounts(downstreamLaneCounts, countArraySize);
//
//
//		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
//		{
//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;
//			
//			float link_length_in_meter = m_Length * 1609.344f;
//			float position = 0.0f;
//			int upstreamCountNum = v;
//
//			DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];
//			pVehicle->m_PrevSpeed = 0.0f;
//			float prevPosition = 0.0f;
//			float prev_acceleration = -99999.0f; //acceleration at t-1
//			float prev_prev_acceleration = -99999.0f; //acceleration at t-2
//
//			for(int t = 1; t < end_time - start_time; t+= 1) 				
//			{
//				if (position < link_length_in_meter)
//				{
//					int bwtt = (int)((link_length_in_meter - position) / (this->m_BackwardWaveSpeed * 1609.34f / 3600.0f)) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
//					int t_at_downstream = t - bwtt;
//					if (t_at_downstream < 0)
//					{
//						position = min(link_length_in_meter, position + this->m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);						
//					}
//					else
//					{
//						if (upstreamCountNum <= downstreamLaneCounts[t_at_downstream] + this->m_KJam * (link_length_in_meter - position) * 0.000621371f)
//						{
//							position = min(link_length_in_meter, position + this->m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);							
//						}
//					}
//				}
//				else
//				{
//					break;
//				}
//
//				if (t % NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND == 0)
//				{
//
//					float SpeedBySecond = position - prevPosition;					
//					float acceleration = SpeedBySecond - pVehicle->m_PrevSpeed;
//
//					float vsp = 0;
//					int OperatingMode = ComputeOperatingModeFromSpeed(vsp,SpeedBySecond, acceleration,0,pVehicle->m_VehicleType);
//
//					if (acceleration <= -2)
//					{
//						OperatingMode = 0;
//					}
//					else
//					{
//						if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
//						{
//							if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
//							{
//								OperatingMode = 0;
//							}
//						}
//					}
//
//					if ((SpeedBySecond / 0.44704f) >= -1.0f && (SpeedBySecond / 0.44704f) < 1.0f)
//					{
//						OperatingMode = 1;
//					}
//
//					pVehicle->m_OperatingModeCount[OperatingMode]++;
//					int integer_speed = SpeedBySecond / 0.44704f;  // convert meter per second to mile per hour
//					pVehicle->m_SpeedCount[integer_speed]++;
//
//
//					pVehicle->m_PrevSpeed  = SpeedBySecond;
//
//					int vehicle_type = pVehicle->m_VehicleType;
//
//					int age = pVehicle->m_Age ;
//
//					int time_in_min = t / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / 60;
//
//					if(EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
//					{
//						cout << "Emission rate data are not available for vehicle type = " <<  vehicle_type 
//							<< ", operating mode = " << OperatingMode 
//							<< " and age = " << age << endl;
//						g_ProgramStop();
//					}
//
//					float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600.0f;
//					float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600.0f;
//					float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600.0f;
//					float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600.0f;
//					float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600.0f;
//
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].Energy+= Energy;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO2+= CO2;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].NOX+= NOX;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO+= CO;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].HC+= HC;
//
//					if (pVehicle->m_AgentID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || 
//						(g_OutputSecondBySecondEmissionData && pVehicle->m_DepartureTime >= g_start_departure_time_for_output_second_by_second_emission_data 
//						&& pVehicle->m_DepartureTime <= g_end_departure_time_for_output_second_by_second_emission_data
//						&& pVehicle->m_bDetailedEmissionOutput))  // record data
//					{
//						VehicleSpeedProfileData element;
//						element.FromNodeNumber = this->m_FromNodeNumber ;
//						element.ToNodeNumber  = this->m_ToNodeNumber ;
//						element.TimeOnThisLinkInSecond = (t-start_time)/NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
//						element.Speed = SpeedBySecond / 0.44704f; // km/h to mph
//						element.Acceleration = acceleration * 3.28084; //meter/sec^2 to feet/sec^2
//						element.OperationMode = OperatingMode;
//						element.VSP  = vsp;
//						element.SpeedBinNo = GetSpeedBinNo(element.Speed);
//						element.VSPBinNo =  GetVSPBinNo(vsp, element.Speed);
//						element.Energy = Energy;
//						element.CO2 = CO2;
//						element.NOX = NOX;
//						element.CO = CO;
//						element.HC = HC;
//
//						pVehicle->m_SpeedProfile[t] = element;
//					}
//
//					prevPosition = position;
//					prev_prev_acceleration = prev_acceleration;
//					prev_acceleration = acceleration * 0.44704f;
//				}				
//			}
//		}
//
//
//		delete [] downstreamLaneCounts;
//	}
//}
//

void DTALink::ComputeVSP()  // VSP: vehicle specific power
{
#ifdef _large_memory_usage

	const int Max_Time_Step_In_A_Cycle = 100;
	const int Max_Simulation_Steps = 1800 * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
	static float positionBuffer1[Max_Simulation_Steps];
	static float positionsBuffer2[Max_Simulation_Steps];	
	float *pLeaderPositions = positionBuffer1;
	float *pFollowerPositions = positionsBuffer2;

	//FILE* output = NULL;

	//if (this->m_FromNodeNumber == 2)
	//{
	//	fopen_s(&output,"trajectory.csv","w");
	//}

	//#pragma omp parallel for 
	for(int LaneNo = 0; LaneNo < m_OutflowNumLanes; LaneNo++)  // for each lane 
	{
		// step 1: sort vehicles arrival times
		sort(m_VehicleDataVector[LaneNo].LaneData.begin(), m_VehicleDataVector[LaneNo].LaneData.end(), vehicleCF_sort_function);

		// step 2: car following simulation
		float link_length_in_meter = m_Length * 1609.344f; //1609.344f: mile to meters

		int leaderInTime = -1;
		int leaderOutTime = -1;

		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
		{
			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;
			DTAVehicle* pVehicle = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];
			// second do loop: start_time to end time
			for(int t = start_time; t <= end_time; t+=1) 				
			{
				//calculate free-flow position
				//xiF(t) = xi(t-τ) + vf(τ)
				pFollowerPositions[t - start_time] = 0.0f;

				if(t > start_time)
				{
					if (v == 0)
					{
						pFollowerPositions[t - start_time] = max(0, min(link_length_in_meter, pFollowerPositions[t - start_time - 1] +  m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval));
					}
					else
					{
						//calculate congested position if it is not the first vehicle
						//xiC(t) = xi-1(t-τ) - δ
						int time_t_minus_tau = t - m_VehicleDataVector[LaneNo].LaneData[v].TimeLag_in_SimulationInterval; // need to convert time in second to time in simulation time interval
						if (time_t_minus_tau < leaderInTime)
						{
							pFollowerPositions[t - start_time] = 0.0f;
						}
						else
						{
							if(time_t_minus_tau >= 0 && time_t_minus_tau <= leaderOutTime)  // the leader has not reached destination yet
							{
								// vehicle v-1: previous car
								//float CongestedDistance = VechileDistanceAry[v-1][time_t_minus_tau%Max_Time_Step_In_A_Cycle] - m_VehicleDataVector[LaneNo].LaneData[v].CriticalSpacing_in_meter;
								float CongestedDistance = max(0, pLeaderPositions[time_t_minus_tau - leaderInTime] - m_VehicleDataVector[LaneNo].LaneData[v].CriticalSpacing_in_meter);
								float potentialDistance = min(link_length_in_meter, pFollowerPositions[t - start_time - 1] + m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval);
								// xi(t) = min(xAF(t), xAC(t))
								//if (FollowerPositions[t % Max_Simulation_Steps] > CongestedDistance /*&& CongestedDistance >= FollowerPositions[(t-1) % Max_Simulation_Steps]*/)
								//{
								//	FollowerPositions[t % Max_Simulation_Steps] = min(CongestedDistance, FollowerPositions[(t-1) % Max_Simulation_Steps]);
								//}

								pFollowerPositions[t - start_time] = max(pFollowerPositions[t - start_time-1], min(CongestedDistance, potentialDistance, link_length_in_meter)); 
								//next step distance should be no less than the current distnance
								// next step distance should be min of (congestioned distance due to backward wave, and free-flow distance)
							}
							else
							{
								pFollowerPositions[t - start_time] = min(link_length_in_meter, pFollowerPositions[t - start_time - 1] +  m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval);
							}
						}

						if (t == end_time)
						{
							pFollowerPositions[t - start_time] = link_length_in_meter;
						}
						if (pVehicle->m_AgentID == g_TargetVehicleID_OutputSecondBySecondEmissionData)
						{
							if (t != start_time)
							{
								int second_from_time0 = t / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
								int hour = second_from_time0 / 3600;
								int min = (second_from_time0 - hour * 3600 ) / 60;
								int second = (second_from_time0 - hour * 3600- min * 60 );

								ASSERT(second >= 0);
								double moving_dist =  pFollowerPositions[t - start_time] - pFollowerPositions[t - start_time - 1];
								double moving_speed = moving_dist * 3600 * 0.000621371; // moving distance from sec, to hour, and distance converted to mile
								_proxy_emission_log(0, pVehicle->m_AgentID, "%d-> %d, lane:%d, t:%d (%02d:%02d:%02d), dist:%5.2f, moving_dist:%5.2f (%5.3f mph)\n", this->m_FromNodeNumber, this->m_ToNodeNumber,
									LaneNo, t, hour, min,second,
									pFollowerPositions[t - start_time], moving_dist, moving_speed);
							}
							else
							{
								_proxy_emission_log(0, pVehicle->m_AgentID, "%d-> %d, lane:%d, t:%d, dist: %5.2f, moving_dist: %5.2f (0 mph)\n", this->m_FromNodeNumber, this->m_ToNodeNumber,
									LaneNo, t, pFollowerPositions[t - start_time], 0.0f);
							}
						}
					}
				}




				//(v >= 0 && v <= 400 && LaneNo == 0)
				//{
				//	DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];

				//	if (output != NULL)
				//	{
				//		if (t != start_time)
				//		{
				//			fprintf(output,"%d, %d, %d, %d, %5.2f,%5.2f\n",v, pVehicle->m_AgentID, LaneNo, t, pFollowerPositions[t - start_time], pFollowerPositions[t - start_time] - pFollowerPositions[t- start_time - 1]);
				//		}
				//		else
				//		{
				//			fprintf(output,"%d, %d, %d, %d,%5.2f,%5.2f\n",v, pVehicle->m_AgentID, LaneNo, t, pFollowerPositions[t - start_time], 0.0f);
				//		}
				//	}
				//}
			}  // for each time step

			if (g_EmissionSmoothVehicleTrajectory)
			{
				MovingAverage(pFollowerPositions, end_time - start_time + 1);
			}

			float prev_acceleration = -99999.0f; //acceleration at t-1
			float prev_prev_acceleration = -99999.0f; //acceleration at t-2
			for (int t = start_time; t <= end_time; t += NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)
			{
				// output speed per second
				// for active vehicles (with positive speed or positive distance
				float SpeedBySecond = m_SpeedLimit * 0.44704f; // 1 mph = 0.44704 meters per second

				if (t == start_time) SpeedBySecond = 0.0f;

				if(t >= start_time + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)  // not the first second
				{
					//SpeedBySecond = (VechileDistanceAry[v][t%Max_Time_Step_In_A_Cycle] - VechileDistanceAry[v][max(0,t-NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)%Max_Time_Step_In_A_Cycle]);						
					SpeedBySecond = min(m_SpeedLimit * 0.44704f,
						max(0, pFollowerPositions[t - start_time] - pFollowerPositions[t - start_time - NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND]));
				}

				if (t + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND >= end_time)
				{
					SpeedBySecond = m_SpeedLimit * 0.44704f;
				}

				// different lanes have different vehicle numbers, so we should not have openMP conflicts here
				DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];
				float acceleration = SpeedBySecond - pVehicle->m_PrevSpeed;

				float vsp = 0;
				int OperatingMode = ComputeOperatingModeFromSpeed(vsp,SpeedBySecond, acceleration,0,pVehicle->m_VehicleType);

				if (acceleration <= -2)
				{
					OperatingMode = 0;
				}
				else
				{
					if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
					{
						if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
						{
							OperatingMode = 0;
						}
					}
				}

				if ((SpeedBySecond / 0.44704f) >= -1.0f && (SpeedBySecond / 0.44704f) < 1.0f)
				{
					OperatingMode = 1;
				}

				pVehicle->m_OperatingModeCount[OperatingMode]++;
				int integer_speed = SpeedBySecond / 0.44704f;  // convert meter per second to mile per hour
				pVehicle->m_SpeedCount[integer_speed]++;


				pVehicle->m_PrevSpeed  = SpeedBySecond;

				int vehicle_type = pVehicle->m_VehicleType;

				int age = pVehicle->m_Age ;

				int time_in_min = start_time / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / 60;

				if (EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
				{
					// try without data
					age = 5 * int(age*1.0 / 5 + 0.5);

				}


				if(EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
				{
					cout << "Emission rate data are not available for vehicle type = " <<  vehicle_type 
						<< ", operating mode = " << OperatingMode 
						<< " and age = " << age << endl;
					g_ProgramStop();
				}

				float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy / 3600.0f;
				float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2 / 3600.0f;
				float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX / 3600.0f;
				float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO / 3600.0f;
				float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC / 3600.0f;

					if(time_in_min ==430 && m_FromNodeNumber == 1 && m_ToNodeNumber == 3)
					{
					TRACE("");
					
					}
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].Energy += Energy;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO2 += CO2;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].NOX += NOX;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO += CO;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].HC += HC;

				if (pVehicle->m_AgentID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || 
					(g_OutputSecondBySecondEmissionData && pVehicle->m_DepartureTime >= g_start_departure_time_for_output_second_by_second_emission_data 
					&& pVehicle->m_DepartureTime <= g_end_departure_time_for_output_second_by_second_emission_data
					&& pVehicle->m_bDetailedEmissionOutput))  // record data
				{
					VehicleSpeedProfileData element;
					element.FromNodeNumber = this->m_FromNodeNumber ;
					element.ToNodeNumber  = this->m_ToNodeNumber ;
					element.TimeOnThisLinkInSecond = (t - start_time) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					element.Speed = SpeedBySecond / 0.44704f; // km/h to mph
					element.Acceleration = acceleration * 3.28084; //meter/sec^2 to feet/sec^2
					element.OperationMode = OperatingMode;
					element.VSP  = vsp;
					element.SpeedBinNo = GetSpeedBinNo(element.Speed);
					element.VSPBinNo =  GetVSPBinNo(vsp, element.Speed);
					element.Energy = Energy;
					element.CO2 = CO2;
					element.NOX = NOX;
					element.CO = CO;
					element.HC = HC;

					pVehicle->m_SpeedProfile[t] = element;
				}

				prev_prev_acceleration = prev_acceleration;
				prev_acceleration = acceleration * 0.44704f;				
			}

			float *tmp = pLeaderPositions;
			pLeaderPositions = pFollowerPositions;
			pFollowerPositions = tmp;
			leaderInTime = start_time;
			leaderOutTime = end_time;
		} // for each vehicle
	} //for each lane

	//if (output != NULL)
	//{
	//	fclose(output);
	//}

	//delete [] LeaderPositions;
	//delete [] FollowerPositions;
#endif 
}



void DTALink::ComputeVSP_FastMethod()  // VSP: vehicle specific power
{
#ifdef _large_memory_usage_emission
	// step 1: car following simulation
	float link_length_in_meter = m_Length * 1609.344f; //1609.344f: mile to meters

	float BackwardWaveSpeed_in_meter_per_second = m_BackwardWaveSpeed  * 1609.344f / 3600;
	for (int LaneNo = 0; LaneNo < m_OutflowNumLanes; LaneNo++)
	{
		int v;
		for (v = 0; v<m_VehicleDataVector[LaneNo].LaneData.size(); v++)
		{
			float distance = 0;  // from the starting point of the link to the current location
			float prev_distance = 0;

			DTAVehicle* pVehicle = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];

			std::map<int, int> l_OperatingModeCount;

			int SequentialLinkNo = m_VehicleDataVector[LaneNo].LaneData[v].SequentialLinkNo;

			for (int t = m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
				t < m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval; t += 1 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)
			{
				//calculate free-flow position
				//xiF(t) = xi(t-τ) + vf(τ)
				prev_distance = distance;
				distance = min(link_length_in_meter, distance + m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval);
				//					TRACE("veh %d, time%d,%f\n",v,t,VechileDistanceAry[v][t]);

				//calculate congested position
				float BWTT_in_second = (link_length_in_meter - distance) / max(0.001, BackwardWaveSpeed_in_meter_per_second);
				int time_t_minus_BWTT = t - BWTT_in_second*NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND; // need to convert time in second to time in simulation time interval
				int DTASimulationTimeIntervalNo = time_t_minus_BWTT*g_DTASimulationInterval;

				if (DTASimulationTimeIntervalNo < 0)
					DTASimulationTimeIntervalNo = 0;
				if (DTASimulationTimeIntervalNo >= m_CumuDeparturelFlow.size() - 1)
					DTASimulationTimeIntervalNo = m_CumuDeparturelFlow.size() - 2;

				int DownstreamCumulativeDepartureCount = m_CumuDeparturelFlow[DTASimulationTimeIntervalNo];
				// reference: Newell's 3-detector theory 

				float SpeedBySecond = (distance - prev_distance)* 0.44704f; // // 1 mph = 0.44704 meters per second

				// output speed per second

				if (t%NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND == 0)  // per_second
				{
					// for active vehicles (with positive speed or positive distance

					// different lanes have different vehicle numbers, so we should not have openMP conflicts here
					float accelation = SpeedBySecond - pVehicle->m_PrevSpeed;

					int OperatingMode = ComputeOperatingModeFromSpeed(SpeedBySecond, accelation);
					pVehicle->m_OperatingModeCount[OperatingMode] += 1;
					pVehicle->m_PrevSpeed = SpeedBySecond;

					l_OperatingModeCount[OperatingMode] += 1;

				}

			}  // for each time t

			// calculate subtotal emissions for each vehicle
			CVehicleEmission element(pVehicle->m_VehicleType, pVehicle->m_OperatingModeCount, pVehicle->m_Age);

			// add the vehicle specific emissions to link-based emission statistics
			m_TotalEnergy += element.Energy;
			m_CO2 += element.CO2;
			m_NOX += element.NOX;
			m_CO += element.CO;
			m_HC += element.HC;

		}  // for active vehicle

	} // for each lane

#endif
}









bool g_VehicularSimulation_version_2(int DayNo, double CurrentTime, int meso_simulation_time_interval_no, int TrafficFlowModelFlag)
{
	std::set<DTANode*>::iterator iterNode;
	std::set<DTALink*>::iterator iterLink;
	std::map<int, DTAVehicle*>::iterator iterVM;
	int   PathNodeList[MAX_NODE_SIZE_IN_A_PATH] = { 0 };

	int time_stamp_in_min = int(CurrentTime + 0.0001);

	std::list<struc_vehicle_item>::iterator vii;

	int vehicle_id_trace = 8297;
	int link_id_trace = -1;
	bool debug_flag = true;


	//DTALite:
	// vertical queue data structure
	// each link  ExitQueue, EntranceQueue: (VehicleID, ReadyTime)

	// step 0: /// update VMS path provision

	if (meso_simulation_time_interval_no % (g_information_updating_interval_of_VMS_in_min * 10) == 0)  //regenerate shortest path tree for VMS every g_information_updating_interval_of_VMS_in_min 
	{
		for (unsigned li = 0; li< g_LinkVector.size(); li++)
		{
			DTALink * pLink = g_LinkVector[li];
			for (unsigned int m = 0; m<pLink->MessageSignVector.size(); m++)
			{

				if (g_LinkVector[li]->MessageSignVector[m].StartDayNo <= DayNo &&
					DayNo <= pLink->MessageSignVector[m].EndDayNo &&
					CurrentTime >= pLink->MessageSignVector[m].StartTime &&
					CurrentTime <= pLink->MessageSignVector[m].EndTime)
				{
					//calculate shortest path
					DTANetworkForSP network(g_NodeVector.size(), g_LinkVector.size(), 1, g_AdjLinkSize);
					float COV_perception_erorr = g_VMSPerceptionErrorRatio;
					network.BuildTravelerInfoNetwork(DayNo, CurrentTime, COV_perception_erorr);
					network.TDLabelCorrecting_DoubleQueue(g_LinkVector[li]->m_ToNodeID, 0, CurrentTime, 1, 10, false, false, false);

					TRACE("\nVMS %d -> %d", g_LinkVector[li]->m_FromNodeNumber, pLink->m_ToNodeNumber);
					// update node predecessor
					g_LinkVector[li]->MessageSignVector[m].Initialize(g_NodeVector.size(), network.NodePredAry, network.LinkNoAry);

				}
			}
		}

	}
	// load vehicle into network



	// step 1: scan all the vehicles, if a vehicle's start time >= CurrentTime, and there is available space in the first link,
	// load this vehicle into the ready queue

	// comment: we use map here as the g_VehicleMap map is sorted by departure time.
	// At each iteration, we start  the last loaded id, and exit if the departure time of a vehicle is later than the current time.

	for (iterVM = g_VehicleMap.find(g_LastLoadedVehicleID); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVeh = iterVM->second;
		if (pVeh->m_bLoaded == false && g_floating_point_value_less_than_or_eq_comparison(pVeh->m_DepartureTime, CurrentTime))  // not being loaded
		{
		
			if (pVeh->m_NodeSize >= 2)  // with physical path
			{
				int FirstLink = pVeh->m_LinkAry[0].LinkNo;

				DTALink* p_link = g_LinkVector[FirstLink];

				if (p_link != NULL)
				{
					struc_vehicle_item vi;
					vi.veh_id = pVeh->m_AgentID;
					g_LastLoadedVehicleID = pVeh->m_AgentID;

					vi.event_time_stamp = pVeh->m_DepartureTime + p_link->GetFreeMovingTravelTime(TrafficFlowModelFlag, CurrentTime);  // unit: min
					pVeh->m_bLoaded = true;
					pVeh->m_SimLinkSequenceNo = 0;

					if (debug_flag && vi.veh_id == vehicle_id_trace)
						TRACE("Step 1: Load vhc %d to link %d with departure time time %5.2f -> %5.2f\n", vi.veh_id, FirstLink, pVeh->m_DepartureTime, vi.event_time_stamp);

					p_link->LoadingBuffer.push_back(vi);  // need to fine-tune
					g_Number_of_GeneratedVehicles += 1;
					g_NetworkMOEAry[time_stamp_in_min].Flow_in_a_min += 1;
					g_NetworkMOEAry[time_stamp_in_min].CumulativeInFlow = g_Number_of_GeneratedVehicles;

				}

			}  // with physical path
			else
			{	//without physical path: skip
				g_LastLoadedVehicleID = pVeh->m_AgentID;
			}


		}

		if (g_floating_point_value_less_than_or_eq_comparison(pVeh->m_DepartureTime, CurrentTime) == false)
		{
			break;
		}


	}

	// loading buffer

	int link_size = g_LinkVector.size();
#pragma omp parallel for
	for (int li = 0; li< link_size; li++)
	{
		DTALink * pLink = g_LinkVector[li];

		while (pLink->LoadingBuffer.size() >0 && pLink->GetNumberOfLanes(DayNo, CurrentTime)>0.01)  // no load vehicle into a blocked link
		{
			struc_vehicle_item vi = pLink->LoadingBuffer.front();
			// we change the time stamp here to reflect the actual loading time into the network, especially for blocked link
			if (vi.event_time_stamp < CurrentTime)
				vi.event_time_stamp = CurrentTime;

			if (debug_flag && vi.veh_id == vehicle_id_trace)
				TRACE("Step 1: Time %f: Load vhc %d from buffer to physical link %d->%d\n", CurrentTime, vi.veh_id, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);

			int NumberOfVehiclesOnThisLinkAtCurrentTime = (int)(pLink->EntranceQueue.size() + pLink->ExitQueue.size());

			// determine link in capacity 
			float AvailableSpaceCapacity = pLink->m_VehicleSpaceCapacity - NumberOfVehiclesOnThisLinkAtCurrentTime;

			// if we use BPR function, no density constraint is imposed --> TrafficFlowModelFlag == 0
			if (TrafficFlowModelFlag == 0 || AvailableSpaceCapacity >= max(2, pLink->m_VehicleSpaceCapacity*(1 - g_MaxDensityRatioForVehicleLoading)))  // at least 10% remaining capacity or 2 vehicle space is left
			{
				pLink->LoadingBuffer.pop_front();
				ASSERT(vi.veh_id >= 0);

				pLink->EntranceQueue.push_back(vi);

				pLink->CFlowArrivalCount += 1;


				int demand_type = g_VehicleMap[vi.veh_id]->m_DemandType;
				pLink->CFlowArrivalCount_DemandType[demand_type] += 1;
				pLink->CFlowArrivalRevenue_DemandType[demand_type] += pLink->GetTollRateInDollar(DayNo, CurrentTime, demand_type);

				DTAVehicle* pVehicle = g_VehicleMap[vi.veh_id];

				// add cumulative flow count to vehicle

				pVehicle->m_TollDollarCost += pLink->GetTollRateInDollar(DayNo, CurrentTime, demand_type);


				if (debug_flag && vi.veh_id == vehicle_id_trace)
					TRACE("Step 1: Time %f: Capacity available, remove vhc %d from buffer to physical link %d->%d\n", CurrentTime, vi.veh_id, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber);

			}
			else
			{
				break;  // physical road is too congested, wait for next time interval to load
			}

		}
	}

	// step 2: move vehicles from EntranceQueue To ExitQueue, if ReadyTime <= CurrentTime)

#pragma omp parallel for
	for (int li = 0; li< link_size; li++)
	{

		DTALink * pLink = g_LinkVector[li];

		while (pLink->EntranceQueue.size() >0)
		{

			struc_vehicle_item vi = pLink->EntranceQueue.front();
			double PlannedArrivalTime = vi.event_time_stamp;

			if (debug_flag && pLink->m_FromNodeNumber == 34 && pLink->m_ToNodeNumber == 30 && CurrentTime >= 860)
			{
				TRACE("Step 3: Time %f, Link: %d -> %d: entrance queue length: %d, exit queue length %d\n",
					CurrentTime, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber,
					pLink->EntranceQueue.size(), pLink->ExitQueue.size());
			}

			if (g_floating_point_value_less_than_or_eq_comparison(PlannedArrivalTime, CurrentTime))  // link arrival time within the simulation time interval
			{
				pLink->EntranceQueue.pop_front();
				pLink->ExitQueue.push_back(vi);

				if (debug_flag && vi.veh_id == vehicle_id_trace)
				{
					TRACE("Step 2: Time %f: Vhc %d moves from entrance queue to exit queue on link %d->%d\n", PlannedArrivalTime, vi.veh_id, g_LinkVector[li]->m_FromNodeNumber, g_LinkVector[li]->m_ToNodeNumber);
					//					link_id_trace = li;
				}

			}
			else
			{
				break;  // the vehicle's actual arrival time is later than the current time, so we exit from the loop, stop searching
			}

		}
	}

	// step 3: determine link in and out capacity

	if (TrafficFlowModelFlag == 0) // BPR function 
	{
		for (unsigned li = 0; li< g_LinkVector.size(); li++)
		{

			// under BPR function, we do not impose the physical capacity constraint but use a BPR link travel time to move vehicles along links	
			g_LinkVector[li]->LinkOutCapacity = 99999;
			g_LinkVector[li]->LinkInCapacity = 99999;

		}
	}
	else // queueing model
	{


#pragma omp parallel for
		for (int li = 0; li< link_size; li++)
		{

			DTALink* pLink = g_LinkVector[li];
			float PerHourCapacity = pLink->GetHourlyPerLaneCapacity(CurrentTime);  // static capacity from BRP function
			float PerHourCapacityAtCurrentSimulatioInterval = PerHourCapacity;

			// freeway 


			if (g_LinkTypeMap[pLink->m_link_type].IsFreeway())
			{
				if (g_StochasticCapacityMode &&  pLink->m_StochaticCapcityFlag >= 1 && meso_simulation_time_interval_no % 150 == 0)  // update stochastic capacity every 15 min
				{
					bool QueueFlag = false;

					if (pLink->ExitQueue.size() > 0)  // vertical exit queue
						QueueFlag = true;

					PerHourCapacityAtCurrentSimulatioInterval = GetStochasticCapacity(QueueFlag, pLink->GetHourlyPerLaneCapacity(CurrentTime));
				}
			}



			float MaximumFlowRate = PerHourCapacityAtCurrentSimulatioInterval *g_DTASimulationInterval / 60.0f*pLink->GetNumberOfLanes(DayNo, CurrentTime); //60 --> cap per min --> unit # of vehicle per simulation interval

			float Capacity = MaximumFlowRate;
			// use integer number of vehicles as unit of capacity

			pLink->LinkOutCapacity = g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(Capacity, li);

			if (debug_flag && pLink->m_FromNodeNumber == 34 && pLink->m_ToNodeNumber == 30 && CurrentTime >= 860)
			{
				TRACE("Step 3: Time %f, Link: %d -> %d: entrance queue length: %d, exit queue length %d, cap = %f, int, %d\n",
					CurrentTime, pLink->m_FromNodeNumber, pLink->m_ToNodeNumber,
					pLink->EntranceQueue.size(), pLink->ExitQueue.size(), Capacity, pLink->LinkOutCapacity);
			}


			int NumberOfVehiclesOnThisLinkAtCurrentTime = (int)(pLink->CFlowArrivalCount - pLink->CFlowDepartureCount);

			float fLinkInCapacity = 99999.0;

			// TrafficFlowModelFlag == 1 -> point queue model
			if (TrafficFlowModelFlag >= 2)  // apply spatial link in capacity for spatial queue models( with spillback and shockwave) 
			{
				// determine link in capacity 
				float AvailableSpaceCapacity = pLink->m_VehicleSpaceCapacity - NumberOfVehiclesOnThisLinkAtCurrentTime;
				fLinkInCapacity = min(AvailableSpaceCapacity, MaximumFlowRate);
				//			TRACE(" time %5.2f, SC: %5.2f, MFR %5.2f\n",CurrentTime, AvailableSpaceCapacity, MaximumFlowRate);

				// the inflow capcaity is the minimum of (1) incoming maximum flow rate (determined by the number of lanes) and (2) available space capacty  on the link.
				// use integer number of vehicles as unit of capacity

				if (TrafficFlowModelFlag == 3 && g_LinkTypeMap[pLink->m_link_type].IsFreeway())  // Newell's model on freeway only
				{
					if (meso_simulation_time_interval_no >= pLink->m_BackwardWaveTimeInSimulationInterval) /// we only apply backward wave checking after the simulation time is later than the backward wave speed interval
					{
						if (debug_flag && link_id_trace == li && CurrentTime >= 480)
						{
							TRACE("Step 3: Time %f, Link: %d -> %d: tracing backward wave\n",
								CurrentTime, g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber,
								pLink->EntranceQueue.size(), pLink->ExitQueue.size());
						}

						int t_residual_minus_backwardwaveTime = max(0, meso_simulation_time_interval_no - pLink->m_BackwardWaveTimeInSimulationInterval) % MAX_TIME_INTERVAL_ADCURVE;

						float VehCountUnderJamDensity = pLink->m_Length * pLink->GetNumberOfLanes(DayNo, CurrentTime) *pLink->m_KJam;
						// when there is a capacity reduction, in the above model, we assume the space capacity is also reduced proportional to the out flow discharge capacity, 
						// for example, if the out flow capacity is reduced from 5 lanes to 1 lanes, say 10K vehicles per hour to 2K vehicles per hour, 
						// our model will also reduce the # of vehicles can be stored on the incident site by the equivalent 4 lanes. 
						// this might cause the dramatic speed change on the incident site, while the upstream link (with the original space capacity) will take more time to propapate the speed change compared to the incident link. 
						// to do list: we should add another parameter of space capacity reduction ratio to represent the fact that the space capacity reduction magnitude is different from the outflow capacity reduction level,
						// particularly for road construction areas on long links, with smooth barrier on the merging section. 
						int N_Arrival_Now_Constrainted = (int)(pLink->m_CumuDeparturelFlow[t_residual_minus_backwardwaveTime] + VehCountUnderJamDensity);  //pLink->m_Length 's unit is mile
						int t_residual_minus_1 = max(0, meso_simulation_time_interval_no - 1) % MAX_TIME_INTERVAL_ADCURVE;

						int N_Now_minus_1 = (int)pLink->m_CumuArrivalFlow[t_residual_minus_1];
						int Flow_allowed = N_Arrival_Now_Constrainted - N_Now_minus_1;
						TRACE("\ntime %d, D:%d,A%d", meso_simulation_time_interval_no, pLink->m_CumuDeparturelFlow[t_residual_minus_1], pLink->m_CumuArrivalFlow[t_residual_minus_1]);

						if (Flow_allowed < 0)
							Flow_allowed = 0;

						if (fLinkInCapacity  > Flow_allowed)
						{
							fLinkInCapacity = Flow_allowed;

							if (Flow_allowed == 0 && N_Arrival_Now_Constrainted > 0)
							{
								if (pLink->m_JamTimeStamp > CurrentTime)
									pLink->m_JamTimeStamp = CurrentTime;

								//							g_LogFile << "Queue spillback"<< CurrentTime <<  g_NodeVector[pLink->m_FromNodeID] << " -> " <<	g_NodeVector[pLink->m_ToNodeID] << " " << pLink->LinkInCapacity << endl;

								// update traffic state
								pLink->m_LinkMOEAry[(int)(CurrentTime)].TrafficStateCode = 2;  // 2: fully congested

								//							TRACE("Queue spillback at %d -> %d\n", g_NodeVector[pLink->m_FromNodeID], g_NodeVector[pLink->m_ToNodeID]);
							}
						}
					}
				}


				float InflowRate = MaximumFlowRate *g_MinimumInFlowRatio;

				if (fLinkInCapacity < InflowRate)  // minimum inflow capacity to keep the network flowing
				{
					fLinkInCapacity = InflowRate;
				}

			}

			// finally we convert the floating-point capacity to integral capacity in terms of number of vehicles
			pLink->LinkInCapacity = g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(fLinkInCapacity, li);

			if (debug_flag && link_id_trace == li)
			{
				TRACE("Step 3: Time %f, Link: %d -> %d: Incapacity %d, %f, OutCapacity: %d\n", CurrentTime, g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber, pLink->LinkInCapacity, fLinkInCapacity, pLink->LinkOutCapacity);
			}
		}


		// distribute link in capacity to different incoming links
		for (unsigned li = 0; li< link_size; li++)
		{
			DTALink* pLink = g_LinkVector[li];
			unsigned int il;
			if (pLink->m_bMergeFlag >= 1)
			{
				int TotalInFlowCount = 0;
				for (il = 0; il< pLink->MergeIncomingLinkVector.size(); il++)
				{
					TotalInFlowCount += g_LinkVector[pLink->MergeIncomingLinkVector[il].m_LinkNo]->ExitQueue.size();  // count vehiciles waiting at exit queue

				}

				if (TotalInFlowCount > pLink->LinkInCapacity)  // demand > supply
				{

					if (pLink->m_bMergeFlag == 1)  // merge with mulitple freeway/onramp links only
					{

						if (g_MergeNodeModelFlag == 0)
						{
							// distribute capacity according to number of lanes, defined previously.
						}

						if (g_MergeNodeModelFlag == 1)
							// distribute capacity according to number of incoming flow waiting on the queue, but it has bias toward on ramp, as the on ramp is using piont queue model now
						{
							for (il = 0; il< pLink->MergeIncomingLinkVector.size(); il++)
							{
								pLink->MergeIncomingLinkVector[il].m_LinkInCapacityRatio = g_LinkVector[pLink->MergeIncomingLinkVector[il].m_LinkNo]->ExitQueue.size()*1.0f / TotalInFlowCount;
							}
						}

						for (il = 0; il< pLink->MergeIncomingLinkVector.size(); il++)
						{
							float LinkOutCapacity = pLink->LinkInCapacity * pLink->MergeIncomingLinkVector[il].m_LinkInCapacityRatio;

							int LinkOutCapacity_int = g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(LinkOutCapacity, li);
							g_LinkVector[pLink->MergeIncomingLinkVector[il].m_LinkNo]->LinkOutCapacity = LinkOutCapacity_int;

						}
					}
					if (pLink->m_bMergeFlag == 2)  // merge with onramp
					{
						// step a. check with flow on onramp
						float MaxMergeCapacity = g_LinkVector[pLink->m_MergeOnrampLinkID]->GetHourlyPerLaneCapacity(CurrentTime)*g_DTASimulationInterval / 60.0f*g_LinkVector[pLink->m_MergeOnrampLinkID]->GetNumberOfLanes(DayNo, CurrentTime) * 0.5f; //60 --> cap per min --> unit # of vehicle per simulation interval
						// 0.5f -> half of the onramp capacity
						// use integer number of vehicles as unit of capacity

						unsigned int MaxMergeCapacity_int = g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp(MaxMergeCapacity, li);

						unsigned int FlowonOnRamp = g_LinkVector[pLink->m_MergeOnrampLinkID]->ExitQueue.size();
						int DownstreamLinkInCapacity = pLink->LinkInCapacity;

						if (FlowonOnRamp > MaxMergeCapacity_int)
							// ramp flow > max merge capacity on ramp  
							// over regions I and II in Dr. Rouphail's diagram
							// capacity on ramp = max merge capacity on ramp
							// capacity on main line = LinkInCapacity - capacity on ramp
							// if flow on main line > capacity on main line  // queue on main line
							// elsewise, no queue on mainline
						{
							g_LinkVector[pLink->m_MergeOnrampLinkID]->LinkOutCapacity = MaxMergeCapacity_int;
							g_LinkVector[pLink->m_MergeMainlineLinkID]->LinkOutCapacity = DownstreamLinkInCapacity - MaxMergeCapacity_int;

						}
						else // ramp flow <= max merge capacity on ramp  // region III  
							// restrict the mainly capacity  // mainly capacity = LinkInCapacity - flow on ramp
						{
							g_LinkVector[pLink->m_MergeOnrampLinkID]->LinkOutCapacity = MaxMergeCapacity_int;
							g_LinkVector[pLink->m_MergeMainlineLinkID]->LinkOutCapacity = DownstreamLinkInCapacity - FlowonOnRamp;

						}
						//		g_LogFile << "merge: mainline capacity"<< CurrentTime << " "  << g_LinkVector [pLink->m_MergeMainlineLinkID] ->LinkOutCapacity << endl;

					}
				}
			}
		}
	}


	// step 4: move vehicles from ExitQueue to next link's EntranceQueue, if there is available capacity
	// NewTime = ReadyTime + FFTT(next link)


	// step 4.1: calculate movement capacity per simulation interval for movements defined in input_movement.csv
	// we will not parallel computing mode for this movement capacity now
	int node_size = g_NodeVector.size();
	for (unsigned node = 0; node < node_size; node++)
	{
		DTANode* pNode = &(g_NodeVector[node]);
		for (std::map<string, DTANodeMovement>::iterator iter = pNode->m_MovementMap.begin();
			iter != pNode->m_MovementMap.end(); iter++)
		{
			if (iter->second.movement_hourly_capacity >= 0)  // if data are available
			{

				float movement_hourly_capacity = iter->second.movement_hourly_capacity / 60 * g_DTASimulationInterval;
				iter->second.movement_capacity_per_simulation_interval = g_GetRandomInteger_SingleProcessorMode(movement_hourly_capacity); // hourly -> min -> 6 seconds);

				iter->second.movement_vehicle_counter = 0; // reset counter of passing vehicles to zero for each simulation interval

				//		TRACE("\n hourly cap: %f, cap per simulation interval %d",movement_hourly_capacity, iter->second.movement_capacity_per_simulation_interval);

			}


		}

	}


	int NextLink;
	DTALink* p_Nextlink;

	//for each node, we scan each incoming link in a randomly sequence, based on meso_simulation_time_interval_no

#pragma omp parallel for
	for (int node = 0; node < node_size; node++)
	{
		int IncomingLinkSize = g_NodeVector[node].m_IncomingLinkVector.size();
		int incoming_link_count = 0;

		int incoming_link_sequence = meso_simulation_time_interval_no%max(1, IncomingLinkSize);  // random start point

		while (incoming_link_count < IncomingLinkSize)
		{

			int li = g_NodeVector[node].m_IncomingLinkVector[incoming_link_sequence];

			DTALink* pLink = g_LinkVector[li];
			//if(debug_flag && (pLink->m_FromNodeNumber  == 10 && pLink->m_ToNodeNumber == 5) && CurrentTime >=480)
			//{
			//	TRACE("Step 3: Time %f, Link: %d -> %d: tracing \n", CurrentTime, g_NodeVector[pLink->m_FromNodeID].m_NodeNumber , g_NodeVector[pLink->m_ToNodeID].m_NodeNumber);
			//}

			// vehicle_out_count is the minimum of LinkOutCapacity and ExitQueue Size

			int vehicle_out_count = pLink->LinkOutCapacity;

			//			g_LogFile << "link out capaity:"<< CurrentTime << " "  << g_NodeVector[pLink->m_FromNodeID] << " ->" << g_NodeVector[pLink->m_ToNodeID]<<" Cap:" << vehicle_out_count<< "queue:" << pLink->ExitQueue.size() << endl;

			if (pLink->ExitQueue.size() <= pLink->LinkOutCapacity)
			{      // under capacity, constrained by existing queue
				vehicle_out_count = pLink->ExitQueue.size();
			}

			list<struc_vehicle_item>::iterator exit_queue_it = pLink->ExitQueue.begin();

			while (pLink->ExitQueue.size() >0 && vehicle_out_count >0 && exit_queue_it != pLink->ExitQueue.end())  // go through
			{
				struc_vehicle_item vi = (*exit_queue_it);

				int vehicle_id = vi.veh_id;

				// record arrival time at the downstream node of current link
				int link_sequence_no = g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo;

				int t_link_arrival_time = 0;
				if (link_sequence_no >= 1)
				{
					t_link_arrival_time = int(g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no - 1].AbsArrivalTimeOnDSN);
				}
				else
				{
					t_link_arrival_time = int(g_VehicleMap[vehicle_id]->m_DepartureTime);
				}
				// not reach destination yet
				int number_of_links = g_VehicleMap[vehicle_id]->m_NodeSize - 1;
				if (g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo < number_of_links - 1)  // not reach destination yet
				{

					// advance to next link

					if (vehicle_id == vehicle_id_trace)
						TRACE("simulation link sequence no. %d", g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo);

					NextLink = g_VehicleMap[vehicle_id]->m_LinkAry[g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo + 1].LinkNo;
					p_Nextlink = g_LinkVector[NextLink];

					if (p_Nextlink == NULL)
					{
						TRACE("Error at vehicle %d,", vehicle_id);
						ASSERT(p_Nextlink != NULL);
					}

					//
					// test if movement capacity available
					//DTANodeMovement movement_element;
					//string movement_id;

					if (g_NodeVector[node].m_MovementMap.size()>0)  // check movement capacity if there is an input movement table
					{

						int from_node = pLink->m_FromNodeNumber;
						int to_node = pLink->m_ToNodeNumber;
						int dest_node = p_Nextlink->m_ToNodeNumber;

						string movement_id = GetMovementStringID(from_node, to_node, dest_node);

						if (g_NodeVector[node].m_MovementMap.find(movement_id) != g_NodeVector[node].m_MovementMap.end()) // the capacity for this movement has been defined
						{

							DTANodeMovement movement_element = g_NodeVector[node].m_MovementMap[movement_id];
							if (movement_element.movement_vehicle_counter >= movement_element.movement_capacity_per_simulation_interval)
							{ // capacity are unavailable

								vehicle_out_count--;

								if (g_FIFOConditionAcrossDifferentMovementFlag == 0)  // not enforcing FIFO conditions 
								{
									++exit_queue_it; // move to the next vehicle

									continue;  // skip the current vehicle, try the next vehicle

								}
								else
								{
									break; // not move any vehicles behind this vehicle
								}



							}
							else
							{
								// move to the next step to check if the link in capacity is available

								//								TRACE("move to the next step to check if the link in capacity is available");

							}

						}

					}  // end of movement checking
					//

					if (p_Nextlink->LinkInCapacity > 0) // if there is available spatial capacity on next link, then move to next link, otherwise, stay on the current link
					{
						float ArrivalTimeOnDSN = 0;
						if (g_floating_point_value_less_than_or_eq_comparison(CurrentTime - g_DTASimulationInterval, vi.event_time_stamp))
							// arrival at previous interval
						{  // no delay 
							ArrivalTimeOnDSN = vi.event_time_stamp;
						}
						else
						{  // delayed at previous time interval, discharge at CurrentTime 
							ArrivalTimeOnDSN = CurrentTime;
						}


						float TimeOnNextLink = 0;

						// update statistics for traveled link
						g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN = ArrivalTimeOnDSN;
						float TravelTime = 0;

						if (link_sequence_no >= 1)
						{
							TravelTime = g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN -
								g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no - 1].AbsArrivalTimeOnDSN;
						}
						else
						{
							TravelTime = g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN -
								g_VehicleMap[vehicle_id]->m_DepartureTime;

						}
						g_VehicleMap[vehicle_id]->m_Delay += (TravelTime - pLink->m_FreeFlowTravelTime);

						// finally move to next link
						g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo = g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo + 1;

						// access VMS information here! p_Nextlink is now the current link
						int IS_id = p_Nextlink->GetInformationResponseID(DayNo, CurrentTime);
						if (IS_id >= 0)
						{
							if (g_VehicleMap[vehicle_id]->GetRandomRatio() * 100 < p_Nextlink->MessageSignVector[IS_id].ResponsePercentage)
							{  // vehicle rerouting

							}

						}



						vi.veh_id = vehicle_id;

						float FFTT = p_Nextlink->GetFreeMovingTravelTime(TrafficFlowModelFlag, CurrentTime);

						vi.event_time_stamp = ArrivalTimeOnDSN + FFTT;

						// remark: when - TimeOnNextLink < 0, it means there are few seconds of left over on current link, which should be spent on next link

						///

						if (debug_flag && p_Nextlink->m_FromNodeNumber == 34 && p_Nextlink->m_ToNodeNumber == 30 && CurrentTime >= 860)
						{
							TRACE("Step 4: Time %f, Link: %d -> %d: vi %d, exit time: %f, FFTT = %f\n",
								CurrentTime, p_Nextlink->m_FromNodeNumber, p_Nextlink->m_ToNodeNumber,
								vi.veh_id, vi.event_time_stamp, FFTT);
						}
						ASSERT(vi.veh_id >= 0);

						p_Nextlink->EntranceQueue.push_back(vi);  // move vehicle from current link to the entrance queue of the next link
						p_Nextlink->CFlowArrivalCount += 1;
						pLink->CFlowDepartureCount += 1;

						int demand_type = g_VehicleMap[vi.veh_id]->m_DemandType;
						p_Nextlink->CFlowArrivalCount_DemandType[demand_type] += 1;
						p_Nextlink->CFlowArrivalRevenue_DemandType[demand_type] += p_Nextlink->GetTollRateInDollar(DayNo, CurrentTime, demand_type);

						DTAVehicle* pVehicle = g_VehicleMap[vi.veh_id];

						pVehicle->m_TollDollarCost += p_Nextlink->GetTollRateInDollar(DayNo, CurrentTime, demand_type);


						if (p_Nextlink->CFlowArrivalCount != (p_Nextlink->CFlowArrivalCount_DemandType[1] + p_Nextlink->CFlowArrivalCount_DemandType[2] + p_Nextlink->CFlowArrivalCount_DemandType[3] + p_Nextlink->CFlowArrivalCount_DemandType[4]))
						{
							//						TRACE("error!");
						}

						if (t_link_arrival_time < pLink->m_LinkMOEAry.size())
						{
							pLink->m_LinkMOEAry[t_link_arrival_time].TotalTravelTime += TravelTime;
							pLink->m_LinkMOEAry[t_link_arrival_time].TotalFlowCount += 1;
						}


						pLink->departure_count += 1;
						pLink->total_departure_based_travel_time += TravelTime;

						if (debug_flag && vi.veh_id == vehicle_id_trace)
						{
							TRACE("Step 4: target vehicle: link arrival time %d, total travel time on current link %d->%d, %f\n", t_link_arrival_time, g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber, pLink->m_LinkMOEAry[t_link_arrival_time].TotalTravelTime);
						}

						//										TRACE("time %d, total travel time %f\n",t_link_arrival_time, pLink->m_LinkMOEAry[t_link_arrival_time].TotalTravelTime);

						p_Nextlink->LinkInCapacity -= 1; // reduce available space capacity by 1

						//remove this vehicle as it has moved to the final destination
						exit_queue_it = pLink->ExitQueue.erase(exit_queue_it);
						vehicle_out_count--;
						continue; // it will not call "++ exit_queue_it" again (in the end of this while loop)



					}
					else
					{
						// no capcity, do nothing, and stay in the vertical exit queue

						if (TrafficFlowModelFlag == 4)  // spatial queue with shock wave and FIFO principle: FIFO, then there is no capacity for one movement, the following vehicles cannot move even there are capacity
						{
							break;
						}

					}

					//			TRACE("move veh %d from link %d link %d %d\n",vehicle_id,pLink->m_LinkNo, p_Nextlink->m_LinkNo);


				}
				else
				{

					// reach destination, increase the counter.
					float ArrivalTimeOnDSN = vi.event_time_stamp;  // no delay at destination node

					// update statistics for traveled link
					int link_sequence_no = g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo;
					g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN = ArrivalTimeOnDSN;
					float TravelTime = 0;

					if (link_sequence_no >= 1)
					{
						TravelTime = g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN -
							g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no - 1].AbsArrivalTimeOnDSN;
					}
					else
					{
						TravelTime = g_VehicleMap[vehicle_id]->m_LinkAry[link_sequence_no].AbsArrivalTimeOnDSN -
							g_VehicleMap[vehicle_id]->m_DepartureTime;

					}
					g_VehicleMap[vehicle_id]->m_Delay += (TravelTime - pLink->m_FreeFlowTravelTime);


					// finally move to next link
					g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo = g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo + 1;


					g_VehicleMap[vehicle_id]->m_ArrivalTime = ArrivalTimeOnDSN;

					g_VehicleMap[vehicle_id]->m_TripTime = g_VehicleMap[vehicle_id]->m_ArrivalTime - g_VehicleMap[vehicle_id]->m_DepartureTime;

					if (debug_flag && vi.veh_id == vehicle_id_trace)
					{
						TRACE("Step 4: time %f, target vehicle reaches the destination, vehicle trip time: %f, # of links  = %d\n",
							ArrivalTimeOnDSN, g_VehicleMap[vehicle_id]->m_TripTime,
							g_VehicleMap[vehicle_id]->m_SimLinkSequenceNo);

						vehicle_id_trace = -1; // not tracking anymore 
					}

					g_VehicleMap[vehicle_id]->m_bComplete = true;
					int OriginDepartureTime = (int)(g_VehicleMap[vehicle_id]->m_DepartureTime);


#pragma omp critical  // keep this section as a single thread as it involves network-wide statistics collection
					{
						g_NetworkMOEAry[OriginDepartureTime].AbsArrivalTimeOnDSN_in_a_min += g_VehicleMap[vehicle_id]->m_TripTime;
						g_Number_of_CompletedVehicles += 1;
						g_NetworkMOEAry[time_stamp_in_min].CumulativeOutFlow = g_Number_of_CompletedVehicles;

					}

					pLink->CFlowDepartureCount += 1;
					pLink->m_LinkMOEAry[t_link_arrival_time].TotalTravelTime += TravelTime;
					pLink->m_LinkMOEAry[t_link_arrival_time].TotalFlowCount += 1;

					//remove this vehicle as it has moved to the next link
					exit_queue_it = pLink->ExitQueue.erase(exit_queue_it);
					vehicle_out_count--;

					continue; // it will not call "++ exit_queue_it" again (in the end of this while loop)
				}



				++exit_queue_it;
			}
			incoming_link_count++;
			incoming_link_sequence = (incoming_link_sequence + 1) % IncomingLinkSize;  // increase incoming_link_sequence by 1 within IncomingLinkSize
		}  // for each incoming link
	} // for each node

	//	step 5: statistics collection
#pragma omp parallel for
	for (int li = 0; li< link_size; li++)
	{

		// Cumulative flow counts

		int t_residual = meso_simulation_time_interval_no % MAX_TIME_INTERVAL_ADCURVE;

		DTALink* pLink = g_LinkVector[li];
		pLink->m_CumuArrivalFlow[t_residual] = pLink->CFlowArrivalCount;
		pLink->m_CumuDeparturelFlow[t_residual] = pLink->CFlowDepartureCount;

		if (meso_simulation_time_interval_no%g_number_of_intervals_per_min == 0)  // per min statistics
		{
			pLink->VehicleCount = pLink->CFlowArrivalCount - pLink->CFlowDepartureCount;

			// queue is the number of vehicles at the end of simulation interval

			pLink->m_LinkMOEAry[time_stamp_in_min].ExitQueueLength = pLink->ExitQueue.size();

			if (pLink->m_LinkMOEAry[time_stamp_in_min].ExitQueueLength >= 1 && pLink->m_LinkMOEAry[time_stamp_in_min].TrafficStateCode != 2)   // not fully congested
				pLink->m_LinkMOEAry[time_stamp_in_min].TrafficStateCode = 1;  // partially congested

			// time_stamp_in_min+1 is because we take the stastistics to next time stamp
			pLink->m_LinkMOEAry[time_stamp_in_min].CumulativeArrivalCount = pLink->CFlowArrivalCount;

			// toll collection 

			//for(int pt = 1; pt < MAX_DEMAND_TYPE_SIZE; pt++)
			//{
			//	pLink->m_LinkMOEAry [time_stamp_in_min].CumulativeArrivalCount_DemandType[pt] = pLink->CFlowArrivalCount_DemandType[pt];
			//	pLink->m_LinkMOEAry [time_stamp_in_min].CumulativeRevenue_DemandType[pt] = pLink->CFlowArrivalRevenue_DemandType[pt];
			//}

			pLink->m_LinkMOEAry[time_stamp_in_min].CumulativeDepartureCount = pLink->CFlowDepartureCount;

			if (debug_flag && link_id_trace == li && vehicle_id_trace >= 0)
			{
				TRACE("step 5: statistics collection: Time %d, link %d -> %d, Cumulative arrival count %d, cumulative departure count %d \n", time_stamp_in_min, g_NodeVector[pLink->m_FromNodeID].m_NodeNumber, g_NodeVector[pLink->m_ToNodeID].m_NodeNumber,
					pLink->m_LinkMOEAry[time_stamp_in_min].CumulativeArrivalCount, pLink->m_LinkMOEAry[time_stamp_in_min].CumulativeDepartureCount);
			}


		}
	}
	return true;
}


