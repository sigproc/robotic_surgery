/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list 
 	of conditions and the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this 
 	list of conditions and the following disclaimer in the documentation and/or other 
 	materials provided with the 	distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors 
 	may be used to endorse or promote products derived from this software without 
 	specific prior written 	permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include "k2_client.h"

std::string topicName = "audio";
int twiceStreamSize = 8200;
int streamSize = 4100;

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startAudio");
	ros::NodeHandle n;
	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	Socket mySocket(serverAddress.c_str(),"9004",twiceStreamSize);
	ros::Publisher audioPub = n.advertise<k2_client::Audio>(topicName,1);
	while(ros::ok())
	{
		mySocket.readData();
		std::string jsonString;
		for(int i=0;i<twiceStreamSize;i+=2)
		{
			jsonString += mySocket.mBuffer[i];
		}
		Json::Value jsonObject;
		Json::Reader jsonReader;
		bool parsingSuccessful = jsonReader.parse(jsonString,jsonObject,false);
		k2_client::Audio audio;
		audio.header.stamp = ros::Time(jsonObject["utcTime"].asDouble());
		audio.header.frame_id =  ros::this_node::getNamespace().substr(1,std::string::npos) + "/microphone_frame";
		audio.beamAngle = jsonObject["beamAngle"].asDouble();
		audio.beamAngleConfidence = jsonObject["beamAngleConfidence"].asDouble();
		for(int i=0;i<256;i++)
		{
			audio.audioStream.push_back(jsonObject["audioStream"][i].asFloat());
		}
		audio.numBytesPerSample = jsonObject["numBytesPerSample"].asUInt();
		audio.numSamplesPerFrame = jsonObject["numSamplesPerFrame"].asUInt();
		audio.frameLifeTime = jsonObject["frameLifeTime"].asDouble();
		audio.samplingFrequency = jsonObject["samplingFrequency"].asUInt();
		audioPub.publish(audio);
	}
	return 0;
}
