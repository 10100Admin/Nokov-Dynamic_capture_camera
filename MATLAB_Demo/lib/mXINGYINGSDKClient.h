#pragma once  

#ifndef MXINGYINGSDKCLIENT_H
#define MXINGYINGSDKCLIENT_H

#ifndef MXINGYINGSDKCLIENT_API
#define MXINGYINGSDKCLIENT_API __declspec(dllexport)  
#endif

#ifndef CALLSTACK
#define CALLSTACK		__stdcall
#endif

#endif

#include <iostream>
#include <vector>
using namespace std;

#include "NokovSDKTypes.h"

/*!
* @brief
*	初始化网络，连接到动捕软件
* @param szServerAddress
* 	输入参数
*	Server端IP地址。szMyIPAddress：Client IP (local IP)；szServerIP：Server IP (computer IP running motion capture software)
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_Initialize(char* szServerAddress);

/*!
* @brief
*	与动捕软件断开连接
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_Uninitialize();

/*!
* @brief
*	获取版本号
* @param Version
* 	输出参数
*	版本信息
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API void XINGYING_GetSDKVersion(unsigned char Version[4]);

/*!
* @brief
*	获取动捕软件应用描述信息
* @param pServerDescription
* 	输出参数
*	动捕软件应用描述信息
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_GetServerDescription(sServerDescription* pServerDescription);

/*!
* @brief
*	获取骨骼数据描述信息
* @param pDataDescriptions
* 	输出参数
*	骨骼数据描述信息
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_GetDataDescriptions(sDataDescriptions** pDataDescriptions);

/*!
* @brief
*	释放骨骼数据描述信息
* @param pDataDescriptions
* 	输入参数
*	骨骼数据描述信息
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_FreeDataDescriptions(sDataDescriptions* pDataDescriptions);

/*!
* @brief
*	解析TimeCode信息
* @param inTimecode
* 	输入参数
*	SMPTE timecode (if available)
* @param inTimecodeSubframe
* 	输入参数
*	timecode sub-frame data
* @param hour
* 	输出参数
*	hour
* @param minute
* 	输出参数
*	minute
* @param second
* 	输出参数
*	second
* @param frame
* 	输出参数
*	frame
* @param subframe
* 	输出参数
*	subframe
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API bool XINGYING_DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);

/*!
* @brief
*	TimeCode信息转换为字符串
* @param inTimecode
* 	输入参数
*	SMPTE timecode (if available)
* @param inTimecodeSubframe
* 	输入参数
*	timecode sub-frame data
* @param Buffer
* 	输出参数
*	输出的TimeCode信息字符串
* @param Buffer
* 	输入参数
*	Buffer的大小
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API bool XINGYING_TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char* Buffer, int BufferSize);

/*!
* @brief
*	获取当前帧动捕数据
* @return sFrameOfMocapData
*	返回当前帧动捕数据
*/
MXINGYINGSDKCLIENT_API sFrameOfMocapData* XINGYING_GetLastFrameOfMocapData();

/*!
* @brief
*	释放当前帧动捕数据
* @return
*	返回0表示成功, 否则表示错误码
*/
MXINGYINGSDKCLIENT_API int XINGYING_FreeFrame(sFrameOfMocapData* pDst);
