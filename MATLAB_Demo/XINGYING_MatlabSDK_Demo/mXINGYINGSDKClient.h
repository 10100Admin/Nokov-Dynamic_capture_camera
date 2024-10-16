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
*	��ʼ�����磬���ӵ��������
* @param szServerAddress
* 	�������
*	Server��IP��ַ��szMyIPAddress��Client IP (local IP)��szServerIP��Server IP (computer IP running motion capture software)
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_Initialize(char* szServerAddress);

/*!
* @brief
*	�붯������Ͽ�����
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_Uninitialize();

/*!
* @brief
*	��ȡ�汾��
* @param Version
* 	�������
*	�汾��Ϣ
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API void XINGYING_GetSDKVersion(unsigned char Version[4]);

/*!
* @brief
*	��ȡ�������Ӧ��������Ϣ
* @param pServerDescription
* 	�������
*	�������Ӧ��������Ϣ
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_GetServerDescription(sServerDescription* pServerDescription);

/*!
* @brief
*	��ȡ��������������Ϣ
* @param pDataDescriptions
* 	�������
*	��������������Ϣ
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_GetDataDescriptions(sDataDescriptions** pDataDescriptions);

/*!
* @brief
*	�ͷŹ�������������Ϣ
* @param pDataDescriptions
* 	�������
*	��������������Ϣ
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_FreeDataDescriptions(sDataDescriptions* pDataDescriptions);

/*!
* @brief
*	����TimeCode��Ϣ
* @param inTimecode
* 	�������
*	SMPTE timecode (if available)
* @param inTimecodeSubframe
* 	�������
*	timecode sub-frame data
* @param hour
* 	�������
*	hour
* @param minute
* 	�������
*	minute
* @param second
* 	�������
*	second
* @param frame
* 	�������
*	frame
* @param subframe
* 	�������
*	subframe
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API bool XINGYING_DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe);

/*!
* @brief
*	TimeCode��Ϣת��Ϊ�ַ���
* @param inTimecode
* 	�������
*	SMPTE timecode (if available)
* @param inTimecodeSubframe
* 	�������
*	timecode sub-frame data
* @param Buffer
* 	�������
*	�����TimeCode��Ϣ�ַ���
* @param Buffer
* 	�������
*	Buffer�Ĵ�С
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API bool XINGYING_TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char* Buffer, int BufferSize);

/*!
* @brief
*	��ȡ��ǰ֡��������
* @return sFrameOfMocapData
*	���ص�ǰ֡��������
*/
MXINGYINGSDKCLIENT_API sFrameOfMocapData* XINGYING_GetLastFrameOfMocapData();

/*!
* @brief
*	�ͷŵ�ǰ֡��������
* @return
*	����0��ʾ�ɹ�, �����ʾ������
*/
MXINGYINGSDKCLIENT_API int XINGYING_FreeFrame(sFrameOfMocapData* pDst);
