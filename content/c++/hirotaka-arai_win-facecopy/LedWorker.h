#pragma once

#include <windows.h>
#include <WindowsX.h>
#include <wchar.h>
#include <string>
#include <assert.h>
#include <map>
//#include "myled.h"

class LedWorker
{
public:
	LedWorker();
	~LedWorker();

	bool IsShowMotioningText1();

	void MyShowMotioningText1(const char * text);
	void MySetLed(int x, int y, int z, int rgb);
	void MyShow(bool bShow);
	void MyWait(int wait);
	void MyClear();
	static DWORD WINAPI ThreadFunc(LPVOID lpParameter);
	void Terminate(); // ���X���b�h�O����X���b�h�I���w��������ׂ̃��\�b�h 

private:

	int myX;
	int myY;
	int myZ;
	int myRgb;
	const char * myText;
	bool myShowFlg;
	int myWait;
	bool myClearFlg;

	bool myExitFlag; // ���I���w����ێ�����t���O 
	HANDLE myMutex;  // ���r���p Mutex 

	DWORD WINAPI ExecThread();
};

