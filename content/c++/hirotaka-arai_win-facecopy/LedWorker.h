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
	void Terminate(); // ←スレッド外からスレッド終了指示をする為のメソッド 

private:

	int myX;
	int myY;
	int myZ;
	int myRgb;
	const char * myText;
	bool myShowFlg;
	int myWait;
	bool myClearFlg;

	bool myExitFlag; // ←終了指示を保持するフラグ 
	HANDLE myMutex;  // ←排他用 Mutex 

	DWORD WINAPI ExecThread();
};

