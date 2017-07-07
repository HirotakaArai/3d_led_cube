// sampleUsingDll.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "stdafx.h"
#include "sampleUsingDll.h"
#include "ledLib.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

namespace
{
	struct Led
	{
		SetUrl_t * SetUrl;
		SetLed_t * SetLed;
		Clear_t * ClearLed;
		Show_t * Show;
		Wait_t * Wait;
	};

	Led LoadMethods(LPCTSTR dll)
	{
		Led m;
		HMODULE mod = LoadLibrary(dll);
		if (!mod){
			throw std::runtime_error("failed to load dll.");
		}
		m.SetUrl = reinterpret_cast<SetUrl_t*>(GetProcAddress(mod, "SetUrl"));
		m.SetLed = reinterpret_cast<SetLed_t*>(GetProcAddress(mod, "SetLed"));
		m.ClearLed = reinterpret_cast<Clear_t*>(GetProcAddress(mod, "Clear"));
		m.Show = reinterpret_cast<Show_t*>(GetProcAddress(mod, "Show"));
		m.Wait = reinterpret_cast<Wait_t*>(GetProcAddress(mod, "Wait"));
		return m;
	}

	void proc(int argc, TCHAR* argv[], TCHAR* envp[])
	{
		Led led = LoadMethods(_T("ledLib.dll"));
		for (int x = 0; x < LED_WIDTH; ++x){
			for (int y = 0; y < LED_HEIGHT; ++y){
				for (int z = 0; z < LED_DEPTH; ++z){
					int c = (x % 2) ? 0xFF0000 : 0x00FF00;
					led.SetLed(x, y, z, c);
					led.Show();
					led.Wait(10);
				}
			}
		}
	}
}

int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	HMODULE hModule = ::GetModuleHandle(NULL);

	if (hModule != NULL)
	{
		// MFC �����������āA�G���[�̏ꍇ�͌��ʂ�������܂��B
		if (!AfxWinInit(hModule, NULL, ::GetCommandLine(), 0))
		{
			// TODO: �K�v�ɉ����ăG���[ �R�[�h��ύX���Ă��������B
			_tprintf(_T("�v���I�ȃG���[: MFC �̏��������ł��܂���ł����B\n"));
			nRetCode = 1;
		}
		else
		{
			proc(argc, argv, envp);
		}
	}
	else
	{
		// TODO: �K�v�ɉ����ăG���[ �R�[�h��ύX���Ă��������B
		_tprintf(_T("�v���I�ȃG���[: GetModuleHandle �����s���܂���\n"));
		nRetCode = 1;
	}

	return nRetCode;
}
