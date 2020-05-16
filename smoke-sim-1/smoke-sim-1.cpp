#include "smoke-sim-1.h"
#include "smokesolver.h"

//
//
// smoke-sim-1
// by Karl Paetow
//
// Created:     April 16, 2020
// Updated:     May 16, 2020
// Iteration:   3
//
// ABOUT
//
// This is an implementation of the basic 2D fluid simulation described in "Real-Time Fluid Dynamics for Games"
// by Jos Stam (2001.) It provides a graphic UI that permits the user to visualze the simulation's output in real
// time and to interact with it using a mouse.
//
// This is a strict implementionation of the content in the paper by Stam. It does not deviate from the methods
// described there. The intent is to provide a working 2D simulation which can then be used as a starting point
// for experimentation, such as altering the algorithms used to calculate the variables.
//
// Two variables are simulated:
// + velocity (vector field)
// + smoke density (scalar)
//
// This is a Windows native implementation that targets the Direct2D graphics API. As a result, it cannot be
// compiled and run on other platforms without significant modification.
//
// Direct2D UI implementation based on the code sample / tutorial provided by Microsoft here:
// https://docs.microsoft.com/en-us/windows/win32/direct2d/direct2d-quickstart
//
//
// Iteration 3
// + Removed remnant code from d2d-simple application that's no longer needed
//
// Iteration 2
// + Commented out rendering code for d2d-simple app
//
// Iteration 1
// + Ported basic Direct2D scaffolding from d2d-simple to create the basic working executable with a static pattern render
//







//
//
// Utility stuff
//
//


template<class Interface>
inline void SafeRelease(
	Interface** ppInterfaceToRelease
)
{
	if (*ppInterfaceToRelease != NULL)
	{
		(*ppInterfaceToRelease)->Release();

		(*ppInterfaceToRelease) = NULL;
	}
}




#ifndef Assert
#if defined( DEBUG ) || defined( _DEBUG )
#define Assert(b) do {if (!(b)) {OutputDebugStringA("Assert: " #b "\n");}} while(0)
#else
#define Assert(b)
#endif //DEBUG || _DEBUG
#endif



#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif





//
//
// Application class
// SmokeSimApp
//
//


class SmokeSimApp
{

public:

	SmokeSimApp() :
		m_hwnd(NULL),
		m_pDirect2dFactory(NULL),
		m_pRenderTarget(NULL),
		m_pLightSlateGrayBrush(NULL),
		m_pCornflowerBlueBrush(NULL),
		m_pDensityBrush(NULL)
	{
		//
		// Smoke solver stuff
		//

		// Create an instance of the solver class
		solver = new SmokeSolver;

		// Initialize variables
		dens_source = NULL;
		dens = NULL;
		u_source = NULL;
		v_source = NULL;
	}


	~SmokeSimApp()
	{
		SafeRelease(&m_pDirect2dFactory);
		SafeRelease(&m_pRenderTarget);
		SafeRelease(&m_pLightSlateGrayBrush);
		SafeRelease(&m_pCornflowerBlueBrush);

		dens_source = NULL;
		dens = NULL;
		u_source = NULL;
		v_source = NULL;
		delete solver;
	}


	// Register the window class and call methods for instantiating drawing resources
	HRESULT Initialize()
	{
		HRESULT hr;

		// Initialize device-indpendent resources, such
		// as the Direct2D factory
		hr = CreateDeviceIndependentResources();

		if (SUCCEEDED(hr))
		{

			// Get the grid size
			// This is necessary for the IX macro to work right
			gridx = solver->get_gridx();
			gridy = solver->get_gridy();

			// Get a pointer to the density field
			// which will be used for rendering
			dens = solver->get_dens();

			// Get pointers to the dens_source, u_source and v_source
			// fields which will be used to inject density
			// and velocity into the simulation
			dens_source = solver->get_dens_source();
			u_source = solver->get_u_source();
			v_source = solver->get_v_source();



			// Register the window class.
			WNDCLASSEX wcex = { sizeof(WNDCLASSEX) };
			wcex.style = CS_HREDRAW | CS_VREDRAW;
			wcex.lpfnWndProc = WndProc;
			wcex.cbClsExtra = 0;
			wcex.cbWndExtra = sizeof(LONG_PTR);
			wcex.hInstance = HINST_THISCOMPONENT;
			wcex.hbrBackground = NULL;
			wcex.lpszMenuName = NULL;
			wcex.hCursor = LoadCursor(NULL, IDI_APPLICATION);
			wcex.lpszClassName = L"SmokeSimApp";

			RegisterClassEx(&wcex);


			// Create the window
			m_hwnd = CreateWindow(
				L"SmokeSimApp",
				L"Smoke Sim 1 - A basic 2D smoke simulation",
				WS_OVERLAPPEDWINDOW,
				CW_USEDEFAULT,
				CW_USEDEFAULT,
				800,
				800,
				NULL,
				NULL,
				HINST_THISCOMPONENT,
				this
			);

			hr = m_hwnd ? S_OK : E_FAIL;
			if (SUCCEEDED(hr))
			{
				ShowWindow(m_hwnd, SW_SHOWNORMAL);
				UpdateWindow(m_hwnd);
			}

		}

		return hr;
	}


	// Process and dispatch messages
	void RunMessageLoop()
	{
		MSG msg;

		while (true)
		{
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}

			// Check for WM_QUIT message
			if (msg.message == WM_QUIT)
				break;

			// Execute one simulation step
			solver->iterate();

			// Render the density field
			OnRender();
		}
	}



private:

	HWND m_hwnd;
	ID2D1Factory* m_pDirect2dFactory;
	ID2D1HwndRenderTarget* m_pRenderTarget;
	ID2D1SolidColorBrush* m_pLightSlateGrayBrush;
	ID2D1SolidColorBrush* m_pCornflowerBlueBrush;
	ID2D1SolidColorBrush* m_pDensityBrush;

	// Simulation-related attributes
	SmokeSolver* solver;
	float* dens_source;
	float* u_source;
	float* v_source;
	float* dens;
	int gridsize = 0;
	int gridx = 0;
	int gridy = 0;
	int injiterations = 500;
	bool injstopflag = false;
	float pwx = 0.0f;
	float pwy = 0.0f;


	// Initialize device-independent resources
	HRESULT CreateDeviceIndependentResources()
	{
		HRESULT hr = S_OK;

		// Create a Direct2D factory.
		hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pDirect2dFactory);

		return hr;
	}


	// Initialize device-dependent resources
	HRESULT CreateDeviceResources()
	{
		HRESULT hr = S_OK;

		if (!m_pRenderTarget)
		{
			RECT rc;
			GetClientRect(m_hwnd, &rc);

			D2D1_SIZE_U size = D2D1::SizeU(
				rc.right - rc.left,
				rc.bottom - rc.top
			);

			// Create a Direct2D render target
			hr = m_pDirect2dFactory->CreateHwndRenderTarget(
				D2D1::RenderTargetProperties(),
				D2D1::HwndRenderTargetProperties(m_hwnd, size),
				&m_pRenderTarget
			);

		}

		return hr;
	}


	// Release device-dependent resource
	void DiscardDeviceResources()
	{
		SafeRelease(&m_pRenderTarget);
		//SafeRelease(&m_pLightSlateGrayBrush);
		//SafeRelease(&m_pCornflowerBlueBrush);
	}


	// Draw content
	HRESULT OnRender()
	{
		HRESULT hr = S_OK;

		int i, j;

		float density = 0.0f;
		float pxl = 0.0f;
		float pxr = 0.0f;
		float pyt = 0.0f;
		float pyb = 0.0f;

		hr = CreateDeviceResources();

		if (SUCCEEDED(hr))
		{
			// IMPORTANT!
			// Always preface drawing operations with this call
			m_pRenderTarget->BeginDraw();


			//
			// Drawing operations occur here
			//



			//
			//
			// Drawing operations for smoke-sim-1
			//
			//

			// Set transform
			m_pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());

			// Get size of the render window
			D2D1_SIZE_F rtSize = m_pRenderTarget->GetSize();
			int width = static_cast<int>(rtSize.width);
			int height = static_cast<int>(rtSize.height);

			//
			// The simulation grid will be scaled to fit the render window
			//

			// Calculate the size of each "pixel" (as the simulation grid may not match the render window size)

			//
			// Render the density
			//
			for (j = 1; j < gridy; j++)
			{
				for (i = 1; i < gridx; i++)
				{

					// Get the value of the density from the curent cell of the simulation density field
					density = (dens[IX(i, j)] * 100);

					// Create the Direct2D brush that will be used to render the current cell of the simulation density field
					hr = m_pRenderTarget->CreateSolidColorBrush(
						D2D1::ColorF(D2D1::ColorF(density, density, density, 1.0f)),
						&m_pDensityBrush
					);

					if (SUCCEEDED(hr))
					{
						// Calculate the coordinates of the current pixel rectangle
						pxl = (float)(i-1) * pwx;
						pxr = ((float)(i-1) * pwx) + pwx;
						pyt = (float)(j-1) * pwy;
						pyb = ((float)(j-1) * pwy) + pwy;

						// Define the position and size of the rectangle that represnets the current pixel
						D2D1_RECT_F pixel = D2D1::RectF(pxl, pyt, pxr, pyb);

						// Draw that pixel
						m_pRenderTarget->FillRectangle(&pixel, m_pDensityBrush);

						// Release the brush in preparation for creating the next one
						SafeRelease(&m_pDensityBrush);
					}
				}
			}



			// IMPORTANT!
			// Always terminate drawing operations with this call
			hr = m_pRenderTarget->EndDraw();

			if (hr == D2DERR_RECREATE_TARGET)
			{
				hr = S_OK;
				DiscardDeviceResources();
			}


			// Stop injection of test sources after X iterations
			if (injiterations > 0)
				injiterations--;

			if (!injstopflag && injiterations <= 0)
			{
				injiterations = 0;
				injstopflag = true;

				// Clear out the source fields - we don't want eternal density or acceleration!
				solver->clear_dens_source();
				solver->clear_vel_source();
			}

		}

		return hr;
	}


	//
	// Resize the render target
	//
	void OnResize(
		UINT width,
		UINT height
	)
	{

		// Recalculate the size of the Direct2D render surface
		if (m_pRenderTarget)
		{
			// Note: This method can fail, but it's okay to ignore the
			// error here, because the error will be returned again
			// the next time EndDraw is called
			m_pRenderTarget->Resize(D2D1::SizeU(width, height));
		}

		// Recalculate the "pixel" width and height
		// (to enable scaling of the density field to fit in the render window)
		gridx = solver->get_gridx();
		gridy = solver->get_gridy();
		pwx = (float)width / (float)gridx;
		pwy = (float)height / (float)gridy;
	}


	//
	// Mouse click handler
	//
	void OnMouseClick(int x, int y)
	{
		int i, j;

		// Calculate the grid cell i, j index coordinates based on the x, y coordinates
		i = (int)((float)x / pwx);
		j = (int)((float)y / pwy);

		// Add density
		dens_source[IX(i, j)] = 3.0f;

		// Add velocity
		v_source[IX(i, j)] = -200.0f;

		// Set injector iteration count
		injiterations = 2;
		injstopflag = false;
	}


	//
	// The window procedure
	//

	static LRESULT CALLBACK WndProc(
		HWND hwnd,
		UINT message,
		WPARAM wParam,
		LPARAM lParam
	)
	{
		LRESULT result = 0;

		if (message == WM_CREATE)
		{
			LPCREATESTRUCT pcs = (LPCREATESTRUCT)lParam;
			SmokeSimApp* pApp = (SmokeSimApp*)pcs->lpCreateParams;

			::SetWindowLongPtrW(
				hwnd,
				GWLP_USERDATA,
				reinterpret_cast<LONG_PTR>(pApp)
			);

			result = 1;
		}
		else
		{
			SmokeSimApp* pApp = reinterpret_cast<SmokeSimApp*>(static_cast<LONG_PTR>(
				::GetWindowLongPtrW(
					hwnd,
					GWLP_USERDATA
				)));

			bool wasHandled = false;

			if (pApp)
			{
				switch (message)
				{
				case WM_SIZE:
				{
					UINT width = LOWORD(lParam);
					UINT height = HIWORD(lParam);
					pApp->OnResize(width, height);
				}
				result = 0;
				wasHandled = true;
				break;

				case WM_DISPLAYCHANGE:
				{
					InvalidateRect(hwnd, NULL, FALSE);
				}
				result = 0;
				wasHandled = true;
				break;

				case WM_PAINT:
				{
					pApp->OnRender();
					ValidateRect(hwnd, NULL);
				}
				result = 0;
				wasHandled = true;
				break;

				case WM_DESTROY:
				{
					PostQuitMessage(0);
				}
				result = 1;
				wasHandled = true;
				break;

				case WM_KEYDOWN:
				{
					switch (wParam)
					{
					case VK_ESCAPE:
						PostQuitMessage(0);
					}
				}
				result = 1;
				wasHandled = true;
				break;

				case WM_LBUTTONDOWN:
				{
					pApp->OnMouseClick(
						GET_X_LPARAM(lParam), 
						GET_Y_LPARAM(lParam)
					);
				}
				result = 1;
				wasHandled = true;
				break;

				case WM_CLOSE:
				{
					PostQuitMessage(0);
				}
				result = 1;
				wasHandled = true;

				default:
					break;
				}
			}

			if (!wasHandled)
			{
				result = DefWindowProc(hwnd, message, wParam, lParam);
			}
		}

		return result;
	}


};



int WINAPI WinMain(
	HINSTANCE /* hInstance */,
	HINSTANCE /* hPrevInstance */,
	LPSTR /* lpCmdLine */,
	int /* nCmdShow */
)
{
	// Use HeapSetInformation to specify that the process should
	// terminate if the heap manager detects an error in any heap used
	// by the process
	// The return value is ignored, because we want to continue running in the
	// unlikely event that HeapSetInformation fails
	HeapSetInformation(NULL, HeapEnableTerminationOnCorruption, NULL, 0);

	if (SUCCEEDED(CoInitialize(NULL)))
	{
		{
			SmokeSimApp app;

			if (SUCCEEDED(app.Initialize()))
			{
				app.RunMessageLoop();
			}
		}
		CoUninitialize();
	}

	return 0;
}
