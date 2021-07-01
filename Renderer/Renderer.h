#pragma once

#ifdef RENDERER_EXPORT
#  define RENDERER_API __declspec(dllexport)
#else
#  define RENDERER_API __declspec(dllimport)
#endif

#include <string>

class RENDERER_API Renderer
{
public:
	Renderer() {}
	virtual ~Renderer() {}

	virtual void Render() = 0;

	static Renderer* CreateDX11Renderer(void* hWnd, const std::string& shader_path);
};