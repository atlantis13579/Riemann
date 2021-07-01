#pragma once

class Renderer
{
public:
	Renderer() {}
	virtual ~Renderer() {}

	virtual void Render() = 0;

	static Renderer* CreateDX11Renderer(void* hWnd);
};