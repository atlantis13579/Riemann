#include "RenderThread.h"

#include <chrono>
#include <utility>

#include "../RiemannRenderer/RiemannRenderer.h"

namespace Riemann
{
	RenderThread::RenderThread(Renderer* renderer)
		: m_Renderer(renderer)
		, m_RenderFps(0.0)
		, m_Running(false)
		, m_Started(false)
	{
	}

	RenderThread::~RenderThread()
	{
		Stop();
	}

	void RenderThread::Start()
	{
		if (m_Started || m_Renderer == nullptr)
		{
			return;
		}

		m_Running = true;
		m_Started = true;
		m_Thread = std::thread(&RenderThread::ThreadMain, this);
	}

	void RenderThread::Stop()
	{
		if (!m_Started)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			m_Running = false;
		}
		m_Condition.notify_all();

		if (m_Thread.joinable())
		{
			m_Thread.join();
		}
		m_Started = false;
	}

	void RenderThread::Submit(Command command)
	{
		if (!command)
		{
			return;
		}

		if (!m_Started)
		{
			if (m_Renderer)
			{
				command(*m_Renderer);
			}
			return;
		}

		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			m_Pending.push_back(std::move(command));
		}
		m_Condition.notify_one();
	}

	double RenderThread::GetRenderFps() const
	{
		return m_RenderFps.load();
	}

	void RenderThread::ThreadMain()
	{
		auto fpsTime = std::chrono::steady_clock::now();
		int frameCount = 0;

		while (true)
		{
			{
				std::lock_guard<std::mutex> lock(m_Mutex);
				if (!m_Running)
				{
					break;
				}
			}

			DrainCommands();
			if (m_Renderer)
			{
				m_Renderer->Render();
				++frameCount;

				const auto curr = std::chrono::steady_clock::now();
				const std::chrono::duration<double> elapsed = curr - fpsTime;
				if (elapsed.count() >= 1.0)
				{
					m_RenderFps.store(frameCount / elapsed.count());
					frameCount = 0;
					fpsTime = curr;
				}
			}

			std::unique_lock<std::mutex> lock(m_Mutex);
			m_Condition.wait_for(lock, std::chrono::milliseconds(16), [this]() {
				return !m_Running || !m_Pending.empty();
			});
		}

		DrainCommands();
	}

	void RenderThread::DrainCommands()
	{
		std::vector<Command> commands;
		{
			std::lock_guard<std::mutex> lock(m_Mutex);
			commands.swap(m_Pending);
		}

		for (Command& command : commands)
		{
			command(*m_Renderer);
		}
	}
}
