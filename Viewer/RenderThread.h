#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace Riemann
{
	class Renderer;

	class RenderThread
	{
	public:
		typedef std::function<void(Renderer&)> Command;

		explicit RenderThread(Renderer* renderer);
		~RenderThread();

		void Start();
		void Stop();
		void SetActive(bool active) { m_Active.store(active); }
		bool IsActive() const { return m_Active.load(); }
		void Submit(Command command);
		double GetRenderFps() const;
		bool SupportsDensityVolume() const;

	private:
		void ThreadMain();
		void DrainCommands();

	private:
		Renderer* m_Renderer;
		std::thread m_Thread;
		std::mutex m_Mutex;
		std::condition_variable m_Condition;
		std::vector<Command> m_Pending;
		std::atomic<double> m_RenderFps;
		std::atomic<bool> m_Active{true};
		bool m_Running;
		bool m_Started;
	};
}
