#pragma once

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
		void Submit(Command command);

	private:
		void ThreadMain();
		void DrainCommands();

	private:
		Renderer* m_Renderer;
		std::thread m_Thread;
		std::mutex m_Mutex;
		std::condition_variable m_Condition;
		std::vector<Command> m_Pending;
		bool m_Running;
		bool m_Started;
	};
}
