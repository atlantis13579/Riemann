#pragma once

#include "RiemannRenderer.h"
#include <deque>

namespace Riemann
{
	// Preserve clicks that begin and end between two render frames.
	// The renderer's input mutex protects this queue.
	class ImguiInputQueue
	{
	public:
		void Push(const ImguiInputState& Input)
		{
			if (Input.MouseButtons != m_Current.MouseButtons)
				m_ButtonEvents.push_back(Input);
			m_Current = Input;
			m_Wheel += Input.MouseWheel;
		}

		ImguiInputState Consume()
		{
			ImguiInputState Input = m_Current;
			if (!m_ButtonEvents.empty())
			{
				Input = m_ButtonEvents.front();
				m_ButtonEvents.pop_front();
			}
			Input.MouseWheel = m_Wheel;
			m_Wheel = 0;
			return Input;
		}

	private:
		ImguiInputState m_Current;
		std::deque<ImguiInputState> m_ButtonEvents;
		int m_Wheel = 0;
	};
}
