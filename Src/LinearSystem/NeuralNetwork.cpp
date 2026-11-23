#include "NeuralNetwork.h"
#include "gemm.inl"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Maths
{
	namespace LinearAlgebra
	{
		namespace
		{
			size_t Product(size_t A, size_t B)
			{
				if (!A || !B || A > std::numeric_limits<size_t>::max() / B)
					throw std::invalid_argument("Invalid neural layer dimensions");
				return A * B;
			}

			float Sigmoid(float X)
			{
				return 1.0f / (1.0f + std::exp(-X));
			}

			float AffineRow(const float* Weights, const std::vector<float>& Input, float Sum)
			{
				for (size_t I = 0; I < Input.size(); ++I)
					Sum += Weights[I] * Input[I];
				return Sum;
			}

			std::vector<float> Take(const std::vector<float>& Packed, size_t& Offset, size_t Count)
			{
				if (Count > Packed.size() - Offset)
					throw std::invalid_argument("Incomplete neural layer parameters");
				const size_t Begin = Offset;
				Offset += Count;
				return std::vector<float>(Packed.begin() + Begin, Packed.begin() + Offset);
			}
		}

		NeuralModule::NeuralModule(size_t Inputs, size_t Outputs) : m_Output(Outputs), m_Inputs(Inputs)
		{
			if (!Inputs || !Outputs)
				throw std::invalid_argument("Empty neural layer");
		}

		const std::vector<float>& NeuralModule::Evaluate(const std::vector<float>& Input)
		{
			if (Input.size() != GetInputSize())
				throw std::invalid_argument("Incorrect neural layer input size");
			if (&Input == &m_Output)
			{
				const std::vector<float> Copy(Input);
				Forward(Copy);
			}
			else
				Forward(Input);
			return m_Output;
		}

		Linear::Linear(size_t Inputs, size_t Outputs, std::vector<float> Weights, std::vector<float> Bias)
			: NeuralModule(Inputs, Outputs), m_Weights(std::move(Weights)), m_Bias(std::move(Bias))
		{
			const size_t MaxDimension = std::numeric_limits<int>::max();
			if (Inputs > MaxDimension || Outputs > MaxDimension)
				throw std::invalid_argument("Linear dimensions exceed the GEMM limit");
			if (m_Weights.size() != Product(Inputs, Outputs) || (!m_Bias.empty() && m_Bias.size() != Outputs))
				throw std::invalid_argument("Invalid linear parameters");
		}

		void Linear::Forward(const std::vector<float>& Input)
		{
			gemm(m_Weights.data(), Input.data(), static_cast<int>(GetOutputSize()), static_cast<int>(GetInputSize()), 1, m_Output.data());
			for (size_t I = 0; I < m_Bias.size(); ++I)
				m_Output[I] += m_Bias[I];
		}

		Activation::Activation(size_t Width, NeuralLayerType Type) : NeuralModule(Width, Width), m_Type(Type)
		{
			switch (Type)
			{
			case NeuralLayerType::Identity:
			case NeuralLayerType::ReLU:
			case NeuralLayerType::ELU:
			case NeuralLayerType::Sigmoid:
			case NeuralLayerType::Tanh:
			case NeuralLayerType::Softmax:
				break;
			default:
				throw std::invalid_argument("Unknown activation");
			}
		}

		void Activation::Forward(const std::vector<float>& Input)
		{
			if (m_Type == NeuralLayerType::Softmax)
			{
				const float Max = *std::max_element(Input.begin(), Input.end());
				double Sum = 0;
				for (size_t I = 0; I < Input.size(); ++I)
				{
					m_Output[I] = std::exp(Input[I] - Max);
					Sum += m_Output[I];
				}
				for (float& Probability : m_Output)
					Probability = static_cast<float>(Probability / Sum);
				return;
			}
			for (size_t I = 0; I < Input.size(); ++I)
			{
				const float X = Input[I];
				switch (m_Type)
				{
				case NeuralLayerType::ReLU: m_Output[I] = std::max(0.0f, X); break;
				case NeuralLayerType::ELU: m_Output[I] = X < 0 ? std::expm1(X) : X; break;
				case NeuralLayerType::Sigmoid: m_Output[I] = Sigmoid(X); break;
				case NeuralLayerType::Tanh: m_Output[I] = std::tanh(X); break;
				default: m_Output[I] = X; break;
				}
			}
		}

		LSTM::LSTM(size_t Inputs, size_t HiddenSize, std::vector<float> InputWeights,
			std::vector<float> RecurrentWeights, std::vector<float> Bias)
			: NeuralModule(Inputs, HiddenSize), m_InputWeights(std::move(InputWeights)),
			m_RecurrentWeights(std::move(RecurrentWeights)), m_Bias(std::move(Bias)),
			m_Cell(HiddenSize), m_Gates(Product(4, HiddenSize))
		{
			if (m_InputWeights.size() != Product(m_Gates.size(), Inputs)
				|| m_RecurrentWeights.size() != Product(m_Gates.size(), HiddenSize)
				|| (!m_Bias.empty() && m_Bias.size() != m_Gates.size()))
				throw std::invalid_argument("Invalid LSTM parameters");
		}

		void LSTM::Reset()
		{
			std::fill(m_Cell.begin(), m_Cell.end(), 0.0f);
			std::fill(m_Output.begin(), m_Output.end(), 0.0f);
		}

		void LSTM::Forward(const std::vector<float>& Input)
		{
			const size_t Width = GetOutputSize();
			for (size_t R = 0; R < m_Gates.size(); ++R)
			{
				const float X = AffineRow(m_InputWeights.data() + R * Input.size(), Input, m_Bias.empty() ? 0.0f : m_Bias[R]);
				m_Gates[R] = AffineRow(m_RecurrentWeights.data() + R * Width, m_Output, X);
			}
			for (size_t J = 0; J < Width; ++J)
			{
				m_Cell[J] = Sigmoid(m_Gates[Width + J]) * m_Cell[J] + Sigmoid(m_Gates[J]) * std::tanh(m_Gates[2 * Width + J]);
				m_Output[J] = Sigmoid(m_Gates[3 * Width + J]) * std::tanh(m_Cell[J]);
			}
		}

		NeuralNetwork::NeuralNetwork(size_t Inputs) : m_Inputs(Inputs)
		{
			if (!Inputs)
				throw std::invalid_argument("Empty network input");
		}

		NeuralNetwork::NeuralNetwork(size_t Inputs, const std::vector<NeuralLayer>& Layers, std::vector<float> Weights)
			: NeuralNetwork(Inputs)
		{
			if (Layers.empty())
				throw std::invalid_argument("Empty neural network");
			size_t Offset = 0;
			for (const NeuralLayer& Spec : Layers)
			{
				const size_t In = GetOutputSize();
				// Older packed models encode Linear followed by ELU as one entry with a width.
				if (Spec.Type == NeuralLayerType::Linear || (Spec.Type == NeuralLayerType::ELU && Spec.Width))
				{
					std::vector<float> Matrix = Take(Weights, Offset, Product(In, Spec.Width));
					std::vector<float> Bias = Take(Weights, Offset, Spec.Width);
					Append(std::make_unique<Linear>(In, Spec.Width, std::move(Matrix), std::move(Bias)));
					if (Spec.Type == NeuralLayerType::ELU)
						Append(std::make_unique<Activation>(Spec.Width, Spec.Type));
				}
				else if (Spec.Type == NeuralLayerType::LSTM)
				{
					const size_t Rows = Product(4, Spec.Width);
					std::vector<float> Matrix = Take(Weights, Offset, Product(Rows, In));
					std::vector<float> Recurrent = Take(Weights, Offset, Product(Rows, Spec.Width));
					std::vector<float> Bias = Take(Weights, Offset, Rows);
					Append(std::make_unique<LSTM>(In, Spec.Width, std::move(Matrix), std::move(Recurrent), std::move(Bias)));
				}
				else
				{
					if (Spec.Width)
						throw std::invalid_argument("An activation inherits its input width");
					Append(std::make_unique<Activation>(In, Spec.Type));
				}
			}
			if (Offset != Weights.size())
				throw std::invalid_argument("Unexpected trailing layer parameters");
		}

		void NeuralNetwork::Append(std::unique_ptr<NeuralModule> Module)
		{
			if (!Module || Module->GetInputSize() != GetOutputSize())
				throw std::invalid_argument("Incompatible network layers");
			m_Layers.push_back(std::move(Module));
		}

		size_t NeuralNetwork::GetOutputSize() const
		{
			return m_Layers.empty() ? m_Inputs : m_Layers.back()->GetOutputSize();
		}

		const std::vector<float>& NeuralNetwork::Evaluate(const std::vector<float>& Input)
		{
			if (m_Layers.empty())
				throw std::logic_error("Cannot evaluate an empty network");
			const std::vector<float>* Values = &Input;
			for (const std::unique_ptr<NeuralModule>& Module : m_Layers)
				Values = &Module->Evaluate(*Values);
			return *Values;
		}

		void NeuralNetwork::Reset()
		{
			for (const std::unique_ptr<NeuralModule>& Module : m_Layers)
				Module->Reset();
		}
	}
}
