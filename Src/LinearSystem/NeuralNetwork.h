#pragma once

#include <cstddef>
#include <memory>
#include <vector>

namespace Maths
{
	namespace LinearAlgebra
	{
		enum class NeuralLayerType { Linear, ELU, LSTM, ReLU, Sigmoid, Tanh, Softmax, Identity };

		struct NeuralLayer
		{
			NeuralLayerType Type;
			size_t Width = 0;
		};

		class NeuralModule
		{
		public:
			virtual ~NeuralModule() = default;
			size_t GetInputSize() const { return m_Inputs; }
			size_t GetOutputSize() const { return m_Output.size(); }
			const std::vector<float>& Evaluate(const std::vector<float>& Input);
			virtual void Reset() {}
		protected:
			NeuralModule(size_t Inputs, size_t Outputs);
			virtual void Forward(const std::vector<float>& Input) = 0;
			std::vector<float> m_Output;
		private:
			size_t m_Inputs;
		};

		// Row-major affine map. An empty bias selects y = Wx.
		class Linear final : public NeuralModule
		{
		public:
			Linear(size_t Inputs, size_t Outputs, std::vector<float> Weights, std::vector<float> Bias = {});
		private:
			void Forward(const std::vector<float>& Input) override;
			std::vector<float> m_Weights, m_Bias;
		};

		class Activation final : public NeuralModule
		{
		public:
			Activation(size_t Width, NeuralLayerType Type);
		private:
			void Forward(const std::vector<float>& Input) override;
			NeuralLayerType m_Type;
		};

		// One recurrent timestep. Matrices and optional combined bias use
		// input, forget, candidate and output gate order.
		class LSTM final : public NeuralModule
		{
		public:
			LSTM(size_t Inputs, size_t HiddenSize, std::vector<float> InputWeights,
				std::vector<float> RecurrentWeights, std::vector<float> Bias = {});
			void Reset() override;
		private:
			void Forward(const std::vector<float>& Input) override;
			std::vector<float> m_InputWeights, m_RecurrentWeights, m_Bias;
			std::vector<float> m_Cell, m_Gates;
		};

		class NeuralNetwork
		{
		public:
			explicit NeuralNetwork(size_t Inputs);
			// Packed Linear: W, b. Packed LSTM: W_input, W_hidden, b.
			// Activations preserve width and consume no weights.
			NeuralNetwork(size_t Inputs, const std::vector<NeuralLayer>& Layers, std::vector<float> Weights);
			void Append(std::unique_ptr<NeuralModule> Module);
			size_t GetInputSize() const { return m_Inputs; }
			size_t GetOutputSize() const;
			const std::vector<float>& Evaluate(const std::vector<float>& Input);
			void Reset();
		private:
			size_t m_Inputs;
			std::vector<std::unique_ptr<NeuralModule>> m_Layers;
		};
	}
}
