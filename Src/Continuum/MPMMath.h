#pragma once

#include <cmath>

#if defined(__CUDACC__)
#define RIEMANN_MPM_HD __host__ __device__
#else
#define RIEMANN_MPM_HD
#endif

namespace Riemann
{
	namespace MPM
	{
		RIEMANN_MPM_HD inline float Clamp(float X, float A, float B) { return fminf(fmaxf(X, A), B); }
		struct Vector
		{
			float x = 0, y = 0, z = 0;
			RIEMANN_MPM_HD Vector() {}
			RIEMANN_MPM_HD Vector(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
			RIEMANN_MPM_HD float& operator[](int I) { return I == 0 ? x : I == 1 ? y : z; }
			RIEMANN_MPM_HD float operator[](int I) const { return I == 0 ? x : I == 1 ? y : z; }
			RIEMANN_MPM_HD Vector operator+(Vector B) const { return {x + B.x, y + B.y, z + B.z}; }
			RIEMANN_MPM_HD Vector operator-(Vector B) const { return {x - B.x, y - B.y, z - B.z}; }
			RIEMANN_MPM_HD Vector operator*(float S) const { return {x * S, y * S, z * S}; }
			RIEMANN_MPM_HD Vector operator/(float S) const { return *this * (1 / S); }
			RIEMANN_MPM_HD Vector& operator+=(Vector B) { x += B.x; y += B.y; z += B.z; return *this; }
			RIEMANN_MPM_HD float Dot(Vector B) const { return x * B.x + y * B.y + z * B.z; }
			RIEMANN_MPM_HD Vector Cross(Vector B) const { return {y * B.z - z * B.y, z * B.x - x * B.z, x * B.y - y * B.x}; }
			RIEMANN_MPM_HD float Length() const { return sqrtf(Dot(*this)); }
		};

		struct Matrix
		{
			float Data[9]{};
			RIEMANN_MPM_HD Matrix() {}
			RIEMANN_MPM_HD float& operator()(int R, int C) { return Data[R * 3 + C]; }
			RIEMANN_MPM_HD float operator()(int R, int C) const { return Data[R * 3 + C]; }
			RIEMANN_MPM_HD static Matrix Identity() { Matrix M; M(0,0) = M(1,1) = M(2,2) = 1; return M; }
			RIEMANN_MPM_HD Vector Column(int C) const { return {Data[C], Data[3 + C], Data[6 + C]}; }
			RIEMANN_MPM_HD void SetColumn(int C, Vector V) { Data[C] = V.x; Data[3 + C] = V.y; Data[6 + C] = V.z; }
			RIEMANN_MPM_HD Matrix operator+(const Matrix& B) const
			{ Matrix M; for (int I = 0; I < 9; ++I) M.Data[I] = Data[I] + B.Data[I]; return M; }
			RIEMANN_MPM_HD Matrix operator-(const Matrix& B) const
			{ Matrix M; for (int I = 0; I < 9; ++I) M.Data[I] = Data[I] - B.Data[I]; return M; }
			RIEMANN_MPM_HD Matrix operator*(float S) const
			{ Matrix M; for (int I = 0; I < 9; ++I) M.Data[I] = Data[I] * S; return M; }
			RIEMANN_MPM_HD Matrix operator*(const Matrix& B) const
			{
				Matrix M;
				for (int R = 0; R < 3; ++R) for (int C = 0; C < 3; ++C)
					for (int K = 0; K < 3; ++K) M(R,C) += (*this)(R,K) * B(K,C);
				return M;
			}
			RIEMANN_MPM_HD Vector operator*(Vector V) const
			{ return {Data[0]*V.x + Data[1]*V.y + Data[2]*V.z, Data[3]*V.x + Data[4]*V.y + Data[5]*V.z, Data[6]*V.x + Data[7]*V.y + Data[8]*V.z}; }
			RIEMANN_MPM_HD Matrix Transpose() const
			{ Matrix M; for (int R = 0; R < 3; ++R) for (int C = 0; C < 3; ++C) M(R,C) = (*this)(C,R); return M; }
			RIEMANN_MPM_HD float Determinant() const { return Column(0).Dot(Column(1).Cross(Column(2))); }
		};
		RIEMANN_MPM_HD inline Matrix Outer(Vector A, Vector B)
		{ Matrix M; for (int R = 0; R < 3; ++R) for (int C = 0; C < 3; ++C) M(R,C) = A[R] * B[C]; return M; }
		RIEMANN_MPM_HD inline Matrix Diagonal(Vector V)
		{ Matrix M; M(0,0) = V.x; M(1,1) = V.y; M(2,2) = V.z; return M; }

		struct SVD { Matrix U, V; Vector Stretch; };
		RIEMANN_MPM_HD inline SVD Decompose(Matrix F)
		{
			Matrix V = Matrix::Identity();
			// One-sided Jacobi avoids squaring the condition number of F.
			for (int Sweep = 0; Sweep < 6; ++Sweep)
				for (int P = 0; P < 2; ++P) for (int Q = P + 1; Q < 3; ++Q)
				{
					const Vector A = F.Column(P), B = F.Column(Q);
					const float AA = A.Dot(A), BB = B.Dot(B), AB = A.Dot(B);
					if (fabsf(AB) <= 1.e-7f * sqrtf(AA * BB) || fabsf(AB) < 1.e-20f) continue;
					const float Tau = (BB - AA) / (2 * AB);
					const float T = (Tau >= 0 ? 1.0f : -1.0f) / (fabsf(Tau) + sqrtf(1 + Tau * Tau));
					const float C = 1 / sqrtf(1 + T * T), S = C * T;
					F.SetColumn(P, A * C - B * S); F.SetColumn(Q, A * S + B * C);
					const Vector VP = V.Column(P), VQ = V.Column(Q);
					V.SetColumn(P, VP * C - VQ * S); V.SetColumn(Q, VP * S + VQ * C);
				}
			SVD Result;
			Result.V = V;
			for (int I = 0; I < 3; ++I) Result.Stretch[I] = fmaxf(F.Column(I).Length(), 1.e-6f);
			for (int P = 0; P < 2; ++P) for (int Q = P + 1; Q < 3; ++Q)
				if (Result.Stretch[P] < Result.Stretch[Q])
				{
					const float Temp = Result.Stretch[P]; Result.Stretch[P] = Result.Stretch[Q]; Result.Stretch[Q] = Temp;
					const Vector A = F.Column(P), B = Result.V.Column(P);
					F.SetColumn(P, F.Column(Q)); F.SetColumn(Q, A);
					Result.V.SetColumn(P, Result.V.Column(Q)); Result.V.SetColumn(Q, B);
				}
			Vector U0 = F.Column(0) / Result.Stretch.x;
			if (U0.Length() < 0.5f) U0 = {1,0,0};
			Vector U1 = F.Column(1) - U0 * U0.Dot(F.Column(1));
			if (U1.Length() < 1.e-6f) U1 = U0.Cross(fabsf(U0.x) < 0.8f ? Vector(1,0,0) : Vector(0,1,0));
			U1 = U1 / fmaxf(U1.Length(), 1.e-6f);
			const Vector U2 = U0.Cross(U1) * (Result.V.Determinant() < 0 ? -1.0f : 1.0f);
			Result.U.SetColumn(0, U0); Result.U.SetColumn(1, U1); Result.U.SetColumn(2, U2);
			return Result;
		}

		struct Material
		{
			float YoungModulus = 40000, PoissonRatio = 0.2f, SnowFraction = 1, Viscosity = 2;
			float Hardening = 8, CompressionYield = 0.025f, StretchYield = 0.0075f, FrictionAngle = 35;
		};
		RIEMANN_MPM_HD inline void Lame(const Material& M, float Jp, float& Mu, float& Lambda)
		{
			const float E = M.YoungModulus * expf(Clamp(M.Hardening * (1 - Jp) * M.SnowFraction, -2, 1.6f));
			Mu = E / (2 * (1 + M.PoissonRatio));
			Lambda = E * M.PoissonRatio / ((1 + M.PoissonRatio) * (1 - 2 * M.PoissonRatio));
		}
		RIEMANN_MPM_HD inline Matrix Stress(const Matrix& F, const Matrix& C, float Jp, const Material& M)
		{
			float Mu, Lambda; Lame(M, Jp, Mu, Lambda);
			const SVD S = Decompose(F);
			const Matrix Rotation = S.U * S.V.Transpose();
			const float J = fmaxf(F.Determinant(), 1.e-6f);
			const Matrix Snow = (F - Rotation) * F.Transpose() * (2 * Mu) + Matrix::Identity() * (Lambda * J * (J - 1));
			const Vector Strain(logf(S.Stretch.x), logf(S.Stretch.y), logf(S.Stretch.z));
			const float Trace = Strain.x + Strain.y + Strain.z;
			const Vector Principal = Strain * (2 * Mu) + Vector(1,1,1) * (Lambda * Trace);
			const Matrix Sand = S.U * Diagonal(Principal) * S.U.Transpose();
			return Snow * M.SnowFraction + Sand * (1 - M.SnowFraction) + (C + C.Transpose()) * M.Viscosity;
		}
		RIEMANN_MPM_HD inline Matrix Project(const Matrix& Trial, float& Jp, const Material& M, float* LogVolumeCorrection = nullptr)
		{
			const SVD S = Decompose(Trial);
			float Mu, Lambda; Lame(M, Jp, Mu, Lambda);
			Vector Strain(logf(S.Stretch.x), logf(S.Stretch.y), logf(S.Stretch.z));
			// Stress-free dilation must close before the grains resist compression.
			if (LogVolumeCorrection) Strain = Strain + Vector(1,1,1) * (*LogVolumeCorrection / 3);
			const float Trace = Strain.x + Strain.y + Strain.z;
			const Vector Deviator = Strain - Vector(1,1,1) * (Trace / 3);
			const float Norm = Deviator.Length(), Sine = sinf(M.FrictionAngle * 0.01745329252f);
			const float Alpha = 0.81649658093f * 2 * Sine / (3 - Sine);
			const float Shear = Norm + (3 * Lambda + 2 * Mu) / (2 * Mu) * Trace * Alpha;
			if (Trace >= 0) Strain = {};
			else if (Shear > 0 && Norm > 1.e-8f) Strain = Strain - Deviator * (Shear / Norm);
			Vector Stretch;
			for (int I = 0; I < 3; ++I)
				Stretch[I] = expf(M.SnowFraction * logf(Clamp(S.Stretch[I], 1 - M.CompressionYield, 1 + M.StretchYield)) + (1 - M.SnowFraction) * Strain[I]);
			if (LogVolumeCorrection)
				*LogVolumeCorrection = (1 - M.SnowFraction) * fmaxf(0.0f, *LogVolumeCorrection
					+ logf(fmaxf(Trial.Determinant(), 1.e-6f)) - logf(fmaxf(Stretch.x * Stretch.y * Stretch.z, 1.e-6f)));
			Jp = Clamp(Jp * fmaxf(Trial.Determinant(), 1.e-6f) / fmaxf(Stretch.x * Stretch.y * Stretch.z, 1.e-6f), 0.4f, 4);
			return S.U * Diagonal(Stretch) * S.V.Transpose();
		}
		RIEMANN_MPM_HD inline float Weight(int I, float F)
		{ return I == 0 ? 0.5f * (1.5f-F)*(1.5f-F) : I == 1 ? 0.75f-(F-1)*(F-1) : 0.5f*(F-0.5f)*(F-0.5f); }
		RIEMANN_MPM_HD inline float Derivative(int I, float F) { return I == 0 ? F-1.5f : I == 1 ? -2*(F-1) : F-0.5f; }
	}
}
