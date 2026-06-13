#include "MetalRenderer.h"

#if defined(__APPLE__)

#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <math.h>
#include <string>
#include <utility>
#include <vector>

#include "../Src/Maths/Matrix4.h"

namespace
{
	const char* kMetalShaderSource = R"MSL(
#include <metal_stdlib>
using namespace metal;

struct MetalVertex
{
	packed_float3 position;
	packed_float3 normal;
};

struct Uniforms
{
	float4x4 world;
	float4x4 view;
	float4x4 projection;
	float4x4 lightView;
	float4x4 lightProjection;
	float4 eyePos;
	float4 lightDir;
	float4 lightColor;
	float4 materialColor;
};

struct VSOut
{
	float4 position [[position]];
	float3 positionWorld;
	float3 normal;
	float4 positionLight;
};

struct ShadowOut
{
	float4 position [[position]];
};

float4 MulRow(float4 value, float4x4 matrixValue)
{
	return float4(
		dot(value, matrixValue[0]),
		dot(value, matrixValue[1]),
		dot(value, matrixValue[2]),
		dot(value, matrixValue[3]));
}

vertex VSOut RiemannMainVS(
	uint vertexId [[vertex_id]],
	const device MetalVertex* vertices [[buffer(0)]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	MetalVertex vertex = vertices[vertexId];

	float4 localPosition = float4(float3(vertex.position), 1.0);
	float4 worldPosition = MulRow(localPosition, uniforms.world);

	VSOut out;
	out.positionWorld = worldPosition.xyz;
	out.position = MulRow(MulRow(worldPosition, uniforms.view), uniforms.projection);
	out.positionLight = MulRow(MulRow(worldPosition, uniforms.lightView), uniforms.lightProjection);
	out.normal = normalize(MulRow(float4(float3(vertex.normal), 0.0), uniforms.world).xyz);
	return out;
}

vertex ShadowOut RiemannShadowVS(
	uint vertexId [[vertex_id]],
	const device MetalVertex* vertices [[buffer(0)]],
	constant Uniforms& uniforms [[buffer(1)]])
{
	MetalVertex vertex = vertices[vertexId];
	float4 localPosition = float4(float3(vertex.position), 1.0);
	float4 worldPosition = MulRow(localPosition, uniforms.world);

	ShadowOut out;
	out.position = MulRow(MulRow(worldPosition, uniforms.lightView), uniforms.lightProjection);
	return out;
}

float SampleShadow(float4 lightPosition, depth2d<float> shadowMap, sampler shadowSampler)
{
	if (fabs(lightPosition.w) < 1.0e-6)
	{
		return 1.0;
	}

	float3 projected = lightPosition.xyz / lightPosition.w;
	float2 uv = float2(projected.x * 0.5 + 0.5, 0.5 - projected.y * 0.5);
	if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || projected.z < 0.0 || projected.z > 1.0)
	{
		return 1.0;
	}

	const float bias = 0.0015;
	return shadowMap.sample_compare(shadowSampler, uv, projected.z - bias);
}

fragment float4 RiemannMainFS(
	VSOut in [[stage_in]],
	constant Uniforms& uniforms [[buffer(1)]],
	depth2d<float> shadowMap [[texture(0)]],
	sampler shadowSampler [[sampler(0)]])
{
	float3 normal = normalize(in.normal);
	float3 lightToScene = normalize(uniforms.lightDir.xyz);
	float3 toLight = -lightToScene;
	float3 toEye = normalize(uniforms.eyePos.xyz - in.positionWorld);
	float3 halfVector = normalize(toLight + toEye);

	float ndotl = saturate(dot(normal, toLight));
	float specular = pow(saturate(dot(normal, halfVector)), 64.0);
	float shadow = SampleShadow(in.positionLight, shadowMap, shadowSampler);

	float3 baseColor = uniforms.materialColor.rgb;
	float ambient = uniforms.lightDir.w;
	float3 diffuse = baseColor * uniforms.lightColor.rgb * ndotl * shadow;
	float3 spec = uniforms.lightColor.rgb * specular * shadow * 0.35;
	return float4(baseColor * ambient + diffuse + spec, uniforms.materialColor.a);
}
)MSL";

	struct MetalUniforms
	{
		Matrix4 World;
		Matrix4 View;
		Matrix4 Projection;
		Matrix4 LightView;
		Matrix4 LightProjection;
		Vector4 EyePos;
		Vector4 LightDir;
		Vector4 LightColor;
		Vector4 MaterialColor;
	};

	void ReleaseObject(id object)
	{
		if (object != nil)
		{
			[object release];
		}
	}

	Vector3 SafeUnit(const Vector3& value, const Vector3& fallback)
	{
		const float lenSq = value.SquareLength();
		if (lenSq <= 1.0e-8f)
		{
			return fallback;
		}
		return value / sqrtf(lenSq);
	}

	Matrix4 GetTransformMatrix(const Transform& transform)
	{
		return Transform3::Compose(transform.pos, transform.quat);
	}

	struct MetalMesh
	{
		MetalMesh() {}
		MetalMesh(const MetalMesh&) = delete;
		MetalMesh& operator=(const MetalMesh&) = delete;

		MetalMesh(MetalMesh&& rhs) noexcept
		{
			MoveFrom(rhs);
		}

		MetalMesh& operator=(MetalMesh&& rhs) noexcept
		{
			if (this != &rhs)
			{
				Release();
				MoveFrom(rhs);
			}
			return *this;
		}

		~MetalMesh()
		{
			Release();
		}

		void Release()
		{
			ReleaseObject(VertexBuffer);
			ReleaseObject(IndexBuffer);
			ReleaseObject(UniformBuffer);
			VertexBuffer = nil;
			IndexBuffer = nil;
			UniformBuffer = nil;
		}

		void MoveFrom(MetalMesh& rhs)
		{
			Id = std::move(rhs.Id);
			VertexBuffer = rhs.VertexBuffer;
			IndexBuffer = rhs.IndexBuffer;
			UniformBuffer = rhs.UniformBuffer;
			VertexCount = rhs.VertexCount;
			IndexCount = rhs.IndexCount;
			PrimitiveType = rhs.PrimitiveType;
			WorldTransform = rhs.WorldTransform;
			Color = rhs.Color;
			CastShadow = rhs.CastShadow;

			rhs.VertexBuffer = nil;
			rhs.IndexBuffer = nil;
			rhs.UniformBuffer = nil;
			rhs.VertexCount = 0;
			rhs.IndexCount = 0;
		}

		std::string Id;
		id<MTLBuffer> VertexBuffer = nil;
		id<MTLBuffer> IndexBuffer = nil;
		id<MTLBuffer> UniformBuffer = nil;
		NSUInteger VertexCount = 0;
		NSUInteger IndexCount = 0;
		MTLPrimitiveType PrimitiveType = MTLPrimitiveTypeTriangle;
		Transform WorldTransform;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		bool CastShadow = true;
	};
}

namespace Riemann
{
	struct MetalRenderer::Impl
	{
		~Impl()
		{
			ReleaseObject(ShadowSampler);
			ReleaseObject(DepthState);
			ReleaseObject(MainPipeline);
			ReleaseObject(ShadowPipeline);
			ReleaseObject(DepthTexture);
			ReleaseObject(ShadowTexture);
			ReleaseObject(CommandQueue);
			ReleaseObject(Layer);
			ReleaseObject(Device);
		}

		bool Init(const RendererCreateInfo& createInfo)
		{
			Width = std::max(1, createInfo.Width);
			Height = std::max(1, createInfo.Height);

			Device = MTLCreateSystemDefaultDevice();
			if (Device == nil)
			{
				return false;
			}

			CommandQueue = [Device newCommandQueue];
			if (CommandQueue == nil)
			{
				return false;
			}

			if (!AttachLayer(createInfo.NativeWindow))
			{
				return false;
			}

			if (!CreatePipelineStates())
			{
				return false;
			}
			if (!CreateDepthState())
			{
				return false;
			}
			if (!CreateShadowResources())
			{
				return false;
			}

			SetCamera(CameraDesc());
			SetLight(DirectionalLightDesc());
			return true;
		}

		bool AttachLayer(void* nativeWindow)
		{
			if (nativeWindow == nullptr)
			{
				return true;
			}

			id nativeObject = (id)nativeWindow;
			NSView* view = nil;
			if ([nativeObject isKindOfClass:[CAMetalLayer class]])
			{
				Layer = (CAMetalLayer*)[nativeObject retain];
			}
			else if ([nativeObject isKindOfClass:[NSWindow class]])
			{
				view = [(NSWindow*)nativeObject contentView];
			}
			else if ([nativeObject isKindOfClass:[NSView class]])
			{
				view = (NSView*)nativeObject;
			}

			if (view != nil)
			{
				[view setWantsLayer:YES];
				CALayer* existingLayer = [view layer];
				if ([existingLayer isKindOfClass:[CAMetalLayer class]])
				{
					Layer = (CAMetalLayer*)[existingLayer retain];
				}
				else
				{
					CAMetalLayer* metalLayer = [[CAMetalLayer alloc] init];
					[view setLayer:metalLayer];
					Layer = metalLayer;
				}
			}

			if (Layer == nil)
			{
				return false;
			}

			[Layer setDevice:Device];
			[Layer setPixelFormat:ColorPixelFormat];
			[Layer setFramebufferOnly:YES];
			[Layer setPresentsWithTransaction:NO];
			UpdateLayerDrawableSize();
			return true;
		}

		bool CreatePipelineStates()
		{
			NSError* error = nil;
			NSString* source = [NSString stringWithUTF8String:kMetalShaderSource];
			id<MTLLibrary> library = [Device newLibraryWithSource:source options:nil error:&error];
			if (library == nil)
			{
				return false;
			}

			id<MTLFunction> mainVS = [library newFunctionWithName:@"RiemannMainVS"];
			id<MTLFunction> mainFS = [library newFunctionWithName:@"RiemannMainFS"];
			id<MTLFunction> shadowVS = [library newFunctionWithName:@"RiemannShadowVS"];
			if (mainVS == nil || mainFS == nil || shadowVS == nil)
			{
				ReleaseObject(mainVS);
				ReleaseObject(mainFS);
				ReleaseObject(shadowVS);
				ReleaseObject(library);
				return false;
			}

			MTLRenderPipelineDescriptor* mainDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[mainDesc setVertexFunction:mainVS];
			[mainDesc setFragmentFunction:mainFS];
			MTLRenderPipelineColorAttachmentDescriptor* mainColor = [[mainDesc colorAttachments] objectAtIndexedSubscript:0];
			[mainColor setPixelFormat:ColorPixelFormat];
			[mainDesc setDepthAttachmentPixelFormat:DepthPixelFormat];
			MainPipeline = [Device newRenderPipelineStateWithDescriptor:mainDesc error:&error];

			MTLRenderPipelineDescriptor* shadowDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[shadowDesc setVertexFunction:shadowVS];
			[shadowDesc setDepthAttachmentPixelFormat:DepthPixelFormat];
			ShadowPipeline = [Device newRenderPipelineStateWithDescriptor:shadowDesc error:&error];

			ReleaseObject(mainDesc);
			ReleaseObject(shadowDesc);
			ReleaseObject(mainVS);
			ReleaseObject(mainFS);
			ReleaseObject(shadowVS);
			ReleaseObject(library);

			return MainPipeline != nil && ShadowPipeline != nil;
		}

		bool CreateDepthState()
		{
			ReleaseObject(DepthState);
			DepthState = nil;

			MTLDepthStencilDescriptor* desc = [[MTLDepthStencilDescriptor alloc] init];
			[desc setDepthCompareFunction:MTLCompareFunctionLess];
			[desc setDepthWriteEnabled:YES];
			DepthState = [Device newDepthStencilStateWithDescriptor:desc];
			ReleaseObject(desc);
			return DepthState != nil;
		}

		bool CreateShadowResources()
		{
			ReleaseObject(ShadowTexture);
			ReleaseObject(ShadowSampler);
			ShadowTexture = nil;
			ShadowSampler = nil;

			MTLTextureDescriptor* textureDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:DepthPixelFormat width:ShadowMapSize height:ShadowMapSize mipmapped:NO];
			[textureDesc setUsage:MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead];
			[textureDesc setStorageMode:MTLStorageModePrivate];
			ShadowTexture = [Device newTextureWithDescriptor:textureDesc];
			if (ShadowTexture == nil)
			{
				return false;
			}

			MTLSamplerDescriptor* samplerDesc = [[MTLSamplerDescriptor alloc] init];
			[samplerDesc setMinFilter:MTLSamplerMinMagFilterLinear];
			[samplerDesc setMagFilter:MTLSamplerMinMagFilterLinear];
			[samplerDesc setMipFilter:MTLSamplerMipFilterNotMipmapped];
			[samplerDesc setSAddressMode:MTLSamplerAddressModeClampToEdge];
			[samplerDesc setTAddressMode:MTLSamplerAddressModeClampToEdge];
			[samplerDesc setCompareFunction:MTLCompareFunctionLessEqual];
			ShadowSampler = [Device newSamplerStateWithDescriptor:samplerDesc];
			ReleaseObject(samplerDesc);
			return ShadowSampler != nil;
		}

		bool EnsureDepthTexture()
		{
			UpdateLayerDrawableSize();

			if (DepthTexture != nil && DepthWidth == Width && DepthHeight == Height)
			{
				return true;
			}

			ReleaseObject(DepthTexture);
			DepthTexture = nil;

			MTLTextureDescriptor* textureDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:DepthPixelFormat width:Width height:Height mipmapped:NO];
			[textureDesc setUsage:MTLTextureUsageRenderTarget];
			[textureDesc setStorageMode:MTLStorageModePrivate];
			DepthTexture = [Device newTextureWithDescriptor:textureDesc];
			DepthWidth = Width;
			DepthHeight = Height;
			return DepthTexture != nil;
		}

		void UpdateLayerDrawableSize()
		{
			if (Layer == nil)
			{
				return;
			}

			CGSize drawableSize = [Layer drawableSize];
			if (drawableSize.width <= 0.0 || drawableSize.height <= 0.0)
			{
				drawableSize = CGSizeMake(static_cast<CGFloat>(Width), static_cast<CGFloat>(Height));
				[Layer setDrawableSize:drawableSize];
			}

			Width = std::max(1, static_cast<int>(drawableSize.width));
			Height = std::max(1, static_cast<int>(drawableSize.height));
		}

		void SetCamera(const CameraDesc& camera)
		{
			Camera = camera;
			Eye = camera.Eye;
			View = Transform3::BuildViewMatrix_LHCoordinateSystem(camera.Eye, camera.At, camera.Up);
			const float aspect = Height != 0 ? Width / static_cast<float>(Height) : 1.0f;
			Projection = Transform3::BuildPerspectiveMatrix_LHCoordinateSystem(camera.FovY, aspect, camera.NearPlane, camera.FarPlane);
		}

		void SetLight(const DirectionalLightDesc& light)
		{
			Light = light;
			Light.Direction = SafeUnit(light.Direction, Vector3(-0.4f, -1.0f, 0.35f));

			Vector3 up = Vector3::UnitY();
			if (fabsf(Light.Direction.Dot(up)) > 0.95f)
			{
				up = Vector3::UnitZ();
			}

			const Vector3 lightEye = Light.ShadowCenter - Light.Direction * Light.ShadowDistance;
			LightView = Transform3::BuildViewMatrix_LHCoordinateSystem(lightEye, Light.ShadowCenter, up);
			LightProjection = Transform3::BuildOrthogonalMatrix_LHCoordinateSystem(
				Light.ShadowSize,
				Light.ShadowSize,
				0.1f,
				Light.ShadowDistance * 2.0f);
		}

		bool AddMesh(const RenderMeshDesc& meshDesc)
		{
			if (meshDesc.Id.empty() || meshDesc.Vertices.empty() || meshDesc.Indices.empty())
			{
				return false;
			}

			DeleteMesh(meshDesc.Id.c_str());

			MetalMesh mesh;
			mesh.Id = meshDesc.Id;
			mesh.VertexCount = static_cast<NSUInteger>(meshDesc.Vertices.size());
			mesh.IndexCount = static_cast<NSUInteger>(meshDesc.Indices.size());
			mesh.PrimitiveType = meshDesc.Topology == RenderPrimitiveTopology::Lines ? MTLPrimitiveTypeLine : MTLPrimitiveTypeTriangle;
			mesh.WorldTransform = meshDesc.WorldTransform;
			mesh.Color = meshDesc.Color;
			mesh.CastShadow = meshDesc.CastShadow && meshDesc.Topology == RenderPrimitiveTopology::Triangles;

			mesh.VertexBuffer = [Device newBufferWithBytes:meshDesc.Vertices.data() length:sizeof(Vertex1) * meshDesc.Vertices.size() options:MTLResourceStorageModeShared];
			mesh.IndexBuffer = [Device newBufferWithBytes:meshDesc.Indices.data() length:sizeof(uint32_t) * meshDesc.Indices.size() options:MTLResourceStorageModeShared];
			mesh.UniformBuffer = [Device newBufferWithLength:sizeof(MetalUniforms) options:MTLResourceStorageModeShared];
			if (mesh.VertexBuffer == nil || mesh.IndexBuffer == nil || mesh.UniformBuffer == nil)
			{
				return false;
			}

			Meshes.push_back(std::move(mesh));
			return true;
		}

		bool UpdateTransform(const RenderTransformUpdate& transform)
		{
			for (MetalMesh& mesh : Meshes)
			{
				if (mesh.Id == transform.Id)
				{
					mesh.WorldTransform = transform.WorldTransform;
					return true;
				}
			}
			return false;
		}

		bool UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount)
		{
			if (id == nullptr || vertices == nullptr || vertexCount <= 0)
			{
				return false;
			}

			for (MetalMesh& mesh : Meshes)
			{
				if (mesh.Id == id)
				{
					id<MTLBuffer> replacement = [Device newBufferWithBytes:vertices length:sizeof(Vertex1) * static_cast<size_t>(vertexCount) options:MTLResourceStorageModeShared];
					if (replacement == nil)
					{
						return false;
					}
					ReleaseObject(mesh.VertexBuffer);
					mesh.VertexBuffer = replacement;
					mesh.VertexCount = static_cast<NSUInteger>(vertexCount);
					return true;
				}
			}
			return false;
		}

		bool DeleteMesh(const char* id)
		{
			if (id == nullptr)
			{
				return false;
			}

			for (size_t i = 0; i < Meshes.size(); ++i)
			{
				if (Meshes[i].Id == id)
				{
					Meshes.erase(Meshes.begin() + i);
					return true;
				}
			}
			return false;
		}

		void UpdateUniforms(MetalMesh& mesh)
		{
			MetalUniforms* uniforms = static_cast<MetalUniforms*>([mesh.UniformBuffer contents]);
			uniforms->World = GetTransformMatrix(mesh.WorldTransform);
			uniforms->View = View;
			uniforms->Projection = Projection;
			uniforms->LightView = LightView;
			uniforms->LightProjection = LightProjection;
			uniforms->EyePos = Vector4(Eye.x, Eye.y, Eye.z, 1.0f);
			uniforms->LightDir = Vector4(Light.Direction.x, Light.Direction.y, Light.Direction.z, Light.Ambient);
			uniforms->LightColor = Vector4(Light.Color.x, Light.Color.y, Light.Color.z, 1.0f);
			uniforms->MaterialColor = mesh.Color;
		}

		void DrawMesh(id<MTLRenderCommandEncoder> encoder, MetalMesh& mesh, bool shadowPass)
		{
			if (shadowPass && !mesh.CastShadow)
			{
				return;
			}

			UpdateUniforms(mesh);
			[encoder setVertexBuffer:mesh.VertexBuffer offset:0 atIndex:0];
			[encoder setVertexBuffer:mesh.UniformBuffer offset:0 atIndex:1];
			if (!shadowPass)
			{
				[encoder setFragmentBuffer:mesh.UniformBuffer offset:0 atIndex:1];
			}

			[encoder drawIndexedPrimitives:mesh.PrimitiveType
				indexCount:mesh.IndexCount
				indexType:MTLIndexTypeUInt32
				indexBuffer:mesh.IndexBuffer
				indexBufferOffset:0];
		}

		void Render()
		{
			if (Layer == nil || CommandQueue == nil || MainPipeline == nil || ShadowPipeline == nil || ShadowTexture == nil)
			{
				return;
			}

			@autoreleasepool
			{
				if (!EnsureDepthTexture())
				{
					return;
				}

				id<CAMetalDrawable> drawable = [Layer nextDrawable];
				if (drawable == nil)
				{
					return;
				}

				id<MTLCommandBuffer> commandBuffer = [CommandQueue commandBuffer];
				if (commandBuffer == nil)
				{
					return;
				}

				MTLRenderPassDescriptor* shadowPass = [MTLRenderPassDescriptor renderPassDescriptor];
				[[shadowPass depthAttachment] setTexture:ShadowTexture];
				[[shadowPass depthAttachment] setLoadAction:MTLLoadActionClear];
				[[shadowPass depthAttachment] setStoreAction:MTLStoreActionStore];
				[[shadowPass depthAttachment] setClearDepth:1.0];

				id<MTLRenderCommandEncoder> shadowEncoder = [commandBuffer renderCommandEncoderWithDescriptor:shadowPass];
				[shadowEncoder setRenderPipelineState:ShadowPipeline];
				[shadowEncoder setDepthStencilState:DepthState];
				[shadowEncoder setCullMode:MTLCullModeNone];
				[shadowEncoder setTriangleFillMode:Wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
				for (MetalMesh& mesh : Meshes)
				{
					DrawMesh(shadowEncoder, mesh, true);
				}
				[shadowEncoder endEncoding];

				MTLRenderPassDescriptor* mainPass = [MTLRenderPassDescriptor renderPassDescriptor];
				MTLRenderPassColorAttachmentDescriptor* mainColorAttachment = [[mainPass colorAttachments] objectAtIndexedSubscript:0];
				[mainColorAttachment setTexture:[drawable texture]];
				[mainColorAttachment setLoadAction:MTLLoadActionClear];
				[mainColorAttachment setStoreAction:MTLStoreActionStore];
				[mainColorAttachment setClearColor:MTLClearColorMake(0.015, 0.018, 0.024, 1.0)];
				[[mainPass depthAttachment] setTexture:DepthTexture];
				[[mainPass depthAttachment] setLoadAction:MTLLoadActionClear];
				[[mainPass depthAttachment] setStoreAction:MTLStoreActionDontCare];
				[[mainPass depthAttachment] setClearDepth:1.0];

				id<MTLRenderCommandEncoder> mainEncoder = [commandBuffer renderCommandEncoderWithDescriptor:mainPass];
				[mainEncoder setRenderPipelineState:MainPipeline];
				[mainEncoder setDepthStencilState:DepthState];
				[mainEncoder setCullMode:MTLCullModeNone];
				[mainEncoder setTriangleFillMode:Wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
				[mainEncoder setFragmentTexture:ShadowTexture atIndex:0];
				[mainEncoder setFragmentSamplerState:ShadowSampler atIndex:0];
				for (MetalMesh& mesh : Meshes)
				{
					DrawMesh(mainEncoder, mesh, false);
				}
				[mainEncoder endEncoding];

				[commandBuffer presentDrawable:drawable];
				[commandBuffer commit];
			}
		}

		MTLPixelFormat ColorPixelFormat = MTLPixelFormatBGRA8Unorm;
		MTLPixelFormat DepthPixelFormat = MTLPixelFormatDepth32Float;
		NSUInteger ShadowMapSize = 2048;

		id<MTLDevice> Device = nil;
		id<MTLCommandQueue> CommandQueue = nil;
		CAMetalLayer* Layer = nil;
		id<MTLRenderPipelineState> MainPipeline = nil;
		id<MTLRenderPipelineState> ShadowPipeline = nil;
		id<MTLDepthStencilState> DepthState = nil;
		id<MTLTexture> DepthTexture = nil;
		id<MTLTexture> ShadowTexture = nil;
		id<MTLSamplerState> ShadowSampler = nil;

		int Width = 1024;
		int Height = 768;
		int DepthWidth = 0;
		int DepthHeight = 0;
		bool Wireframe = false;

		CameraDesc Camera;
		DirectionalLightDesc Light;
		Matrix4 View;
		Matrix4 Projection;
		Matrix4 LightView;
		Matrix4 LightProjection;
		Vector3 Eye;
		std::vector<MetalMesh> Meshes;
	};

	MetalRenderer::MetalRenderer()
		: m_Impl(new Impl())
	{
	}

	MetalRenderer::~MetalRenderer()
	{
		delete m_Impl;
	}

	bool MetalRenderer::Init(const RendererCreateInfo& createInfo)
	{
		return m_Impl->Init(createInfo);
	}

	void MetalRenderer::Render()
	{
		m_Impl->Render();
	}

	void MetalRenderer::SetCamera(const CameraDesc& camera)
	{
		m_Impl->SetCamera(camera);
	}

	void MetalRenderer::SetLight(const DirectionalLightDesc& light)
	{
		m_Impl->SetLight(light);
	}

	bool MetalRenderer::AddMesh(const RenderMeshDesc& mesh)
	{
		return m_Impl->AddMesh(mesh);
	}

	bool MetalRenderer::UpdateTransform(const RenderTransformUpdate& transform)
	{
		return m_Impl->UpdateTransform(transform);
	}

	bool MetalRenderer::UpdateVertices(const char* id, const Vertex1* vertices, int vertexCount)
	{
		return m_Impl->UpdateVertices(id, vertices, vertexCount);
	}

	bool MetalRenderer::DeleteMesh(const char* id)
	{
		return m_Impl->DeleteMesh(id);
	}

	bool MetalRenderer::Reset()
	{
		m_Impl->Meshes.clear();
		return true;
	}

	void MetalRenderer::SetFillMode(bool wireframe)
	{
		m_Impl->Wireframe = wireframe;
	}

	void MetalRenderer::SetDepthMode()
	{
		(void)m_Impl->CreateDepthState();
	}

	Renderer* Renderer::CreateMetalRenderer(const RendererCreateInfo& createInfo)
	{
		MetalRenderer* renderer = new MetalRenderer();
		if (renderer->Init(createInfo))
		{
			return renderer;
		}

		delete renderer;
		return nullptr;
	}
}

#endif
