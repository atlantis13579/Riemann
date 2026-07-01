#include "MetalRenderer.h"

#if defined(__APPLE__)

#import <Cocoa/Cocoa.h>
#import <CoreGraphics/CoreGraphics.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <math.h>
#include <mutex>
#include <stdint.h>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"
#include "../Src/Maths/Matrix4.h"

namespace
{
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
		Vector4 OutlineColor;
		Vector4 RenderParams;
	};

	struct MetalImguiUniforms
	{
		float ScreenSize[2];
		float Padding[2];
	};

	struct MetalImguiVertex
	{
		float Pos[2];
		float Tex[2];
		uint32_t Color;
	};

	struct MetalImguiGlyph
	{
		float U0 = 0.0f;
		float V0 = 0.0f;
		float U1 = 0.0f;
		float V1 = 0.0f;
		float Width = 0.0f;
		float Height = 0.0f;
		float Advance = 0.0f;
	};

	const float kImguiScale = 1.0f / 8.0f;
	const float kImguiPi = 3.14159265358979323846f;
	const float kImguiTabStops[4] = { 150.0f, 210.0f, 270.0f, 330.0f };

	void ReleaseObject(id object)
	{
		if (object != nil)
		{
			[object release];
		}
	}

	std::string JoinPath(const std::string& directory, const char* fileName)
	{
		if (directory.empty())
		{
			return fileName;
		}

		const char last = directory[directory.size() - 1];
		if (last == '/' || last == '\\')
		{
			return directory + fileName;
		}
		return directory + "/" + fileName;
	}

	std::string ReadTextFile(const std::string& fileName)
	{
		std::ifstream file(fileName.c_str(), std::ios::in | std::ios::binary);
		if (!file)
		{
			return std::string();
		}

		std::ostringstream ss;
		ss << file.rdbuf();
		return ss.str();
	}

	std::string LoadRendererShaderSource(const std::string& shaderPath)
	{
		const char* shaderFileName = "metal_shader.metal";
		if (!shaderPath.empty())
		{
			std::string source = ReadTextFile(JoinPath(shaderPath, shaderFileName));
			if (!source.empty())
			{
				return source;
			}
		}

		const char* candidates[] =
		{
			"Renderer/",
			"../Renderer/",
			"../../Renderer/",
			"../../../Renderer/",
			"../../../../Renderer/",
		};

		for (const char* candidate : candidates)
		{
			std::string source = ReadTextFile(JoinPath(candidate, shaderFileName));
			if (!source.empty())
			{
				return source;
			}
		}

		return std::string();
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
			FillMode = rhs.FillMode;
			WorldTransform = rhs.WorldTransform;
			Color = rhs.Color;
			CastShadow = rhs.CastShadow;
			OutlineEnabled = rhs.OutlineEnabled;
			OutlineColor = rhs.OutlineColor;
			OutlineThickness = rhs.OutlineThickness;

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
		Riemann::RenderFillMode FillMode = Riemann::RenderFillMode::RendererDefault;
		Transform WorldTransform;
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		bool CastShadow = true;
		bool OutlineEnabled = false;
		Vector4 OutlineColor = Vector4(1.0f, 0.82f, 0.18f, 1.0f);
		float OutlineThickness = 0.06f;
	};
}

namespace Riemann
{
	struct MetalRenderer::Impl
	{
		~Impl()
		{
			ReleaseObject(ImguiSampler);
			ReleaseObject(ImguiDepthState);
			ReleaseObject(ImguiPipeline);
			ReleaseObject(ImguiUniformBuffer);
			ReleaseObject(ImguiWhiteTexture);
			ReleaseObject(ImguiFontTexture);
			ReleaseObject(ShadowSampler);
			ReleaseObject(OutlineMaskDepthState);
			ReleaseObject(OutlineDepthState);
			ReleaseObject(DepthState);
			ReleaseObject(OutlineMaskPipeline);
			ReleaseObject(OutlinePipeline);
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

			if (!CreatePipelineStates(createInfo.ShaderPath))
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
			if (!CreateImguiResources())
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

		bool CreatePipelineStates(const std::string& shaderPath)
		{
			const std::string shaderSource = LoadRendererShaderSource(shaderPath);
			if (shaderSource.empty())
			{
				return false;
			}

			NSError* error = nil;
			NSString* source = [NSString stringWithUTF8String:shaderSource.c_str()];
			id<MTLLibrary> library = [Device newLibraryWithSource:source options:nil error:&error];
			if (library == nil)
			{
				return false;
			}

			id<MTLFunction> mainVS = [library newFunctionWithName:@"RiemannMainVS"];
			id<MTLFunction> mainFS = [library newFunctionWithName:@"RiemannMainFS"];
			id<MTLFunction> shadowVS = [library newFunctionWithName:@"RiemannShadowVS"];
			id<MTLFunction> outlineVS = [library newFunctionWithName:@"RiemannOutlineVS"];
			id<MTLFunction> outlineFS = [library newFunctionWithName:@"RiemannOutlineFS"];
			id<MTLFunction> imguiVS = [library newFunctionWithName:@"RiemannImguiVS"];
			id<MTLFunction> imguiFS = [library newFunctionWithName:@"RiemannImguiFS"];
			if (mainVS == nil || mainFS == nil || shadowVS == nil || outlineVS == nil || outlineFS == nil || imguiVS == nil || imguiFS == nil)
			{
				ReleaseObject(mainVS);
				ReleaseObject(mainFS);
				ReleaseObject(shadowVS);
				ReleaseObject(outlineVS);
				ReleaseObject(outlineFS);
				ReleaseObject(imguiVS);
				ReleaseObject(imguiFS);
				ReleaseObject(library);
				return false;
			}

			MTLRenderPipelineDescriptor* mainDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[mainDesc setVertexFunction:mainVS];
			[mainDesc setFragmentFunction:mainFS];
			MTLRenderPipelineColorAttachmentDescriptor* mainColor = [[mainDesc colorAttachments] objectAtIndexedSubscript:0];
			[mainColor setPixelFormat:ColorPixelFormat];
			[mainDesc setDepthAttachmentPixelFormat:MainDepthStencilPixelFormat];
			[mainDesc setStencilAttachmentPixelFormat:MainDepthStencilPixelFormat];
			MainPipeline = [Device newRenderPipelineStateWithDescriptor:mainDesc error:&error];

			MTLRenderPipelineDescriptor* shadowDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[shadowDesc setVertexFunction:shadowVS];
			[shadowDesc setDepthAttachmentPixelFormat:ShadowDepthPixelFormat];
			ShadowPipeline = [Device newRenderPipelineStateWithDescriptor:shadowDesc error:&error];

			MTLRenderPipelineDescriptor* outlineMaskDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[outlineMaskDesc setVertexFunction:mainVS];
			MTLRenderPipelineColorAttachmentDescriptor* outlineMaskColor = [[outlineMaskDesc colorAttachments] objectAtIndexedSubscript:0];
			[outlineMaskColor setPixelFormat:ColorPixelFormat];
			[outlineMaskColor setWriteMask:MTLColorWriteMaskNone];
			[outlineMaskDesc setDepthAttachmentPixelFormat:MainDepthStencilPixelFormat];
			[outlineMaskDesc setStencilAttachmentPixelFormat:MainDepthStencilPixelFormat];
			OutlineMaskPipeline = [Device newRenderPipelineStateWithDescriptor:outlineMaskDesc error:&error];

			MTLRenderPipelineDescriptor* outlineDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[outlineDesc setVertexFunction:outlineVS];
			[outlineDesc setFragmentFunction:outlineFS];
			MTLRenderPipelineColorAttachmentDescriptor* outlineColor = [[outlineDesc colorAttachments] objectAtIndexedSubscript:0];
			[outlineColor setPixelFormat:ColorPixelFormat];
			[outlineDesc setDepthAttachmentPixelFormat:MainDepthStencilPixelFormat];
			[outlineDesc setStencilAttachmentPixelFormat:MainDepthStencilPixelFormat];
			OutlinePipeline = [Device newRenderPipelineStateWithDescriptor:outlineDesc error:&error];

			MTLRenderPipelineDescriptor* imguiDesc = [[MTLRenderPipelineDescriptor alloc] init];
			[imguiDesc setVertexFunction:imguiVS];
			[imguiDesc setFragmentFunction:imguiFS];
			MTLRenderPipelineColorAttachmentDescriptor* imguiColor = [[imguiDesc colorAttachments] objectAtIndexedSubscript:0];
			[imguiColor setPixelFormat:ColorPixelFormat];
			[imguiColor setBlendingEnabled:YES];
			[imguiColor setSourceRGBBlendFactor:MTLBlendFactorSourceAlpha];
			[imguiColor setDestinationRGBBlendFactor:MTLBlendFactorOneMinusSourceAlpha];
			[imguiColor setRgbBlendOperation:MTLBlendOperationAdd];
			[imguiColor setSourceAlphaBlendFactor:MTLBlendFactorOne];
			[imguiColor setDestinationAlphaBlendFactor:MTLBlendFactorOneMinusSourceAlpha];
			[imguiColor setAlphaBlendOperation:MTLBlendOperationAdd];
			ImguiPipeline = [Device newRenderPipelineStateWithDescriptor:imguiDesc error:&error];

			ReleaseObject(mainDesc);
			ReleaseObject(shadowDesc);
			ReleaseObject(outlineMaskDesc);
			ReleaseObject(outlineDesc);
			ReleaseObject(imguiDesc);
			ReleaseObject(mainVS);
			ReleaseObject(mainFS);
			ReleaseObject(shadowVS);
			ReleaseObject(outlineVS);
			ReleaseObject(outlineFS);
			ReleaseObject(imguiVS);
			ReleaseObject(imguiFS);
			ReleaseObject(library);

			return MainPipeline != nil && ShadowPipeline != nil && OutlineMaskPipeline != nil && OutlinePipeline != nil && ImguiPipeline != nil;
		}

		bool CreateDepthState()
		{
			ReleaseObject(DepthState);
			ReleaseObject(OutlineMaskDepthState);
			ReleaseObject(OutlineDepthState);
			DepthState = nil;
			OutlineMaskDepthState = nil;
			OutlineDepthState = nil;

			MTLDepthStencilDescriptor* desc = [[MTLDepthStencilDescriptor alloc] init];
			[desc setDepthCompareFunction:MTLCompareFunctionLess];
			[desc setDepthWriteEnabled:YES];
			DepthState = [Device newDepthStencilStateWithDescriptor:desc];
			ReleaseObject(desc);
			if (DepthState == nil)
			{
				return false;
			}

			MTLStencilDescriptor* maskStencil = [[MTLStencilDescriptor alloc] init];
			[maskStencil setStencilCompareFunction:MTLCompareFunctionAlways];
			[maskStencil setStencilFailureOperation:MTLStencilOperationKeep];
			[maskStencil setDepthFailureOperation:MTLStencilOperationKeep];
			[maskStencil setDepthStencilPassOperation:MTLStencilOperationReplace];
			[maskStencil setReadMask:0xff];
			[maskStencil setWriteMask:0xff];

			MTLDepthStencilDescriptor* maskDesc = [[MTLDepthStencilDescriptor alloc] init];
			[maskDesc setDepthCompareFunction:MTLCompareFunctionAlways];
			[maskDesc setDepthWriteEnabled:NO];
			[maskDesc setFrontFaceStencil:maskStencil];
			[maskDesc setBackFaceStencil:maskStencil];
			OutlineMaskDepthState = [Device newDepthStencilStateWithDescriptor:maskDesc];
			ReleaseObject(maskDesc);
			ReleaseObject(maskStencil);
			if (OutlineMaskDepthState == nil)
			{
				return false;
			}

			MTLStencilDescriptor* outlineStencil = [[MTLStencilDescriptor alloc] init];
			[outlineStencil setStencilCompareFunction:MTLCompareFunctionNotEqual];
			[outlineStencil setStencilFailureOperation:MTLStencilOperationKeep];
			[outlineStencil setDepthFailureOperation:MTLStencilOperationKeep];
			[outlineStencil setDepthStencilPassOperation:MTLStencilOperationKeep];
			[outlineStencil setReadMask:0xff];
			[outlineStencil setWriteMask:0x00];

			MTLDepthStencilDescriptor* outlineDesc = [[MTLDepthStencilDescriptor alloc] init];
			[outlineDesc setDepthCompareFunction:MTLCompareFunctionAlways];
			[outlineDesc setDepthWriteEnabled:NO];
			[outlineDesc setFrontFaceStencil:outlineStencil];
			[outlineDesc setBackFaceStencil:outlineStencil];
			OutlineDepthState = [Device newDepthStencilStateWithDescriptor:outlineDesc];
			ReleaseObject(outlineDesc);
			ReleaseObject(outlineStencil);
			return OutlineDepthState != nil;
		}

		bool CreateShadowResources()
		{
			ReleaseObject(ShadowTexture);
			ReleaseObject(ShadowSampler);
			ShadowTexture = nil;
			ShadowSampler = nil;

			MTLTextureDescriptor* textureDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:ShadowDepthPixelFormat width:ShadowMapSize height:ShadowMapSize mipmapped:NO];
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

		bool CreateImguiResources()
		{
			MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
			[depthDesc setDepthCompareFunction:MTLCompareFunctionAlways];
			[depthDesc setDepthWriteEnabled:NO];
			ImguiDepthState = [Device newDepthStencilStateWithDescriptor:depthDesc];
			ReleaseObject(depthDesc);
			if (ImguiDepthState == nil)
			{
				return false;
			}

			MTLSamplerDescriptor* samplerDesc = [[MTLSamplerDescriptor alloc] init];
			[samplerDesc setMinFilter:MTLSamplerMinMagFilterLinear];
			[samplerDesc setMagFilter:MTLSamplerMinMagFilterLinear];
			[samplerDesc setMipFilter:MTLSamplerMipFilterNotMipmapped];
			[samplerDesc setSAddressMode:MTLSamplerAddressModeClampToEdge];
			[samplerDesc setTAddressMode:MTLSamplerAddressModeClampToEdge];
			ImguiSampler = [Device newSamplerStateWithDescriptor:samplerDesc];
			ReleaseObject(samplerDesc);
			if (ImguiSampler == nil)
			{
				return false;
			}

			ImguiUniformBuffer = [Device newBufferWithLength:sizeof(MetalImguiUniforms) options:MTLResourceStorageModeShared];
			if (ImguiUniformBuffer == nil)
			{
				return false;
			}

			const unsigned char whitePixel[4] = { 255, 255, 255, 255 };
			ImguiWhiteTexture = CreateImguiTexture(1, 1, whitePixel);
			if (ImguiWhiteTexture == nil)
			{
				return false;
			}

			return CreateImguiFontTexture();
		}

		id<MTLTexture> CreateImguiTexture(int width, int height, const unsigned char* pixels)
		{
			if (width <= 0 || height <= 0 || pixels == nullptr)
			{
				return nil;
			}

			MTLTextureDescriptor* textureDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
				width:static_cast<NSUInteger>(width)
				height:static_cast<NSUInteger>(height)
				mipmapped:NO];
			[textureDesc setUsage:MTLTextureUsageShaderRead];
			[textureDesc setStorageMode:MTLStorageModeManaged];
			id<MTLTexture> texture = [Device newTextureWithDescriptor:textureDesc];
			if (texture == nil)
			{
				return nil;
			}

			MTLRegion region = MTLRegionMake2D(0, 0, static_cast<NSUInteger>(width), static_cast<NSUInteger>(height));
			[texture replaceRegion:region mipmapLevel:0 withBytes:pixels bytesPerRow:static_cast<NSUInteger>(width * 4)];
			return texture;
		}

		bool CreateImguiFontTexture()
		{
			const int atlasWidth = 512;
			const int atlasHeight = 512;
			std::vector<unsigned char> pixels(static_cast<size_t>(atlasWidth * atlasHeight * 4), 0);

			@autoreleasepool
			{
				CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
				if (colorSpace == nullptr)
				{
					return false;
				}

				CGContextRef context = CGBitmapContextCreate(
					pixels.data(),
					atlasWidth,
					atlasHeight,
					8,
					atlasWidth * 4,
					colorSpace,
					kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
				CGColorSpaceRelease(colorSpace);
				if (context == nullptr)
				{
					return false;
				}

				NSGraphicsContext* graphicsContext = [NSGraphicsContext graphicsContextWithCGContext:context flipped:YES];
				[NSGraphicsContext saveGraphicsState];
				[NSGraphicsContext setCurrentContext:graphicsContext];
				[graphicsContext setShouldAntialias:YES];

				NSFont* font = [NSFont systemFontOfSize:15.0];
				NSDictionary* attributes = [NSDictionary dictionaryWithObjectsAndKeys:
					font, NSFontAttributeName,
					[NSColor whiteColor], NSForegroundColorAttributeName,
					nil];

				NSSize measure = [@"M" sizeWithAttributes:attributes];
				ImguiFontHeight = std::max(8, static_cast<int>(std::ceil(measure.height)));

				int penX = 1;
				int penY = 1;
				const int rowHeight = ImguiFontHeight + 4;
				for (int c = 32; c < 128; ++c)
				{
					char ch = static_cast<char>(c);
					NSString* string = [[NSString alloc] initWithBytes:&ch length:1 encoding:NSASCIIStringEncoding];
					NSSize size = [string sizeWithAttributes:attributes];
					const int glyphWidth = std::max(1, static_cast<int>(std::ceil(size.width)));
					const int glyphHeight = ImguiFontHeight;

					if (penX + glyphWidth + 2 >= atlasWidth)
					{
						penX = 1;
						penY += rowHeight;
					}
					if (penY + glyphHeight + 2 >= atlasHeight)
					{
						[string release];
						break;
					}

					MetalImguiGlyph& glyph = ImguiGlyphs[c - 32];
					glyph.U0 = penX / static_cast<float>(atlasWidth);
					glyph.V0 = penY / static_cast<float>(atlasHeight);
					glyph.U1 = (penX + glyphWidth) / static_cast<float>(atlasWidth);
					glyph.V1 = (penY + glyphHeight) / static_cast<float>(atlasHeight);
					glyph.Width = static_cast<float>(glyphWidth);
					glyph.Height = static_cast<float>(glyphHeight);
					glyph.Advance = static_cast<float>(glyphWidth);

					[string drawAtPoint:NSMakePoint(static_cast<CGFloat>(penX), static_cast<CGFloat>(penY)) withAttributes:attributes];
					[string release];
					penX += glyphWidth + 1;
				}

				[NSGraphicsContext restoreGraphicsState];
				CGContextRelease(context);
			}

			ImguiFontTexture = CreateImguiTexture(atlasWidth, atlasHeight, pixels.data());
			return ImguiFontTexture != nil;
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

			MTLTextureDescriptor* textureDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MainDepthStencilPixelFormat width:Width height:Height mipmapped:NO];
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
			const float nearPlane = camera.NearPlane > 0.05f ? camera.NearPlane : 0.05f;
			const float farPlane = camera.FarPlane > nearPlane + 1.0f ? camera.FarPlane : nearPlane + 1.0f;
			Camera.NearPlane = nearPlane;
			Camera.FarPlane = farPlane;
			Eye = camera.Eye;
			View = Transform3::BuildViewMatrix_LHCoordinateSystem(camera.Eye, camera.At, camera.Up);
			const float aspect = Height != 0 ? Width / static_cast<float>(Height) : 1.0f;
			Projection = Transform3::BuildPerspectiveMatrix_LHCoordinateSystem(camera.FovY, aspect, nearPlane, farPlane);
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

		void SetImguiDrawCallback(ImguiDrawCallback callback, void* userData)
		{
			std::lock_guard<std::mutex> lock(ImguiMutex);
			ImguiCallback = callback;
			ImguiUserData = userData;
		}

		void UpdateImguiInput(const ImguiInputState& input)
		{
			std::lock_guard<std::mutex> lock(ImguiMutex);
			ImguiInput.MouseX = input.MouseX;
			ImguiInput.MouseY = input.MouseY;
			ImguiInput.MouseButtons = input.MouseButtons;
			ImguiScroll += input.MouseWheel;
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
			mesh.FillMode = meshDesc.FillMode;
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

		bool SetMeshOutline(const char* id, bool enabled, const Vector4& color, float thickness)
		{
			if (id == nullptr)
			{
				return false;
			}

			for (MetalMesh& mesh : Meshes)
			{
				if (mesh.Id == id)
				{
					mesh.OutlineEnabled = enabled;
					mesh.OutlineColor = color;
					mesh.OutlineThickness = thickness > 0.0f ? thickness : 0.0f;
					return true;
				}
			}
			return false;
		}

		MTLTriangleFillMode ResolveTriangleFillMode(RenderFillMode fillMode) const
		{
			return IsWireframeFillMode(fillMode) ? MTLTriangleFillModeLines : MTLTriangleFillModeFill;
		}

		bool IsWireframeFillMode(RenderFillMode fillMode) const
		{
			return fillMode == RenderFillMode::Wireframe ||
				(fillMode == RenderFillMode::RendererDefault && Wireframe);
		}

		bool ShouldRenderUnlit(const MetalMesh& mesh, bool shadowPass, bool outlinePass) const
		{
			if (shadowPass || outlinePass)
			{
				return false;
			}
			return mesh.PrimitiveType == MTLPrimitiveTypeLine ||
				(mesh.PrimitiveType == MTLPrimitiveTypeTriangle && IsWireframeFillMode(mesh.FillMode));
		}

		void UpdateUniforms(MetalMesh& mesh, bool shadowPass, bool outlinePass)
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
			uniforms->OutlineColor = mesh.OutlineColor;
			uniforms->RenderParams = Vector4(outlinePass ? mesh.OutlineThickness : 0.0f, ShouldRenderUnlit(mesh, shadowPass, outlinePass) ? 1.0f : 0.0f, 0.0f, 0.0f);
		}

		void DrawMesh(id<MTLRenderCommandEncoder> encoder, MetalMesh& mesh, bool shadowPass, bool outlinePass)
		{
			if (shadowPass && !mesh.CastShadow)
			{
				return;
			}

			UpdateUniforms(mesh, shadowPass, outlinePass);
			[encoder setVertexBuffer:mesh.VertexBuffer offset:0 atIndex:0];
			[encoder setVertexBuffer:mesh.UniformBuffer offset:0 atIndex:1];
			if (!shadowPass)
			{
				[encoder setFragmentBuffer:mesh.UniformBuffer offset:0 atIndex:1];
			}
			if (!outlinePass)
			{
				[encoder setTriangleFillMode:ResolveTriangleFillMode(mesh.FillMode)];
			}

			[encoder drawIndexedPrimitives:mesh.PrimitiveType
				indexCount:mesh.IndexCount
				indexType:MTLIndexTypeUInt32
				indexBuffer:mesh.IndexBuffer
				indexBufferOffset:0];
		}

		float ImguiToScreenY(float y) const
		{
			return static_cast<float>(Height) - y;
		}

		void SetImguiScissor(id<MTLRenderCommandEncoder> encoder, int x, int y, int w, int h)
		{
			const int maxX = std::max(1, Width) - 1;
			const int maxY = std::max(1, Height) - 1;
			const int left = std::max(0, std::min(x, maxX));
			const int top = std::max(0, std::min(Height - (y + h), maxY));
			const int right = std::max(left + 1, std::min(std::max(1, Width), x + w));
			const int bottom = std::max(top + 1, std::min(std::max(1, Height), Height - y));

			MTLScissorRect rect = {};
			rect.x = static_cast<NSUInteger>(left);
			rect.y = static_cast<NSUInteger>(top);
			rect.width = static_cast<NSUInteger>(right - left);
			rect.height = static_cast<NSUInteger>(bottom - top);
			[encoder setScissorRect:rect];
		}

		void DrawImguiVertices(id<MTLRenderCommandEncoder> encoder, const MetalImguiVertex* vertices, size_t vertexCount, id<MTLTexture> texture)
		{
			if (vertices == nullptr || vertexCount == 0 || texture == nil)
			{
				return;
			}

			id<MTLBuffer> vertexBuffer = [Device newBufferWithBytes:vertices
				length:sizeof(MetalImguiVertex) * vertexCount
				options:MTLResourceStorageModeShared];
			if (vertexBuffer == nil)
			{
				return;
			}

			[encoder setVertexBuffer:vertexBuffer offset:0 atIndex:0];
			[encoder setFragmentTexture:texture atIndex:0];
			[encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:static_cast<NSUInteger>(vertexCount)];
			ReleaseObject(vertexBuffer);
		}

		void AddImguiRect(id<MTLRenderCommandEncoder> encoder, float x, float y, float w, float h, unsigned int color)
		{
			const float left = x;
			const float right = x + w;
			const float top = ImguiToScreenY(y + h);
			const float bottom = ImguiToScreenY(y);
			const MetalImguiVertex vertices[6] =
			{
				{ { left, top }, { 0.0f, 0.0f }, color },
				{ { right, top }, { 0.0f, 0.0f }, color },
				{ { right, bottom }, { 0.0f, 0.0f }, color },
				{ { left, top }, { 0.0f, 0.0f }, color },
				{ { right, bottom }, { 0.0f, 0.0f }, color },
				{ { left, bottom }, { 0.0f, 0.0f }, color },
			};
			DrawImguiVertices(encoder, vertices, 6, ImguiWhiteTexture);
		}

		void AddImguiRoundedRect(id<MTLRenderCommandEncoder> encoder, float x, float y, float w, float h, float radius, unsigned int color)
		{
			radius = std::max(0.0f, std::min(radius, std::min(w, h) * 0.5f));
			if (radius <= 0.5f)
			{
				AddImguiRect(encoder, x, y, w, h, color);
				return;
			}

			const float left = x;
			const float right = x + w;
			const float top = ImguiToScreenY(y + h);
			const float bottom = ImguiToScreenY(y);
			const float r = radius;
			const int segments = 6;

			std::vector<MetalImguiVertex> points;
			points.reserve(28);
			const float centers[4][2] =
			{
				{ right - r, top + r },
				{ right - r, bottom - r },
				{ left + r, bottom - r },
				{ left + r, top + r },
			};
			const float startAngles[4] =
			{
				-kImguiPi * 0.5f,
				0.0f,
				kImguiPi * 0.5f,
				kImguiPi,
			};

			for (int corner = 0; corner < 4; ++corner)
			{
				for (int i = 0; i <= segments; ++i)
				{
					const float angle = startAngles[corner] + (kImguiPi * 0.5f) * (static_cast<float>(i) / static_cast<float>(segments));
					MetalImguiVertex vertex = {};
					vertex.Pos[0] = centers[corner][0] + std::cos(angle) * r;
					vertex.Pos[1] = centers[corner][1] + std::sin(angle) * r;
					vertex.Color = color;
					points.push_back(vertex);
				}
			}

			MetalImguiVertex center = {};
			center.Pos[0] = (left + right) * 0.5f;
			center.Pos[1] = (top + bottom) * 0.5f;
			center.Color = color;

			std::vector<MetalImguiVertex> vertices;
			vertices.reserve(points.size() * 3);
			for (size_t i = 0; i < points.size(); ++i)
			{
				vertices.push_back(center);
				vertices.push_back(points[i]);
				vertices.push_back(points[(i + 1) % points.size()]);
			}
			DrawImguiVertices(encoder, vertices.data(), vertices.size(), ImguiWhiteTexture);
		}

		void AddImguiLine(id<MTLRenderCommandEncoder> encoder, float x0, float y0, float x1, float y1, float radius, unsigned int color)
		{
			y0 = ImguiToScreenY(y0);
			y1 = ImguiToScreenY(y1);

			float dx = x1 - x0;
			float dy = y1 - y0;
			const float len = std::sqrt(dx * dx + dy * dy);
			if (len <= 0.0001f)
			{
				return;
			}
			dx /= len;
			dy /= len;

			const float halfWidth = std::max(1.0f, radius) * 0.5f;
			const float nx = -dy * halfWidth;
			const float ny = dx * halfWidth;
			const MetalImguiVertex vertices[6] =
			{
				{ { x0 - nx, y0 - ny }, { 0.0f, 0.0f }, color },
				{ { x0 + nx, y0 + ny }, { 0.0f, 0.0f }, color },
				{ { x1 + nx, y1 + ny }, { 0.0f, 0.0f }, color },
				{ { x0 - nx, y0 - ny }, { 0.0f, 0.0f }, color },
				{ { x1 + nx, y1 + ny }, { 0.0f, 0.0f }, color },
				{ { x1 - nx, y1 - ny }, { 0.0f, 0.0f }, color },
			};
			DrawImguiVertices(encoder, vertices, 6, ImguiWhiteTexture);
		}

		void AddImguiTriangle(id<MTLRenderCommandEncoder> encoder, float x, float y, float w, float h, int flags, unsigned int color)
		{
			MetalImguiVertex vertices[3] = {};
			if (flags == 1)
			{
				vertices[0].Pos[0] = x;
				vertices[0].Pos[1] = ImguiToScreenY(y);
				vertices[1].Pos[0] = x + w;
				vertices[1].Pos[1] = ImguiToScreenY(y + h * 0.5f);
				vertices[2].Pos[0] = x;
				vertices[2].Pos[1] = ImguiToScreenY(y + h);
			}
			else
			{
				vertices[0].Pos[0] = x;
				vertices[0].Pos[1] = ImguiToScreenY(y + h);
				vertices[1].Pos[0] = x + w * 0.5f;
				vertices[1].Pos[1] = ImguiToScreenY(y);
				vertices[2].Pos[0] = x + w;
				vertices[2].Pos[1] = ImguiToScreenY(y + h);
			}
			vertices[0].Color = color;
			vertices[1].Color = color;
			vertices[2].Color = color;
			DrawImguiVertices(encoder, vertices, 3, ImguiWhiteTexture);
		}

		float MeasureImguiText(const char* text) const
		{
			if (text == nullptr)
			{
				return 0.0f;
			}

			float x = 0.0f;
			float length = 0.0f;
			while (*text)
			{
				const unsigned char c = static_cast<unsigned char>(*text);
				if (c == '\t')
				{
					for (float tabStop : kImguiTabStops)
					{
						if (x < tabStop)
						{
							x = tabStop;
							break;
						}
					}
					length = std::max(length, x);
				}
				else if (c >= 32 && c < 128)
				{
					const MetalImguiGlyph& glyph = ImguiGlyphs[c - 32];
					length = std::max(length, x + glyph.Width);
					x += glyph.Advance;
				}
				++text;
			}
			return length;
		}

		void AddImguiText(id<MTLRenderCommandEncoder> encoder, float x, float y, int align, const char* text, unsigned int color)
		{
			if (text == nullptr || ImguiFontTexture == nil)
			{
				return;
			}

			if (align == IMGUI_ALIGN_CENTER)
			{
				x -= MeasureImguiText(text) * 0.5f;
			}
			else if (align == IMGUI_ALIGN_RIGHT)
			{
				x -= MeasureImguiText(text);
			}

			const float originX = x;
			const float top = ImguiToScreenY(y + static_cast<float>(ImguiFontHeight));
			std::vector<MetalImguiVertex> vertices;
			vertices.reserve(std::strlen(text) * 6);

			while (*text)
			{
				const unsigned char c = static_cast<unsigned char>(*text);
				if (c == '\t')
				{
					for (float tabStop : kImguiTabStops)
					{
						if (x < originX + tabStop)
						{
							x = originX + tabStop;
							break;
						}
					}
				}
				else if (c >= 32 && c < 128)
				{
					const MetalImguiGlyph& glyph = ImguiGlyphs[c - 32];
					const float left = x;
					const float right = x + glyph.Width;
					const float bottom = top + glyph.Height;

					vertices.push_back({ { left, top }, { glyph.U0, glyph.V0 }, color });
					vertices.push_back({ { right, top }, { glyph.U1, glyph.V0 }, color });
					vertices.push_back({ { right, bottom }, { glyph.U1, glyph.V1 }, color });
					vertices.push_back({ { left, top }, { glyph.U0, glyph.V0 }, color });
					vertices.push_back({ { right, bottom }, { glyph.U1, glyph.V1 }, color });
					vertices.push_back({ { left, bottom }, { glyph.U0, glyph.V1 }, color });
					x += glyph.Advance;
				}
				++text;
			}

			if (!vertices.empty())
			{
				DrawImguiVertices(encoder, vertices.data(), vertices.size(), ImguiFontTexture);
			}
		}

		void DrawImguiQueue(id<MTLRenderCommandEncoder> encoder)
		{
			const imguiGfxCmd* commands = imguiGetRenderQueue();
			const int commandCount = imguiGetRenderQueueSize();
			for (int i = 0; i < commandCount; ++i)
			{
				const imguiGfxCmd& cmd = commands[i];
				if (cmd.type == IMGUI_GFXCMD_RECT)
				{
					const float x = static_cast<float>(cmd.rect.x) * kImguiScale + 0.5f;
					const float y = static_cast<float>(cmd.rect.y) * kImguiScale + 0.5f;
					const float w = static_cast<float>(cmd.rect.w) * kImguiScale - 1.0f;
					const float h = static_cast<float>(cmd.rect.h) * kImguiScale - 1.0f;
					const float r = static_cast<float>(cmd.rect.r) * kImguiScale;
					if (cmd.rect.r == 0)
					{
						AddImguiRect(encoder, x, y, w, h, cmd.col);
					}
					else
					{
						AddImguiRoundedRect(encoder, x, y, w, h, r, cmd.col);
					}
				}
				else if (cmd.type == IMGUI_GFXCMD_LINE)
				{
					AddImguiLine(
						encoder,
						static_cast<float>(cmd.line.x0) * kImguiScale,
						static_cast<float>(cmd.line.y0) * kImguiScale,
						static_cast<float>(cmd.line.x1) * kImguiScale,
						static_cast<float>(cmd.line.y1) * kImguiScale,
						static_cast<float>(cmd.line.r) * kImguiScale,
						cmd.col);
				}
				else if (cmd.type == IMGUI_GFXCMD_TRIANGLE)
				{
					AddImguiTriangle(
						encoder,
						static_cast<float>(cmd.rect.x) * kImguiScale + 0.5f,
						static_cast<float>(cmd.rect.y) * kImguiScale + 0.5f,
						static_cast<float>(cmd.rect.w) * kImguiScale - 1.0f,
						static_cast<float>(cmd.rect.h) * kImguiScale - 1.0f,
						cmd.flags,
						cmd.col);
				}
				else if (cmd.type == IMGUI_GFXCMD_TEXT)
				{
					AddImguiText(encoder, static_cast<float>(cmd.text.x), static_cast<float>(cmd.text.y), cmd.text.align, cmd.text.text, cmd.col);
				}
				else if (cmd.type == IMGUI_GFXCMD_SCISSOR)
				{
					if (cmd.flags)
					{
						SetImguiScissor(encoder, cmd.rect.x, cmd.rect.y, cmd.rect.w, cmd.rect.h);
					}
					else
					{
						SetImguiScissor(encoder, 0, 0, Width, Height);
					}
				}
			}
		}

		void RenderImgui(id<MTLRenderCommandEncoder> encoder)
		{
			if (ImguiPipeline == nil || ImguiDepthState == nil || ImguiUniformBuffer == nil || ImguiSampler == nil)
			{
				return;
			}

			ImguiDrawCallback callback = nullptr;
			void* userData = nullptr;
			ImguiInputState input;
			{
				std::lock_guard<std::mutex> lock(ImguiMutex);
				callback = ImguiCallback;
				userData = ImguiUserData;
				input = ImguiInput;
				input.MouseWheel = ImguiScroll;
				ImguiScroll = 0;
			}

			if (callback == nullptr)
			{
				return;
			}

			input.MouseY = Height - 1 - input.MouseY;
			imguiBeginFrame(input.MouseX, input.MouseY, input.MouseButtons, input.MouseWheel);
			callback(Width, Height, userData);
			imguiEndFrame();

			MetalImguiUniforms* uniforms = static_cast<MetalImguiUniforms*>([ImguiUniformBuffer contents]);
			uniforms->ScreenSize[0] = static_cast<float>(std::max(1, Width));
			uniforms->ScreenSize[1] = static_cast<float>(std::max(1, Height));
			uniforms->Padding[0] = 0.0f;
			uniforms->Padding[1] = 0.0f;

			[encoder setRenderPipelineState:ImguiPipeline];
			[encoder setDepthStencilState:ImguiDepthState];
			[encoder setCullMode:MTLCullModeNone];
			[encoder setTriangleFillMode:MTLTriangleFillModeFill];
			[encoder setVertexBuffer:ImguiUniformBuffer offset:0 atIndex:1];
			[encoder setFragmentSamplerState:ImguiSampler atIndex:0];
			SetImguiScissor(encoder, 0, 0, Width, Height);
			DrawImguiQueue(encoder);
			SetImguiScissor(encoder, 0, 0, Width, Height);
		}

		void RenderOutline(id<MTLRenderCommandEncoder> encoder)
		{
			if (OutlineMaskPipeline == nil || OutlinePipeline == nil || OutlineMaskDepthState == nil || OutlineDepthState == nil)
			{
				return;
			}

			bool hasOutline = false;
			for (const MetalMesh& mesh : Meshes)
			{
				if (mesh.OutlineEnabled && mesh.PrimitiveType == MTLPrimitiveTypeTriangle)
				{
					hasOutline = true;
					break;
				}
			}
			if (!hasOutline)
			{
				return;
			}

			[encoder setRenderPipelineState:OutlineMaskPipeline];
			[encoder setDepthStencilState:OutlineMaskDepthState];
			[encoder setStencilReferenceValue:1];
			[encoder setCullMode:MTLCullModeNone];
			[encoder setTriangleFillMode:MTLTriangleFillModeFill];
			for (MetalMesh& mesh : Meshes)
			{
				if (!mesh.OutlineEnabled || mesh.PrimitiveType != MTLPrimitiveTypeTriangle)
				{
					continue;
				}

				DrawMesh(encoder, mesh, false, true);
			}

			[encoder setRenderPipelineState:OutlinePipeline];
			[encoder setDepthStencilState:OutlineDepthState];
			[encoder setStencilReferenceValue:1];
			[encoder setCullMode:MTLCullModeFront];
			[encoder setTriangleFillMode:MTLTriangleFillModeFill];
			for (MetalMesh& mesh : Meshes)
			{
				if (!mesh.OutlineEnabled || mesh.PrimitiveType != MTLPrimitiveTypeTriangle)
				{
					continue;
				}

				DrawMesh(encoder, mesh, false, true);
			}
		}

		void Render()
		{
			if (Layer == nil || CommandQueue == nil || MainPipeline == nil || ShadowPipeline == nil || OutlineMaskPipeline == nil || OutlinePipeline == nil || ShadowTexture == nil)
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
				[shadowEncoder setTriangleFillMode:ResolveTriangleFillMode(RenderFillMode::RendererDefault)];
				for (MetalMesh& mesh : Meshes)
				{
					DrawMesh(shadowEncoder, mesh, true, false);
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
				[[mainPass stencilAttachment] setTexture:DepthTexture];
				[[mainPass stencilAttachment] setLoadAction:MTLLoadActionClear];
				[[mainPass stencilAttachment] setStoreAction:MTLStoreActionDontCare];
				[[mainPass stencilAttachment] setClearStencil:0];

				id<MTLRenderCommandEncoder> mainEncoder = [commandBuffer renderCommandEncoderWithDescriptor:mainPass];
				[mainEncoder setRenderPipelineState:MainPipeline];
				[mainEncoder setDepthStencilState:DepthState];
				[mainEncoder setCullMode:MTLCullModeNone];
				[mainEncoder setTriangleFillMode:ResolveTriangleFillMode(RenderFillMode::RendererDefault)];
				[mainEncoder setFragmentTexture:ShadowTexture atIndex:0];
				[mainEncoder setFragmentSamplerState:ShadowSampler atIndex:0];
				for (MetalMesh& mesh : Meshes)
				{
					DrawMesh(mainEncoder, mesh, false, false);
				}
				RenderOutline(mainEncoder);
				RenderImgui(mainEncoder);
				[mainEncoder endEncoding];

				[commandBuffer presentDrawable:drawable];
				[commandBuffer commit];
			}
		}

		MTLPixelFormat ColorPixelFormat = MTLPixelFormatBGRA8Unorm;
		MTLPixelFormat MainDepthStencilPixelFormat = MTLPixelFormatDepth32Float_Stencil8;
		MTLPixelFormat ShadowDepthPixelFormat = MTLPixelFormatDepth32Float;
		NSUInteger ShadowMapSize = 2048;

		id<MTLDevice> Device = nil;
		id<MTLCommandQueue> CommandQueue = nil;
		CAMetalLayer* Layer = nil;
		id<MTLRenderPipelineState> MainPipeline = nil;
		id<MTLRenderPipelineState> ShadowPipeline = nil;
		id<MTLRenderPipelineState> OutlineMaskPipeline = nil;
		id<MTLRenderPipelineState> OutlinePipeline = nil;
		id<MTLRenderPipelineState> ImguiPipeline = nil;
		id<MTLDepthStencilState> DepthState = nil;
		id<MTLDepthStencilState> OutlineMaskDepthState = nil;
		id<MTLDepthStencilState> OutlineDepthState = nil;
		id<MTLDepthStencilState> ImguiDepthState = nil;
		id<MTLTexture> DepthTexture = nil;
		id<MTLTexture> ShadowTexture = nil;
		id<MTLTexture> ImguiWhiteTexture = nil;
		id<MTLTexture> ImguiFontTexture = nil;
		id<MTLSamplerState> ShadowSampler = nil;
		id<MTLSamplerState> ImguiSampler = nil;
		id<MTLBuffer> ImguiUniformBuffer = nil;

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

		std::mutex ImguiMutex;
		ImguiInputState ImguiInput;
		ImguiDrawCallback ImguiCallback = nullptr;
		void* ImguiUserData = nullptr;
		int ImguiScroll = 0;
		MetalImguiGlyph ImguiGlyphs[96];
		int ImguiFontHeight = 16;
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

	bool MetalRenderer::SetMeshOutline(const char* id, bool enabled, const Vector4& color, float thickness)
	{
		return m_Impl->SetMeshOutline(id, enabled, color, thickness);
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

	void MetalRenderer::SetImguiDrawCallback(ImguiDrawCallback callback, void* userData)
	{
		m_Impl->SetImguiDrawCallback(callback, userData);
	}

	void MetalRenderer::UpdateImguiInput(const ImguiInputState& input)
	{
		m_Impl->UpdateImguiInput(input);
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
