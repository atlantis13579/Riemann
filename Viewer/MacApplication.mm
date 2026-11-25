// macOS application shell for the Riemann viewer.
//
// Mirrors Viewer/WinApplication.cpp: one window, one CAMetalLayer, a dedicated
// render thread and the same fixed 60 Hz simulation step plus the same mouse /
// keyboard routing into WorldViewer.

#import <Cocoa/Cocoa.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cerrno>
#include <string>
#include <vector>

#include <limits.h>
#include <mach-o/dyld.h>
#include <stdlib.h>
#include <unistd.h>

#include "RenderThread.h"
#include "Viewer.h"
#include "../Renderer/RiemannRenderer.h"

namespace
{
	const float kFixedDt = 1.0f / 60.0f;
	const unsigned char kImguiMouseLeft = 0x01;
	const unsigned char kImguiMouseRight = 0x02;

	Riemann::Renderer* g_Renderer = nullptr;
	Riemann::RenderThread* g_RenderThread = nullptr;
	Riemann::WorldViewer* g_Viewer = nullptr;

	int g_ClientWidth = 1280;
	int g_ClientHeight = 800;
	int g_MouseX = 0;
	int g_MouseY = 0;
	unsigned char g_ImguiMouseButtons = 0;
	bool g_ImguiMouseCapture = false;

	bool FileExists(const std::string& fileName)
	{
		FILE* file = std::fopen(fileName.c_str(), "rb");
		if (file == nullptr)
		{
			return false;
		}
		std::fclose(file);
		return true;
	}

	std::string EnsureTrailingSlash(const std::string& path)
	{
		if (path.empty() || path.back() == '/')
		{
			return path;
		}
		return path + "/";
	}

	std::string GetAbsoluteDirectory(const std::string& path)
	{
		char resolved[PATH_MAX] = {};
		if (::realpath(path.empty() ? "." : path.c_str(), resolved) == nullptr)
		{
			return std::string();
		}
		return EnsureTrailingSlash(resolved);
	}

	std::string GetExecutableDirectory()
	{
		uint32_t size = 0;
		_NSGetExecutablePath(nullptr, &size);
		std::vector<char> buffer(size + 1, '\0');
		if (_NSGetExecutablePath(buffer.data(), &size) != 0)
		{
			return std::string();
		}

		char resolved[PATH_MAX] = {};
		const char* path = ::realpath(buffer.data(), resolved) != nullptr ? resolved : buffer.data();
		const std::string executablePath(path);
		const size_t separator = executablePath.find_last_of('/');
		return separator == std::string::npos ? std::string() : executablePath.substr(0, separator + 1);
	}

	std::string ResolveRepositoryRoot()
	{
		std::vector<std::string> candidates;
		candidates.push_back(std::string());
		candidates.push_back("../");
		candidates.push_back("../../");
		candidates.push_back("../../../");
		candidates.push_back("../../../../");

		const std::string executableDirectory = GetExecutableDirectory();
		if (!executableDirectory.empty())
		{
			// A bundled binary lives at Viewer.app/Contents/MacOS/Viewer, so the
			// repository root sits several levels above it.
			std::string upward;
			for (int level = 0; level <= 8; ++level)
			{
				candidates.push_back(executableDirectory + upward);
				upward += "../";
			}
		}

		for (const std::string& candidate : candidates)
		{
			const std::string root = GetAbsoluteDirectory(candidate);
			if (!root.empty()
				&& FileExists(root + "Renderer/metal_shader.metal")
				&& FileExists(root + "Contents/Scenes/01_stack_boxes.json"))
			{
				return root;
			}
		}
		return std::string();
	}

	// Scenes and shaders are resolved relative to the working directory, so pin
	// it to the repository root before anything else reads a file.
	void MakeRepositoryWorkingDirectory(const std::string& root)
	{
		if (root.empty())
		{
			return;
		}
		if (::chdir(root.c_str()) != 0)
		{
		}
	}

	void DiagLog(const char* format, ...)
	{
		FILE* file = std::fopen("/tmp/riemann-diag.log", "a");
		if (file == nullptr)
		{
			return;
		}
		va_list args;
		va_start(args, format);
		std::vfprintf(file, format, args);
		va_end(args);
		std::fputc('\n', file);
		std::fclose(file);
	}

	std::string CommandLineScene()
	{
		NSArray<NSString*>* arguments = [[NSProcessInfo processInfo] arguments];
		if (arguments == nil)
		{
			return std::string();
		}
		for (NSUInteger index = 1; index < [arguments count]; ++index)
		{
			NSString* argument = [arguments objectAtIndex:index];
			if (argument == nil || [argument length] == 0 || [argument hasPrefix:@"-"])
			{
				continue;
			}
			return std::string([argument UTF8String]);
		}
		return std::string();
	}

	bool IsMouseOverImgui()
	{
		if (g_Viewer == nullptr)
		{
			return false;
		}
		return g_Viewer->IsImguiPanelHovered(g_MouseX, g_MouseY, g_ClientWidth, g_ClientHeight);
	}

	void SubmitImguiInput(int wheel)
	{
		if (g_Renderer == nullptr)
		{
			return;
		}

		Riemann::ImguiInputState input;
		input.MouseX = g_MouseX;
		input.MouseY = g_MouseY;
		input.MouseButtons = g_ImguiMouseButtons;
		input.MouseWheel = wheel;
		g_Renderer->UpdateImguiInput(input);
	}

	// NSView is unflipped, the viewer expects top-left origin coordinates.
	void UpdateMousePosition(NSEvent* event, NSView* view)
	{
		if (event == nil || view == nil)
		{
			return;
		}
		const NSPoint point = [view convertPoint:[event locationInWindow] fromView:nil];
		g_MouseX = static_cast<int>(std::lround(point.x));
		g_MouseY = static_cast<int>(std::lround([view bounds].size.height - point.y));
	}

	unsigned char CurrentMouseButtons()
	{
		const NSUInteger pressed = [NSEvent pressedMouseButtons];
		unsigned char buttons = 0;
		if ((pressed & 1u) != 0)
		{
			buttons |= kImguiMouseLeft;
		}
		if ((pressed & 2u) != 0)
		{
			buttons |= kImguiMouseRight;
		}
		return buttons;
	}

	void ClearMovementKeys()
	{
		if (g_Viewer == nullptr)
		{
			return;
		}
		const char keys[] = {'w', 'a', 's', 'd', 'q', 'e', 'i', 'j', 'k', 'l', ' '};
		for (char key : keys)
		{
			g_Viewer->SetMovementKey(key, false);
		}
		g_Viewer->SceneDragEndMsg();
	}

	void HandleMouseMove(NSEvent* event, NSView* view)
	{
		UpdateMousePosition(event, view);
		g_ImguiMouseButtons = CurrentMouseButtons();
		SubmitImguiInput(0);

		if (g_Viewer == nullptr)
		{
			return;
		}

		const bool rotateDown = (g_ImguiMouseButtons & kImguiMouseRight) != 0;
		const bool imguiActive = g_ImguiMouseCapture || IsMouseOverImgui();
		g_Viewer->MouseMsg(g_MouseX, g_MouseY, !imguiActive && rotateDown);
		if (!imguiActive && !rotateDown)
		{
			g_Viewer->MoveSnowCapsule(g_MouseX, g_MouseY, g_ClientWidth, g_ClientHeight);
		}
		if (!imguiActive && (g_ImguiMouseButtons & kImguiMouseLeft) != 0)
		{
			g_Viewer->SceneDragMsg(g_MouseX, g_MouseY, g_ClientWidth, g_ClientHeight);
		}
	}

	void HandleLeftButton(NSEvent* event, NSView* view, bool down)
	{
		UpdateMousePosition(event, view);
		if (down)
		{
			g_ImguiMouseButtons |= kImguiMouseLeft;
			g_ImguiMouseCapture = IsMouseOverImgui();
		}
		else
		{
			g_ImguiMouseButtons &= ~kImguiMouseLeft;
		}
		SubmitImguiInput(0);

		if (g_Viewer == nullptr)
		{
			return;
		}

		g_Viewer->MouseMsg(g_MouseX, g_MouseY, false);
		if (down)
		{
			if (!g_ImguiMouseCapture)
			{
				g_Viewer->SceneRayMsg(g_MouseX, g_MouseY, g_ClientWidth, g_ClientHeight);
			}
		}
		else
		{
			g_Viewer->SceneDragEndMsg();
			g_ImguiMouseCapture = false;
		}
	}

	void HandleRightButton(NSEvent* event, NSView* view, bool down)
	{
		UpdateMousePosition(event, view);
		if (down)
		{
			g_ImguiMouseButtons |= kImguiMouseRight;
			g_ImguiMouseCapture = IsMouseOverImgui();
		}
		else
		{
			g_ImguiMouseButtons &= ~kImguiMouseRight;
		}
		SubmitImguiInput(0);

		if (g_Viewer == nullptr)
		{
			return;
		}

		const bool rotateDown = (g_ImguiMouseButtons & kImguiMouseRight) != 0;
		const bool imguiActive = g_ImguiMouseCapture || IsMouseOverImgui();
		g_Viewer->MouseMsg(g_MouseX, g_MouseY, !imguiActive && rotateDown);
		if (!down)
		{
			g_ImguiMouseCapture = false;
		}
	}

	void HandleScroll(NSEvent* event, NSView* view)
	{
		UpdateMousePosition(event, view);

		const double delta = [event hasPreciseScrollingDeltas]
			? [event scrollingDeltaY] * 4.0
			: [event scrollingDeltaY] * 120.0;
		if (std::fabs(delta) < 1.0)
		{
			return;
		}

		const int steps = static_cast<int>(std::lround(delta));
		SubmitImguiInput(steps > 0 ? -1 : 1);

		if (g_Viewer == nullptr || IsMouseOverImgui())
		{
			return;
		}
		const bool controlDown = ([event modifierFlags] & NSEventModifierFlagControl) != 0;
		g_Viewer->MouseWheel(steps, controlDown);
	}

	void HandleKey(NSEvent* event, bool down)
	{
		if (g_Viewer == nullptr)
		{
			return;
		}
		NSString* characters = [event charactersIgnoringModifiers];
		if (characters == nil || [characters length] == 0)
		{
			return;
		}
		const char key = static_cast<char>([[characters lowercaseString] characterAtIndex:0]);
		g_Viewer->SetMovementKey(key, down);
		if (down)
		{
			g_Viewer->KeyboardMsg(key);
		}
	}

	void InstallApplicationMenu()
	{
		NSMenu* menuBar = [[NSMenu alloc] init];
		NSMenuItem* applicationItem = [[NSMenuItem alloc] init];
		[menuBar addItem:applicationItem];

		NSMenu* applicationMenu = [[NSMenu alloc] init];
		NSString* quitTitle = [@"Quit " stringByAppendingString:[[NSProcessInfo processInfo] processName]];
		NSMenuItem* quitItem = [[NSMenuItem alloc]
			initWithTitle:quitTitle
			action:@selector(terminate:)
			keyEquivalent:@"q"];
		[applicationMenu addItem:quitItem];
		[applicationItem setSubmenu:applicationMenu];

		[NSApp setMainMenu:menuBar];
	}

	// Native replacement for the Win32 GetOpenFileName pickers used by the
	// mesh-cutting and simplification panels.
	std::string MacOpenPanel(const char* extensions, const std::string& initialPath)
	{
		@autoreleasepool
		{
			NSOpenPanel* panel = [NSOpenPanel openPanel];
			[panel setCanChooseFiles:YES];
			[panel setCanChooseDirectories:NO];
			[panel setAllowsMultipleSelection:NO];
			[panel setResolvesAliases:YES];

			NSMutableArray* allowedTypes = [NSMutableArray array];
			const std::string list(extensions != nullptr ? extensions : "");
			size_t start = 0;
			while (start <= list.size())
			{
				const size_t separator = list.find(';', start);
				std::string extension = list.substr(start,
					separator == std::string::npos ? std::string::npos : separator - start);
				while (!extension.empty() && (extension.front() == '*' || extension.front() == '.'))
				{
					extension.erase(extension.begin());
				}
				if (!extension.empty())
				{
					[allowedTypes addObject:[NSString stringWithUTF8String:extension.c_str()]];
				}
				if (separator == std::string::npos)
				{
					break;
				}
				start = separator + 1;
			}
			if ([allowedTypes count] > 0)
			{
				[panel setAllowedFileTypes:allowedTypes];
			}

			if (!initialPath.empty())
			{
				NSString* path = [NSString stringWithUTF8String:initialPath.c_str()];
				if (path != nil)
				{
					[panel setDirectoryURL:[NSURL fileURLWithPath:[path stringByDeletingLastPathComponent]]];
				}
			}

			if ([panel runModal] != NSModalResponseOK)
			{
				return std::string();
			}
			NSURL* url = [[panel URLs] firstObject];
			if (url == nil || [url path] == nil)
			{
				return std::string();
			}
			return std::string([[url path] UTF8String]);
		}
	}
}

@interface RiemannViewerMetalView : NSView
@end

@implementation RiemannViewerMetalView
{
	NSTrackingArea* trackingArea_;
}

- (BOOL)isOpaque
{
	return YES;
}

- (BOOL)acceptsFirstResponder
{
	return YES;
}

- (BOOL)acceptsFirstMouse:(NSEvent*)event
{
	(void)event;
	return YES;
}

- (void)viewDidMoveToWindow
{
	[super viewDidMoveToWindow];
	self.window.acceptsMouseMovedEvents = YES;
	[self.window makeFirstResponder:self];
}

- (void)updateTrackingAreas
{
	[super updateTrackingAreas];
	if (trackingArea_ != nil)
	{
		[self removeTrackingArea:trackingArea_];
		trackingArea_ = nil;
	}
	const NSTrackingAreaOptions options = NSTrackingMouseMoved
		| NSTrackingMouseEnteredAndExited
		| NSTrackingActiveInKeyWindow
		| NSTrackingInVisibleRect;
	trackingArea_ = [[NSTrackingArea alloc] initWithRect:NSZeroRect
		options:options
		owner:self
		userInfo:nil];
	[self addTrackingArea:trackingArea_];
}

- (void)mouseMoved:(NSEvent*)event
{
	HandleMouseMove(event, self);
}

- (void)mouseDragged:(NSEvent*)event
{
	HandleMouseMove(event, self);
}

- (void)rightMouseDragged:(NSEvent*)event
{
	HandleMouseMove(event, self);
}

- (void)otherMouseDragged:(NSEvent*)event
{
	HandleMouseMove(event, self);
}

- (void)mouseDown:(NSEvent*)event
{
	HandleLeftButton(event, self, true);
}

- (void)mouseUp:(NSEvent*)event
{
	HandleLeftButton(event, self, false);
}

- (void)rightMouseDown:(NSEvent*)event
{
	HandleRightButton(event, self, true);
}

- (void)rightMouseUp:(NSEvent*)event
{
	HandleRightButton(event, self, false);
}

- (void)scrollWheel:(NSEvent*)event
{
	HandleScroll(event, self);
}

- (void)keyDown:(NSEvent*)event
{
	HandleKey(event, true);
}

- (void)keyUp:(NSEvent*)event
{
	HandleKey(event, false);
}

@end

@interface RiemannViewerAppDelegate : NSObject <NSApplicationDelegate, NSWindowDelegate>
@end

@implementation RiemannViewerAppDelegate
{
	NSWindow* window_;
	RiemannViewerMetalView* view_;
	CAMetalLayer* layer_;
	NSTimer* timer_;
	int viewportWidth_;
	int viewportHeight_;
	float uiScale_;
	double accumulator_;
	std::chrono::steady_clock::time_point lastTick_;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification
{
	(void)notification;

	const std::string repositoryRoot = ResolveRepositoryRoot();
	DiagLog("exeDir=%s", GetExecutableDirectory().c_str());
	DiagLog("cwd=%s", GetAbsoluteDirectory(".").c_str());
	DiagLog("repoRoot=%s", repositoryRoot.c_str());
	DiagLog("shaderReadable=%d errno=%d", (int)FileExists("/Volumes/ext/src/Riemann/Renderer/metal_shader.metal"),
		(int)errno);
	MakeRepositoryWorkingDirectory(repositoryRoot);

	const NSRect frame = NSMakeRect(80.0, 80.0, 1280.0, 800.0);
	const NSWindowStyleMask style = NSWindowStyleMaskTitled
		| NSWindowStyleMaskClosable
		| NSWindowStyleMaskMiniaturizable
		| NSWindowStyleMaskResizable;
	window_ = [[NSWindow alloc] initWithContentRect:frame
		styleMask:style
		backing:NSBackingStoreBuffered
		defer:NO];
	window_.title = @"Riemann Viewer";
	window_.delegate = self;
	[window_ setReleasedWhenClosed:NO];

	view_ = [[RiemannViewerMetalView alloc] initWithFrame:frame];
	view_.wantsLayer = YES;
	layer_ = [CAMetalLayer layer];
	view_.layer = layer_;
	window_.contentView = view_;

	// The drawable covers the full device resolution so the 3D pass is crisp on
	// Retina panels. The renderer keeps Dear-ImGui in logical points and scales
	// its geometry by UiScale, so the panel layout matches the Windows build.
	const CGFloat scale = std::max<CGFloat>(1.0, window_.backingScaleFactor);
	const NSSize size = view_.bounds.size;
	const int width = std::max(1, static_cast<int>(std::lround(size.width * scale)));
	const int height = std::max(1, static_cast<int>(std::lround(size.height * scale)));
	layer_.contentsScale = scale;
	layer_.drawableSize = CGSizeMake(width, height);
	viewportWidth_ = static_cast<int>(std::lround(size.width));
	viewportHeight_ = static_cast<int>(std::lround(size.height));
	g_ClientWidth = viewportWidth_;
	g_ClientHeight = viewportHeight_;
	uiScale_ = static_cast<float>(scale);

	Riemann::RendererCreateInfo createInfo;
	createInfo.NativeWindow = (void*)layer_;
	createInfo.ShaderPath = (repositoryRoot.empty() ? std::string() : repositoryRoot + "Renderer/");
	createInfo.Width = width;
	createInfo.Height = height;
	createInfo.UiScale = uiScale_;

	g_Renderer = Riemann::Renderer::CreatePlatformRenderer(createInfo);
	if (g_Renderer == nullptr)
	{
		NSAlert* alert = [[NSAlert alloc] init];
		alert.messageText = @"Riemann Viewer could not start";
		alert.informativeText = @"The Metal renderer failed to initialize. Run the app from a checkout so Renderer/metal_shader.metal is reachable.";
		[alert addButtonWithTitle:@"Quit"];
		[alert runModal];
		[NSApp terminate:nil];
		return;
	}
	SubmitImguiInput(0);

	g_RenderThread = new Riemann::RenderThread(g_Renderer);
	g_RenderThread->Start();

	std::string scene = CommandLineScene();
	if (!repositoryRoot.empty())
	{
		const std::string repositoryScene = repositoryRoot + "Contents/Scenes/"
			+ (scene.empty() ? "01_stack_boxes.json" : scene);
		if (scene.empty() || (!FileExists(scene) && FileExists(repositoryScene)))
		{
			scene = repositoryScene;
		}
	}

	g_Viewer = new Riemann::WorldViewer(g_RenderThread, scene.empty() ? "01_stack_boxes.json" : scene);
	g_Viewer->SetViewportSize(viewportWidth_, viewportHeight_);
	g_Viewer->UpdateCamera();

	Riemann::SetFileDialogHandler(&MacOpenPanel);

	accumulator_ = 0.0;
	lastTick_ = std::chrono::steady_clock::now();
	timer_ = [NSTimer timerWithTimeInterval:1.0 / 120.0
		target:self
		selector:@selector(tick:)
		userInfo:nil
		repeats:YES];
	[timer_ setTolerance:0.001];
	[[NSRunLoop mainRunLoop] addTimer:timer_ forMode:NSRunLoopCommonModes];

	[window_ center];
	[window_ makeKeyAndOrderFront:nil];
	[NSApp activateIgnoringOtherApps:YES];
}

- (void)updateViewportSize
{
	if (view_ == nil || layer_ == nil)
	{
		return;
	}

	const CGFloat scale = std::max<CGFloat>(1.0, window_.backingScaleFactor);
	const NSSize size = view_.bounds.size;
	const int width = std::max(1, static_cast<int>(std::lround(size.width)));
	const int height = std::max(1, static_cast<int>(std::lround(size.height)));
	const bool scaleChanged = static_cast<float>(scale) != uiScale_;
	if (width == viewportWidth_ && height == viewportHeight_ && !scaleChanged)
	{
		return;
	}

	viewportWidth_ = width;
	viewportHeight_ = height;
	uiScale_ = static_cast<float>(scale);
	layer_.contentsScale = scale;
	layer_.drawableSize = CGSizeMake(size.width * scale, size.height * scale);
	g_ClientWidth = width;
	g_ClientHeight = height;
	if (g_Viewer != nullptr)
	{
		g_Viewer->SetViewportSize(width, height);
	}
}

- (void)tick:(NSTimer*)timer
{
	(void)timer;
	[self updateViewportSize];
	if (g_Viewer == nullptr)
	{
		return;
	}

	const auto current = std::chrono::steady_clock::now();
	double elapsed = std::chrono::duration<double>(current - lastTick_).count();
	lastTick_ = current;
	if (elapsed < 0.0)
	{
		elapsed = 0.0;
	}
	accumulator_ += std::min(elapsed, 0.25);

	int steps = 0;
	while (accumulator_ >= kFixedDt && steps < 4)
	{
		g_Viewer->UpdateCameraMovement(kFixedDt);
		g_Viewer->UpdateSimulator(kFixedDt);
		accumulator_ -= kFixedDt;
		++steps;
	}
	if (steps == 4)
	{
		// The app fell behind; drop the backlog instead of running long frames.
		accumulator_ = 0.0;
	}
}

- (void)windowDidBecomeKey:(NSNotification*)notification
{
	(void)notification;
	if (g_RenderThread != nullptr)
	{
		g_RenderThread->SetActive(true);
	}
	if (g_Viewer != nullptr)
	{
		g_Viewer->UpdateCamera();
	}
}

- (void)windowDidResignKey:(NSNotification*)notification
{
	(void)notification;
	if (g_RenderThread != nullptr)
	{
		g_RenderThread->SetActive(false);
	}
	ClearMovementKeys();
}

- (void)windowDidResize:(NSNotification*)notification
{
	(void)notification;
	[self updateViewportSize];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender
{
	(void)sender;
	return YES;
}

- (void)applicationWillTerminate:(NSNotification*)notification
{
	(void)notification;

	if (timer_ != nil)
	{
		[timer_ invalidate];
		timer_ = nil;
	}

	// Stop the render thread first so the teardown commands it receives run
	// inline on this thread, exactly like the Win32 WM_DESTROY path.
	if (g_RenderThread != nullptr)
	{
		g_RenderThread->Stop();
	}

	delete g_Viewer;
	g_Viewer = nullptr;

	delete g_RenderThread;
	g_RenderThread = nullptr;

	delete g_Renderer;
	g_Renderer = nullptr;
}

@end

int main(int argc, const char* argv[])
{
	(void)argc;
	(void)argv;
	@autoreleasepool
	{
		NSApplication* application = [NSApplication sharedApplication];
		[application setActivationPolicy:NSApplicationActivationPolicyRegular];
		InstallApplicationMenu();

		RiemannViewerAppDelegate* delegate = [[RiemannViewerAppDelegate alloc] init];
		application.delegate = delegate;
		[application run];
	}
	return 0;
}
