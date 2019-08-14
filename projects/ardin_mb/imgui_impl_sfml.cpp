// dear imgui: Platform Binding for SFML
// This needs to be used along with a Renderer (e.g. OpenGL3..)
// (Info: GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan graphics context creation, etc.)
// (Requires: GLFW 3.1+)

// Implemented features:
//  [X] Platform: Clipboard support.
//  [X] Platform: Gamepad support. Enable with 'io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad'.
//  [x] Platform: Mouse cursor shape and visibility. Disable with 'io.ConfigFlags |= ImGuiConfigFlags_NoMouseCursorChange'. FIXME: 3 cursors types are missing from GLFW.
//  [X] Platform: Keyboard arrays indexed using GLFW_KEY_* codes, e.g. ImGui::IsKeyPressed(GLFW_KEY_SPACE).

// You can copy and use unmodified imgui_impl_* files in your project. See main.cpp for an example of using this.
// If you are new to dear imgui, read examples/README.txt and read the documentation at the top of imgui.cpp.
// https://github.com/ocornut/imgui

// CHANGELOG
// (minor and older changes stripped away, please see git history for details)
#include "imgui.h"
#include "imgui_impl_sfml.h"

// SFML
#include <SFML/Main.hpp>


#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#if (SFML_VERSION_MAJOR == 2) && (SFML_VERSION_MINOR >= 5)
    #define SFML_WINDOW_AND_CURSOR_PRESENT

    #define SFML_BACKSPACE sf::Keyboard::Backspace;
#else
    #define SFML_BACKSPACE sf::Keyboard::BackSpace;
#endif

static sf::Window*          g_Window = NULL;
static sf::Clock            g_Clock;
//static bool                 g_MouseJustPressed[5] = { false, false, false, false, false };

#ifdef SFML_WINDOW_AND_CURSOR_PRESENT
static sf::Cursor           g_MouseCursors[ImGuiMouseCursor_COUNT];
#endif
static std::string          g_ClipboardContents;

static const char* ImGui_ImplSfml_GetClipboardText(void*)
{
#ifdef SFML_WINDOW_AND_CURSOR_PRESENT
    g_ClipboardContents = sf::Clipboard::getString();
    return g_ClipboardContents.c_str();
#else
    return "";
#endif
}

static void ImGui_ImplSfml_SetClipboardText(void*, const char* text)
{
#ifdef SFML_WINDOW_AND_CURSOR_PRESENT
    sf::Clipboard::setString(text);
#endif
}

bool ImGui_ImplSfml_Init(sf::Window* window)
{
    g_Window = window;
    g_Clock.restart();

    // Setup back-end capabilities flags
    ImGuiIO& io = ImGui::GetIO();
    io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;         // We can honor GetMouseCursor() values (optional)
    io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;          // We can honor io.WantSetMousePos requests (optional, rarely used)
    io.BackendPlatformName = "imgui_impl_sfml";

    // Keyboard mapping. ImGui will use those indices to peek into the io.KeysDown[] array.
    io.KeyMap[ImGuiKey_Tab] = sf::Keyboard::Tab;
    io.KeyMap[ImGuiKey_LeftArrow] = sf::Keyboard::Left;
    io.KeyMap[ImGuiKey_RightArrow] = sf::Keyboard::Right;
    io.KeyMap[ImGuiKey_UpArrow] = sf::Keyboard::Up;
    io.KeyMap[ImGuiKey_DownArrow] = sf::Keyboard::Down;
    io.KeyMap[ImGuiKey_PageUp] = sf::Keyboard::PageUp;
    io.KeyMap[ImGuiKey_PageDown] = sf::Keyboard::PageDown;
    io.KeyMap[ImGuiKey_Home] = sf::Keyboard::Home;
    io.KeyMap[ImGuiKey_End] = sf::Keyboard::End;
    io.KeyMap[ImGuiKey_Insert] = sf::Keyboard::Insert;
    io.KeyMap[ImGuiKey_Delete] = sf::Keyboard::Delete;
    io.KeyMap[ImGuiKey_Backspace] = SFML_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = sf::Keyboard::Space;
    io.KeyMap[ImGuiKey_Enter] = sf::Keyboard::Return;
    io.KeyMap[ImGuiKey_Escape] = sf::Keyboard::Escape;
    io.KeyMap[ImGuiKey_A] = sf::Keyboard::A;
    io.KeyMap[ImGuiKey_C] = sf::Keyboard::C;
    io.KeyMap[ImGuiKey_V] = sf::Keyboard::V;
    io.KeyMap[ImGuiKey_X] = sf::Keyboard::X;
    io.KeyMap[ImGuiKey_Z] = sf::Keyboard::Z;
    io.KeyMap[ImGuiKey_Y] = sf::Keyboard::Y;

    io.SetClipboardTextFn = ImGui_ImplSfml_SetClipboardText;
    io.GetClipboardTextFn = ImGui_ImplSfml_GetClipboardText;
#if defined(_WIN32)
    io.ImeWindowHandle = (void*)g_Window->getSystemHandle();
#endif

#ifdef SFML_WINDOW_AND_CURSOR_PRESENT
    g_MouseCursors[ImGuiMouseCursor_Arrow].loadFromSystem(sf::Cursor::Arrow);
    g_MouseCursors[ImGuiMouseCursor_TextInput].loadFromSystem(sf::Cursor::Text);
    g_MouseCursors[ImGuiMouseCursor_ResizeAll].loadFromSystem(sf::Cursor::SizeAll);.
    g_MouseCursors[ImGuiMouseCursor_ResizeNS].loadFromSystem(sf::Cursor::SizeVertical);
    g_MouseCursors[ImGuiMouseCursor_ResizeEW].loadFromSystem(sf::Cursor::SizeHorizontal);
    g_MouseCursors[ImGuiMouseCursor_ResizeNESW].loadFromSystem(sf::Cursor::SizeBottomLeftTopRight);
    g_MouseCursors[ImGuiMouseCursor_ResizeNWSE].loadFromSystem(sf::Cursor::SizeTopLeftBottomRight);
    g_MouseCursors[ImGuiMouseCursor_Hand].loadFromSystem(sf::Cursor::Hand);
#endif
    return true;
}

void ImGui_ImplSfml_Shutdown()
{
}

static void ImGui_ImplSfml_UpdateMousePosAndButtons()
{
    // Update buttons
    ImGuiIO& io = ImGui::GetIO();
    for (int i = 0; i < IM_ARRAYSIZE(io.MouseDown); i++)
    {
        // If a mouse press event came, always pass it as "mouse held this frame", so we don't miss click-release events that are shorter than 1 frame.
        io.MouseDown[i] = sf::Mouse::isButtonPressed((sf::Mouse::Button)i);
    }

    // Update mouse position
    const ImVec2 mouse_pos_backup = io.MousePos;
    io.MousePos = ImVec2(-FLT_MAX, -FLT_MAX);
#ifdef __EMSCRIPTEN__
    const bool focused = true; // Emscripten
#else
    const bool focused = g_Window->hasFocus();
#endif
    if (focused)
    {
        if (io.WantSetMousePos)
        {
            sf::Mouse::setPosition(sf::Vector2i(mouse_pos_backup.x, mouse_pos_backup.y), *g_Window);
        }
        else
        {
            sf::Vector2i mouse = sf::Mouse::getPosition(*g_Window);
            io.MousePos = ImVec2((float)mouse.x, (float)mouse.y);
        }
    }
}

static void ImGui_ImplSfml_UpdateMouseCursor()
{
    ImGuiIO& io = ImGui::GetIO();
    if ((io.ConfigFlags & ImGuiConfigFlags_NoMouseCursorChange))
        return;

    ImGuiMouseCursor imgui_cursor = ImGui::GetMouseCursor();
    if (imgui_cursor == ImGuiMouseCursor_None || io.MouseDrawCursor)
    {
        // Hide OS mouse cursor if imgui is drawing it or if it wants no cursor
        g_Window->setMouseCursorVisible(false);
    }
    else
    {
#ifdef SFML_WINDOW_AND_CURSOR_PRESENT
        // Show OS mouse cursor
        g_Window->setMouseCursor(g_MouseCursors[imgui_cursor]);
#endif
        g_Window->setMouseCursorVisible(true);
    }
}

void ImGui_ImplSfml_NewFrame()
{
    ImGuiIO& io = ImGui::GetIO();
    IM_ASSERT(io.Fonts->IsBuilt() && "Font atlas not built! It is generally built by the renderer back-end. Missing call to renderer _NewFrame() function? e.g. ImGui_ImplOpenGL3_NewFrame().");

    // Setup display size (every frame to accommodate for window resizing)
    const auto windowSize = g_Window->getSize();
    io.DisplaySize = ImVec2((float)windowSize.x, (float)windowSize.y);
    io.DisplayFramebufferScale = ImVec2(1, 1);

    // Setup time step
    io.DeltaTime = g_Clock.getElapsedTime().asSeconds();
    g_Clock.restart();

    ImGui_ImplSfml_UpdateMousePosAndButtons();
    ImGui_ImplSfml_UpdateMouseCursor();

    // Gamepad navigation mapping [BETA]
    /*memset(io.NavInputs, 0, sizeof(io.NavInputs));
    if (io.ConfigFlags & ImGuiConfigFlags_NavEnableGamepad)
    {
        // Update gamepad inputs
        #define MAP_BUTTON(NAV_NO, BUTTON_NO)       { if (buttons_count > BUTTON_NO && buttons[BUTTON_NO] == GLFW_PRESS) io.NavInputs[NAV_NO] = 1.0f; }
        #define MAP_ANALOG(NAV_NO, AXIS_NO, V0, V1) { float v = (axes_count > AXIS_NO) ? axes[AXIS_NO] : V0; v = (v - V0) / (V1 - V0); if (v > 1.0f) v = 1.0f; if (io.NavInputs[NAV_NO] < v) io.NavInputs[NAV_NO] = v; }
        int axes_count = 0, buttons_count = 0;
        const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axes_count);
        const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttons_count);
        MAP_BUTTON(ImGuiNavInput_Activate,   0);     // Cross / A
        MAP_BUTTON(ImGuiNavInput_Cancel,     1);     // Circle / B
        MAP_BUTTON(ImGuiNavInput_Menu,       2);     // Square / X
        MAP_BUTTON(ImGuiNavInput_Input,      3);     // Triangle / Y
        MAP_BUTTON(ImGuiNavInput_DpadLeft,   13);    // D-Pad Left
        MAP_BUTTON(ImGuiNavInput_DpadRight,  11);    // D-Pad Right
        MAP_BUTTON(ImGuiNavInput_DpadUp,     10);    // D-Pad Up
        MAP_BUTTON(ImGuiNavInput_DpadDown,   12);    // D-Pad Down
        MAP_BUTTON(ImGuiNavInput_FocusPrev,  4);     // L1 / LB
        MAP_BUTTON(ImGuiNavInput_FocusNext,  5);     // R1 / RB
        MAP_BUTTON(ImGuiNavInput_TweakSlow,  4);     // L1 / LB
        MAP_BUTTON(ImGuiNavInput_TweakFast,  5);     // R1 / RB
        MAP_ANALOG(ImGuiNavInput_LStickLeft, 0,  -0.3f,  -0.9f);
        MAP_ANALOG(ImGuiNavInput_LStickRight,0,  +0.3f,  +0.9f);
        MAP_ANALOG(ImGuiNavInput_LStickUp,   1,  +0.3f,  +0.9f);
        MAP_ANALOG(ImGuiNavInput_LStickDown, 1,  -0.3f,  -0.9f);
        #undef MAP_BUTTON
        #undef MAP_ANALOG
        if (axes_count > 0 && buttons_count > 0)
            io.BackendFlags |= ImGuiBackendFlags_HasGamepad;
        else
            io.BackendFlags &= ~ImGuiBackendFlags_HasGamepad;
    }*/
}

void ImGui_ImplSfml_ProcessEvent(const sf::Event &event)
{
    ImGuiIO& io = ImGui::GetIO();
    if(event.type == sf::Event::TextEntered) {
        if (event.text.unicode > 0 && event.text.unicode < 0x10000) {
            io.AddInputCharacter((unsigned short)event.text.unicode);
        }
    }
    else if(event.type == sf::Event::MouseWheelScrolled) {
        if(event.mouseWheelScroll.wheel == sf::Mouse::HorizontalWheel) {
            io.MouseWheelH += event.mouseWheelScroll.delta;
        }
        else if(event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel) {
            io.MouseWheel += event.mouseWheelScroll.delta;
        }
    }
    else if(event.type == sf::Event::KeyPressed || event.type == sf::Event::KeyReleased) {
        io.KeysDown[event.key.code] = (event.type == sf::Event::KeyPressed);

        // Modifiers are not reliable across systems
        io.KeyCtrl = io.KeysDown[sf::Keyboard::LControl] || io.KeysDown[sf::Keyboard::RControl];
        io.KeyShift = io.KeysDown[sf::Keyboard::LShift] || io.KeysDown[sf::Keyboard::RShift];
        io.KeyAlt = io.KeysDown[sf::Keyboard::LAlt] || io.KeysDown[sf::Keyboard::RAlt];
        //io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
    }
}
