// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
#include "stdafx.h"
#include "photo_mode.h"
#include "hfr_blend.h"

#include <report/zip.h>
#include <video/presenter.h>
#include <ui/ui.h>
#include <user/settings.h>
#include <kernel/function.h>
#include <hid/input.h>
#include <imgui.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <future>
#include <string>

namespace game::photo_mode
{
    namespace
    {
        std::atomic<bool> s_active{ false };
        std::atomic<bool> s_needsCapture{ false };
        std::atomic<bool> s_freezeWorld{ true };
        std::atomic<bool> s_hideHud{ true };
        std::atomic<bool> s_showOverlay{ true };
        std::atomic<bool> s_invertLookY{ false };
        std::atomic<bool> s_invertLookX{ false };

        // Diagnostic readings for on-screen telemetry
        float s_dbgLx = 0.0f, s_dbgLy = 0.0f;
        float s_dbgRx = 0.0f, s_dbgRy = 0.0f;
        float s_dbgLt = 0.0f, s_dbgRt = 0.0f;

        // Camera pose in world coordinates (+X right, +Y up, +Z forward)
        hfr::blend::V3 s_pos{ 0.0f, 0.0f, 0.0f };
        float s_yaw = 0.0f;    // Radians around global Y (+Y up)
        float s_pitch = 0.0f;  // Radians elevation (clamped)
        float s_roll = 0.0f;   // Radians camera tilt around Forward vector
        float s_fov = 10923.0f; // 16-bit angle (~60 deg)

        // Keyboard states
        bool s_keyForward = false;
        bool s_keyBackward = false;
        bool s_keyLeft = false;
        bool s_keyRight = false;
        bool s_keyUp = false;
        bool s_keyDown = false;
        bool s_keyTurbo = false;
        bool s_keySlow = false;
        bool s_keyRollLeft = false;
        bool s_keyRollRight = false;
        bool s_mouseLooking = false;

        // Controller analog states
        float s_stickMoveX = 0.0f;
        float s_stickMoveY = 0.0f;
        float s_stickLookX = 0.0f;
        float s_stickLookY = 0.0f;
        float s_triggerUp = 0.0f;
        float s_triggerDown = 0.0f;

        // Controller chords
        bool s_padBack = false;
        bool s_padR3 = false;

        std::filesystem::path GetScreenshotsDirectory()
        {
            const char* home = std::getenv("HOME");
            if (!home || !*home)
                return {};
            std::filesystem::path dir = std::filesystem::path(home) / "Pictures" / "SpeedBreaker";
            std::error_code ec;
            std::filesystem::create_directories(dir, ec);
            return dir;
        }

        std::string FormatTimestamp()
        {
            auto now = std::chrono::system_clock::now();
            auto timeT = std::chrono::system_clock::to_time_t(now);
            std::tm tm{};
#ifdef _WIN32
            localtime_s(&tm, &timeT);
#else
            localtime_r(&timeT, &tm);
#endif
            char buf[64];
            std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm);
            return std::string(buf);
        }
    }

    bool IsActive()
    {
        return s_active.load(std::memory_order_relaxed);
    }

    void SetActive(bool active)
    {
        if (active && !settings::GetBool(settings::Id::PhotoMode))
            return;

        if (s_active.exchange(active, std::memory_order_relaxed) == active)
            return;

        if (active)
        {
            s_needsCapture.store(true, std::memory_order_release);
            s_keyForward = s_keyBackward = s_keyLeft = s_keyRight = false;
            s_keyUp = s_keyDown = s_keyTurbo = s_keySlow = false;
            s_keyRollLeft = s_keyRollRight = false;
            s_mouseLooking = false;
            s_stickMoveX = s_stickMoveY = 0.0f;
            s_stickLookX = s_stickLookY = 0.0f;
            s_triggerUp = s_triggerDown = 0.0f;
            ui::Toast("Modo Foto activado: camara libre lista");
        }
        else
        {
            ui::Toast("Modo Foto desactivado");
        }
    }

    void Toggle()
    {
        SetActive(!IsActive());
    }

    bool ShouldFreezeWorld()
    {
        return IsActive() && s_freezeWorld.load(std::memory_order_relaxed);
    }

    bool ShouldHideHud()
    {
        return IsActive() && s_hideHud.load(std::memory_order_relaxed);
    }

    bool ShouldShowOverlay()
    {
        return IsActive() && s_showOverlay.load(std::memory_order_relaxed);
    }

    bool NeedsCaptureInitialPose()
    {
        return s_needsCapture.load(std::memory_order_acquire);
    }

    void CaptureInitialPose(uint8_t* base, uint32_t cameraAddress)
    {
        if (!cameraAddress)
            return;

        float m[16];
        for (int i = 0; i < 16; i++)
        {
            uint32_t bits = PPC_LOAD_U32(cameraAddress + i * 4);
            memcpy(&m[i], &bits, sizeof(bits));
        }

        s_pos = hfr::blend::ViewPosition(m);

        // In EAGL / NFSMW view matrix, columns 0, 1, 2 are Right, Up, Forward
        const hfr::blend::V3 R = { m[0], m[4], m[8] };
        const hfr::blend::V3 U = { m[1], m[5], m[9] };
        const hfr::blend::V3 F = { m[2], m[6], m[10] };

        float flen = std::sqrt(F.x * F.x + F.y * F.y + F.z * F.z);
        if (flen > 1e-6f)
        {
            const float fx = F.x / flen;
            const float fy = F.y / flen;
            const float fz = F.z / flen;
            // NFSMW coordinates: +X right, +Y up, +Z forward
            s_pitch = std::asin(std::clamp(fy, -0.999f, 0.999f));
            s_yaw = std::atan2(fx, fz);
        }
        else
        {
            s_pitch = 0.0f;
            s_yaw = 0.0f;
        }

        // Start level (roll = 0) so the camera is never tilted on activation
        s_roll = 0.0f;

        const uint16_t fov = PPC_LOAD_U16(cameraAddress + 196);
        if (fov >= 1000 && fov <= 32000)
            s_fov = float(fov);
        else
            s_fov = 10923.0f;

        fprintf(stderr, "[photo] Camara capturada: pos=(%.2f, %.2f, %.2f) yaw=%.1f deg pitch=%.1f deg fov=%.1f\n",
                s_pos.x, s_pos.y, s_pos.z, s_yaw * 57.29578f, s_pitch * 57.29578f, s_fov * 360.0f / 65536.0f);

        s_needsCapture.store(false, std::memory_order_release);
    }

    void ApplyToCamera(uint8_t* base, uint32_t cameraAddress)
    {
        if (!cameraAddress)
            return;

        const float cp = std::cos(s_pitch), sp = std::sin(s_pitch);
        const float cy = std::cos(s_yaw), sy = std::sin(s_yaw);
        const float cr = std::cos(s_roll), sr = std::sin(s_roll);

        // Forward vector in NFSMW (+X right, +Y up, +Z forward)
        const hfr::blend::V3 F = { sy * cp, sp, cy * cp };
        // Horizontal right vector (at roll = 0)
        const hfr::blend::V3 R0 = { cy, 0.0f, -sy };
        // Up vector (at roll = 0) = F x R0
        const hfr::blend::V3 U0 = { -sy * sp, cp, -cy * sp };

        // Apply roll around F
        const hfr::blend::V3 R = { R0.x * cr + U0.x * sr, R0.y * cr + U0.y * sr, R0.z * cr + U0.z * sr };
        const hfr::blend::V3 U = { -R0.x * sr + U0.x * cr, -R0.y * sr + U0.y * cr, -R0.z * sr + U0.z * cr };

        float m[16];
        // In EAGL / NFSMW view matrix: R_cam = transpose(R_world)
        // Column 0: Right (R)
        m[0] = R.x;  m[4] = R.y;  m[8]  = R.z;
        // Column 1: Up (U)
        m[1] = U.x;  m[5] = U.y;  m[9]  = U.z;
        // Column 2: Forward (F)
        m[2] = F.x;  m[6] = F.y;  m[10] = F.z;
        // Column 3: 0
        m[3] = 0.0f; m[7] = 0.0f; m[11] = 0.0f;

        // Row 3: Translation = -(pos . Axis)
        m[12] = -(s_pos.x * R.x + s_pos.y * R.y + s_pos.z * R.z);
        m[13] = -(s_pos.x * U.x + s_pos.y * U.y + s_pos.z * U.z);
        m[14] = -(s_pos.x * F.x + s_pos.y * F.y + s_pos.z * F.z);
        m[15] = 1.0f;

        for (int i = 0; i < 16; i++)
        {
            uint32_t bits;
            memcpy(&bits, &m[i], sizeof(bits));
            PPC_STORE_U32(cameraAddress + i * 4, bits);
        }

        const uint16_t fovU16 = uint16_t(std::clamp(std::round(s_fov), 1000.0f, 32000.0f));
        PPC_STORE_U16(cameraAddress + 196, fovU16);
    }

    void Update(double dtSeconds)
    {
        if (!IsActive())
            return;

        const float dt = float(std::clamp(dtSeconds, 0.001, 0.1));

        float speed = 15.0f;
        if (s_keyTurbo)
            speed *= 4.0f;
        if (s_keySlow)
            speed *= 0.2f;

        float moveFwd = 0.0f;
        float moveRight = 0.0f;
        float moveUp = 0.0f;

        if (s_keyForward)
            moveFwd += 1.0f;
        if (s_keyBackward)
            moveFwd -= 1.0f;
        if (s_keyRight)
            moveRight += 1.0f;
        if (s_keyLeft)
            moveRight -= 1.0f;
        if (s_keyUp)
            moveUp += 1.0f;
        if (s_keyDown)
            moveUp -= 1.0f;

        // Controller sampling: direct per-frame polling matching hid::Update()
        if (SDL_Gamepad* pad = hid::GetGamepad())
        {
            auto axis = [&](SDL_GamepadAxis a) {
                return float(SDL_GetGamepadAxis(pad, a)) / 32767.0f;
            };

            const float lx = axis(SDL_GAMEPAD_AXIS_LEFTX);
            const float ly = axis(SDL_GAMEPAD_AXIS_LEFTY);
            const float rx = axis(SDL_GAMEPAD_AXIS_RIGHTX);
            const float ry = axis(SDL_GAMEPAD_AXIS_RIGHTY);
            const float lt = std::clamp(axis(SDL_GAMEPAD_AXIS_LEFT_TRIGGER), 0.0f, 1.0f);
            const float rt = std::clamp(axis(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), 0.0f, 1.0f);

            s_dbgLx = lx; s_dbgLy = ly;
            s_dbgRx = rx; s_dbgRy = ry;
            s_dbgLt = lt; s_dbgRt = rt;

            // Left stick: lateral & forward flight
            if (std::abs(lx) > 0.08f)
                moveRight += (lx > 0.0f ? (lx - 0.08f) : (lx + 0.08f)) / 0.92f;
            if (std::abs(ly) > 0.08f)
                moveFwd -= (ly > 0.0f ? (ly - 0.08f) : (ly + 0.08f)) / 0.92f;

            // Triggers: elevation (RT ascends, LT descends)
            if (rt > 0.04f)
                moveUp += (rt - 0.04f) / 0.96f;
            if (lt > 0.04f)
                moveUp -= (lt - 0.04f) / 0.96f;

            // Shoulder buttons: speed modifiers
            if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER))
                speed *= 4.0f;
            if (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER))
                speed *= 0.2f;

            // Right stick: camera look rotation
            if (std::abs(rx) > 0.08f)
            {
                float normRx = (rx > 0.0f ? (rx - 0.08f) : (rx + 0.08f)) / 0.92f;
                s_yaw += (s_invertLookX ? -1.0f : 1.0f) * normRx * 2.2f * dt;
            }
            if (std::abs(ry) > 0.08f)
            {
                float normRy = (ry > 0.0f ? (ry - 0.08f) : (ry + 0.08f)) / 0.92f;
                // SDL stick UP is ry < 0. Pushing stick UP tilts camera UP (increasing pitch).
                float pitchDelta = (s_invertLookY ? 1.0f : -1.0f) * normRy * 1.8f * dt;
                s_pitch += pitchDelta;
                s_pitch = std::clamp(s_pitch, -1.55f, 1.55f);
            }
        }
        else
        {
            // Fallback for events if gamepad polling is not ready
            moveFwd += -s_stickMoveY;
            moveRight += s_stickMoveX;
            moveUp += (s_triggerUp - s_triggerDown);

            if (std::abs(s_stickLookX) > 0.05f)
                s_yaw += (s_invertLookX ? -1.0f : 1.0f) * s_stickLookX * 2.2f * dt;
            if (std::abs(s_stickLookY) > 0.05f)
            {
                s_pitch += (s_invertLookY ? 1.0f : -1.0f) * s_stickLookY * 1.8f * dt;
                s_pitch = std::clamp(s_pitch, -1.55f, 1.55f);
            }
        }

        // Roll
        if (s_keyRollLeft)
            s_roll -= 1.5f * dt;
        if (s_keyRollRight)
            s_roll += 1.5f * dt;

        // Position displacement
        const float cp = std::cos(s_pitch), sp = std::sin(s_pitch);
        const float cy = std::cos(s_yaw), sy = std::sin(s_yaw);
        const float cr = std::cos(s_roll), sr = std::sin(s_roll);

        const hfr::blend::V3 F = { sy * cp, sp, cy * cp };
        const hfr::blend::V3 R0 = { cy, 0.0f, -sy };
        const hfr::blend::V3 U0 = { -sy * sp, cp, -cy * sp };
        const hfr::blend::V3 R = { R0.x * cr + U0.x * sr, R0.y * cr + U0.y * sr, R0.z * cr + U0.z * sr };

        // Horizontal flight along F and R; vertical elevation along pure world +Y
        s_pos.x += (F.x * moveFwd + R.x * moveRight) * speed * dt;
        s_pos.y += (F.y * moveFwd + R.y * moveRight + moveUp) * speed * dt;
        s_pos.z += (F.z * moveFwd + R.z * moveRight) * speed * dt;
    }

    bool HandleEvent(const SDL_Event& e)
    {
        // Toggle chord detection on controller
        if (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN || e.type == SDL_EVENT_GAMEPAD_BUTTON_UP)
        {
            const bool down = (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN);
            if (e.gbutton.button == SDL_GAMEPAD_BUTTON_BACK)
                s_padBack = down;
            else if (e.gbutton.button == SDL_GAMEPAD_BUTTON_RIGHT_STICK)
                s_padR3 = down;

            if (s_padBack && s_padR3 && down)
            {
                Toggle();
                return true;
            }
        }

        // Keyboard toggle
        if (e.type == SDL_EVENT_KEY_DOWN && !e.key.repeat)
        {
            if (e.key.key == SDLK_P)
            {
                Toggle();
                return true;
            }
        }

        if (!IsActive())
            return false;

        // Inside Photo Mode: handle flight and features
        switch (e.type)
        {
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP:
        {
            const bool down = (e.type == SDL_EVENT_KEY_DOWN);
            switch (e.key.key)
            {
            case SDLK_W:
                s_keyForward = down;
                return true;
            case SDLK_S:
                s_keyBackward = down;
                return true;
            case SDLK_A:
                s_keyLeft = down;
                return true;
            case SDLK_D:
                s_keyRight = down;
                return true;
            case SDLK_SPACE:
                s_keyUp = down;
                return true;
            case SDLK_C:
                s_keyDown = down;
                return true;
            case SDLK_LSHIFT:
            case SDLK_RSHIFT:
                s_keyTurbo = down;
                return true;
            case SDLK_LALT:
            case SDLK_RALT:
                s_keySlow = down;
                return true;
            case SDLK_Q:
                s_keyRollLeft = down;
                return true;
            case SDLK_E:
                s_keyRollRight = down;
                return true;
            case SDLK_R:
                if (down)
                    s_roll = 0.0f;
                return true;
            case SDLK_I:
                if (down)
                {
                    bool newVal = !s_invertLookY.load(std::memory_order_relaxed);
                    s_invertLookY.store(newVal, std::memory_order_relaxed);
                    ui::Toast(newVal ? "Invertir Mirada Y: Activado" : "Invertir Mirada Y: Desactivado");
                }
                return true;
            case SDLK_H:
                if (down)
                    s_showOverlay.store(!s_showOverlay.load(std::memory_order_relaxed), std::memory_order_relaxed);
                return true;
            case SDLK_F12:
            case SDLK_RETURN:
                if (down)
                    CapturePhoto();
                return true;
            case SDLK_ESCAPE:
                if (down)
                    SetActive(false);
                return true;
            case SDLK_LEFTBRACKET:
                if (down)
                    s_fov = std::min(s_fov + 400.0f, 28000.0f);
                return true;
            case SDLK_RIGHTBRACKET:
                if (down)
                    s_fov = std::max(s_fov - 400.0f, 2500.0f);
                return true;
            default:
                break;
            }
            break;
        }

        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
        {
            if (e.button.button == SDL_BUTTON_RIGHT || e.button.button == SDL_BUTTON_LEFT)
            {
                s_mouseLooking = (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN);
                return true;
            }
            break;
        }

        case SDL_EVENT_MOUSE_MOTION:
        {
            if (s_mouseLooking)
            {
                s_yaw += (s_invertLookX ? -1.0f : 1.0f) * e.motion.xrel * 0.003f;
                s_pitch -= (s_invertLookY ? -1.0f : 1.0f) * e.motion.yrel * 0.003f;
                s_pitch = std::clamp(s_pitch, -1.55f, 1.55f);
                return true;
            }
            break;
        }

        case SDL_EVENT_MOUSE_WHEEL:
        {
            if (e.wheel.y != 0.0f)
            {
                s_fov -= e.wheel.y * 350.0f;
                s_fov = std::clamp(s_fov, 2500.0f, 28000.0f);
                return true;
            }
            break;
        }

        case SDL_EVENT_GAMEPAD_AXIS_MOTION:
        {
            const float val = float(e.gaxis.value) / 32767.0f;
            switch (e.gaxis.axis)
            {
            case SDL_GAMEPAD_AXIS_LEFTX:
                s_stickMoveX = (std::abs(val) > 0.08f) ? val : 0.0f;
                return true;
            case SDL_GAMEPAD_AXIS_LEFTY:
                s_stickMoveY = (std::abs(val) > 0.08f) ? val : 0.0f;
                return true;
            case SDL_GAMEPAD_AXIS_RIGHTX:
                s_stickLookX = (std::abs(val) > 0.08f) ? val : 0.0f;
                return true;
            case SDL_GAMEPAD_AXIS_RIGHTY:
                s_stickLookY = (std::abs(val) > 0.08f) ? val : 0.0f;
                return true;
            case SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
                s_triggerUp = (val > 0.05f) ? val : 0.0f;
                return true;
            case SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
                s_triggerDown = (val > 0.05f) ? val : 0.0f;
                return true;
            default:
                break;
            }
            break;
        }

        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
        {
            switch (e.gbutton.button)
            {
            case SDL_GAMEPAD_BUTTON_WEST: // X button
                CapturePhoto();
                return true;
            case SDL_GAMEPAD_BUTTON_NORTH: // Y button
                s_showOverlay.store(!s_showOverlay.load(std::memory_order_relaxed), std::memory_order_relaxed);
                return true;
            case SDL_GAMEPAD_BUTTON_EAST: // B button
                SetActive(false);
                return true;
            case SDL_GAMEPAD_BUTTON_RIGHT_STICK:
                s_roll = 0.0f;
                return true;
            case SDL_GAMEPAD_BUTTON_DPAD_UP:
                s_fov = std::max(s_fov - 600.0f, 2500.0f);
                return true;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN:
                s_fov = std::min(s_fov + 600.0f, 28000.0f);
                return true;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT:
                s_roll -= 0.1f;
                return true;
            case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:
                s_roll += 0.1f;
                return true;
            case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER:
                s_keyTurbo = true;
                return true;
            case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:
                s_keySlow = true;
                return true;
            default:
                break;
            }
            break;
        }

        case SDL_EVENT_GAMEPAD_BUTTON_UP:
        {
            if (e.gbutton.button == SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER)
                s_keyTurbo = false;
            else if (e.gbutton.button == SDL_GAMEPAD_BUTTON_LEFT_SHOULDER)
                s_keySlow = false;
            return true;
        }

        default:
            break;
        }

        return false;
    }

    void CapturePhoto()
    {
        std::future<video::CapturedFrame> futureFrame = video::CaptureFrame();

        std::thread([f = std::move(futureFrame)]() mutable {
            if (!f.valid())
                return;

            if (f.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
            {
                ui::Toast("Error capturando foto (tiempo de espera agotado)");
                return;
            }

            video::CapturedFrame frame = f.get();
            if (frame.rgb.empty() || frame.width == 0 || frame.height == 0)
            {
                ui::Toast("Error: cuadro vacio");
                return;
            }

            std::filesystem::path dir = GetScreenshotsDirectory();
            if (dir.empty())
            {
                ui::Toast("Error: no se encontro la carpeta de imagenes");
                return;
            }

            std::string filename = std::format("SpeedBreaker_{}.png", FormatTimestamp());
            std::filesystem::path fullPath = dir / filename;

            std::vector<uint8_t> png = report::EncodePng(frame.rgb.data(), frame.width, frame.height);
            if (png.empty())
            {
                ui::Toast("Error codificando PNG");
                return;
            }

            std::ofstream out(fullPath, std::ios::binary);
            if (!out)
            {
                ui::Toast("Error guardando archivo en disco");
                return;
            }

            out.write(reinterpret_cast<const char*>(png.data()), png.size());
            out.close();

            ui::Toast(std::format("Foto guardada: Pictures/SpeedBreaker/{}", filename));
            fprintf(stderr, "[photo] %ux%u guardada exitosamente en %s\n", frame.width, frame.height, fullPath.string().c_str());
        }).detach();
    }

    void DrawOverlay()
    {
        if (!ShouldShowOverlay())
            return;

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 20.0f, vp->WorkPos.y + vp->WorkSize.y - 20.0f), ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.70f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                 ImGuiWindowFlags_NoNav;

        if (ImGui::Begin("##PhotoModeOverlay", nullptr, flags))
        {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.15f, 1.0f), "MODO FOTO");
            ImGui::SameLine();
            ImGui::TextDisabled("| FOV: %.1f deg  Inclinacion: %.1f deg", s_fov * 360.0f / 65536.0f, s_roll * 180.0f / 3.14159f);

            bool invY = s_invertLookY.load(std::memory_order_relaxed);
            ImGui::SameLine();
            if (ImGui::Checkbox("Invertir Y (Tecla I)", &invY))
                s_invertLookY.store(invY, std::memory_order_relaxed);

            bool invX = s_invertLookX.load(std::memory_order_relaxed);
            ImGui::SameLine();
            if (ImGui::Checkbox("Invertir X", &invX))
                s_invertLookX.store(invX, std::memory_order_relaxed);

            ImGui::Separator();
            ImGui::Text("WASD / Stick Izq: Mover  |  Click Derecho / Stick Der: Mirar");
            ImGui::Text("Espacio / RT: Subir  |  C / LT: Bajar  |  Shift / RB: Turbo  |  Alt / LB: Precision");
            ImGui::Text("Q/E / D-pad: Rotar  |  R / R3: Restablecer  |  F12 / X: Tomar Foto  |  H / Y: Ayuda  |  P / B: Salir");

            if (hid::GetGamepad() != nullptr)
            {
                ImGui::Separator();
                ImGui::TextDisabled("Mando: LT: %.2f | RT: %.2f | Stick L: (%.2f, %.2f) | Stick R: (%.2f, %.2f)",
                                    s_dbgLt, s_dbgRt, s_dbgLx, s_dbgLy, s_dbgRx, s_dbgRy);
            }
        }
        ImGui::End();
    }
}
