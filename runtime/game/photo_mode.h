// SpeedBreaker runtime. GPL-3.0-or-later (see COPYING).
#pragma once
#include <cstdint>
#include <SDL3/SDL_events.h>

namespace game::photo_mode
{
    // Whether Photo Mode is active.
    bool IsActive();

    // Toggle or explicitly set Photo Mode.
    void Toggle();
    void SetActive(bool active);

    // Engine freeze and UI suppression queries.
    bool ShouldFreezeWorld();
    bool ShouldHideHud();
    bool ShouldShowOverlay();

    // Event handling (keyboard, mouse, gamepad).
    // Returns true if the event was consumed by Photo Mode.
    bool HandleEvent(const SDL_Event& e);

    // Updates camera kinematics and flight physics every frame.
    void Update(double dtSeconds);

    // Integration with sub_8243EAC0 (view setup).
    bool NeedsCaptureInitialPose();
    void CaptureInitialPose(uint8_t* base, uint32_t cameraAddress);
    void ApplyToCamera(uint8_t* base, uint32_t cameraAddress);

    // ImGui on-screen overlay.
    void DrawOverlay();

    // Capture photo / screenshot to ~/Pictures/SpeedBreaker/.
    void CapturePhoto();
}
