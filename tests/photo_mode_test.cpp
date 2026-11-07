// SpeedBreaker tests. GPL-3.0-or-later (see COPYING).
//
// Unit test for photo mode camera math, coordinate spaces, and view matrix decomposition.
// Build by hand: c++ -std=c++20 -O2 -I runtime tests/photo_mode_test.cpp
#include <game/hfr_blend.h>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <algorithm>

namespace
{
    int s_failures = 0;

#define CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            s_failures++; \
            fprintf(stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
        } \
    } while (0)

    bool Near(float a, float b, float eps = 1e-3f)
    {
        return std::fabs(a - b) <= eps;
    }

    void BuildViewMatrix(hfr::blend::V3 pos, float yaw, float pitch, float roll, float out[16])
    {
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        const float cr = std::cos(roll), sr = std::sin(roll);

        // World-space orientation axes of the camera in NFSMW (+X right, +Y up, +Z forward)
        const hfr::blend::V3 F = { sy * cp, sp, cy * cp };
        const hfr::blend::V3 R0 = { cy, 0.0f, -sy };
        const hfr::blend::V3 U0 = { -sy * sp, cp, -cy * sp };

        const hfr::blend::V3 R = { R0.x * cr + U0.x * sr, R0.y * cr + U0.y * sr, R0.z * cr + U0.z * sr };
        const hfr::blend::V3 U = { -R0.x * sr + U0.x * cr, -R0.y * sr + U0.y * cr, -R0.z * sr + U0.z * cr };

        // In EAGL / NFSMW view matrix: R_cam = transpose(R_world)
        // Column 0: Right (R)
        out[0] = R.x;  out[4] = R.y;  out[8]  = R.z;
        // Column 1: Up (U)
        out[1] = U.x;  out[5] = U.y;  out[9]  = U.z;
        // Column 2: Forward (F)
        out[2] = F.x;  out[6] = F.y;  out[10] = F.z;
        // Column 3: 0
        out[3] = 0.0f; out[7] = 0.0f; out[11] = 0.0f;

        // Row 3: Translation = -(pos . Axis)
        out[12] = -(pos.x * R.x + pos.y * R.y + pos.z * R.z);
        out[13] = -(pos.x * U.x + pos.y * U.y + pos.z * U.z);
        out[14] = -(pos.x * F.x + pos.y * F.y + pos.z * F.z);
        out[15] = 1.0f;
    }

    void TestOrthonormality()
    {
        float m[16];
        BuildViewMatrix({ 123.4f, -456.7f, 89.0f }, 0.75f, -0.3f, 0.2f, m);

        hfr::blend::V3 colR = { m[0], m[4], m[8] };
        hfr::blend::V3 colU = { m[1], m[5], m[9] };
        hfr::blend::V3 colF = { m[2], m[6], m[10] };

        CHECK(Near(hfr::blend::Len(colR), 1.0f), "Column 0 (Right) length must be 1.0");
        CHECK(Near(hfr::blend::Len(colU), 1.0f), "Column 1 (Up) length must be 1.0");
        CHECK(Near(hfr::blend::Len(colF), 1.0f), "Column 2 (Forward) length must be 1.0");

        CHECK(Near(hfr::blend::Dot(colR, colU), 0.0f), "R and U must be orthogonal");
        CHECK(Near(hfr::blend::Dot(colR, colF), 0.0f), "R and F must be orthogonal");
        CHECK(Near(hfr::blend::Dot(colU, colF), 0.0f), "U and F must be orthogonal");
    }

    void TestViewPositionInversion()
    {
        const hfr::blend::V3 expectedPos = { 542.1f, -1208.3f, 45.6f };
        float m[16];
        BuildViewMatrix(expectedPos, 1.25f, 0.45f, -0.15f, m);

        hfr::blend::V3 computedPos = hfr::blend::ViewPosition(m);

        CHECK(Near(computedPos.x, expectedPos.x, 0.05f), "Inverted X position must match");
        CHECK(Near(computedPos.y, expectedPos.y, 0.05f), "Inverted Y position must match");
        CHECK(Near(computedPos.z, expectedPos.z, 0.05f), "Inverted Z position must match");
    }

    void TestAngleDecomposition()
    {
        const float expectedYaw = 0.85f;
        const float expectedPitch = -0.28f;
        const float expectedRoll = 0.15f;
        float m[16];
        BuildViewMatrix({ 0.0f, 0.0f, 0.0f }, expectedYaw, expectedPitch, expectedRoll, m);

        // Columns are Right, Up, Forward
        hfr::blend::V3 R = { m[0], m[4], m[8] };
        hfr::blend::V3 U = { m[1], m[5], m[9] };
        hfr::blend::V3 F = { m[2], m[6], m[10] };

        float flen = std::sqrt(F.x * F.x + F.y * F.y + F.z * F.z);
        F.x /= flen; F.y /= flen; F.z /= flen;

        float computedPitch = std::asin(std::clamp(F.y, -0.999f, 0.999f));
        float computedYaw = std::atan2(F.x, F.z);

        const float cp = std::cos(computedPitch), sp = std::sin(computedPitch);
        const float cy = std::cos(computedYaw), sy = std::sin(computedYaw);
        const hfr::blend::V3 R0 = { cy, 0.0f, -sy };
        const hfr::blend::V3 U0 = { -sy * sp, cp, -cy * sp };

        const float dotR0 = R.x * R0.x + R.y * R0.y + R.z * R0.z;
        const float dotU0 = R.x * U0.x + R.y * U0.y + R.z * U0.z;
        float computedRoll = std::atan2(dotU0, dotR0);

        CHECK(Near(computedPitch, expectedPitch, 1e-4f), "Decomposed pitch must match");
        CHECK(Near(computedYaw, expectedYaw, 1e-4f), "Decomposed yaw must match");
        CHECK(Near(computedRoll, expectedRoll, 1e-4f), "Decomposed roll must match");
    }

    void TestElevationAndOrientation()
    {
        // Test that shifting position along -U produces pure vertical displacement
        // in camera space with zero lateral (R) or forward (F) displacement.
        const float yaw = 0.6f;
        const float pitch = -0.3f;
        const float roll = 0.25f;

        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        const float cr = std::cos(roll), sr = std::sin(roll);

        const hfr::blend::V3 R0 = { cy, 0.0f, -sy };
        const hfr::blend::V3 U0 = { -sy * sp, cp, -cy * sp };
        const hfr::blend::V3 R = { R0.x * cr + U0.x * sr, R0.y * cr + U0.y * sr, R0.z * cr + U0.z * sr };
        const hfr::blend::V3 U = { -R0.x * sr + U0.x * cr, -R0.y * sr + U0.y * cr, -R0.z * sr + U0.z * cr };

        hfr::blend::V3 pos0 = { 100.0f, 20.0f, -50.0f };
        float deltaElevation = 5.0f; // e.g. RT pressed

        // Elevation displacement along -U
        hfr::blend::V3 pos1 = {
            pos0.x - U.x * deltaElevation,
            pos0.y - U.y * deltaElevation,
            pos0.z - U.z * deltaElevation
        };

        float m0[16], m1[16];
        BuildViewMatrix(pos0, yaw, pitch, roll, m0);
        BuildViewMatrix(pos1, yaw, pitch, roll, m1);

        // In camera space: delta translation = m1[12..14] - m0[12..14]
        float deltaCamX = m1[12] - m0[12]; // Along Right (lateral)
        float deltaCamY = m1[13] - m0[13]; // Along Up (vertical)
        float deltaCamZ = m1[14] - m0[14]; // Along Forward (depth)

        CHECK(Near(deltaCamX, 0.0f, 1e-4f), "Elevation along -U must produce zero lateral shift in camera space");
        CHECK(Near(deltaCamZ, 0.0f, 1e-4f), "Elevation along -U must produce zero forward/backward shift");
        CHECK(Near(deltaCamY, deltaElevation, 1e-4f), "Elevation along -U must produce positive vertical shift in camera space");
    }
}

int main()
{
    TestOrthonormality();
    TestViewPositionInversion();
    TestAngleDecomposition();
    TestElevationAndOrientation();

    if (s_failures == 0)
    {
        printf("PASS: all photo mode camera math tests succeeded!\n");
        return 0;
    }
    else
    {
        printf("FAILED: %d tests failed\n", s_failures);
        return 1;
    }
}
