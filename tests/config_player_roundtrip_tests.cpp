// A PLAYER build reads every key a player build writes.
//
// The settings file is written from the whole of the settings, and read by a chain of key tests of which a part
// is compiled into development builds only. A key that is written always and read inside that part is saved,
// then called unknown at the next start: the setting works for the session it is changed in and is forgotten.
// This target is compiled as a player build (no HALOMCCVR_DEV_DIAGNOSTICS), so that part is not in it.
#include <windows.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include "../src/common/config.h"

namespace
{
std::vector<std::string> g_unknownKeys;
int failures = 0;
void Check(bool ok, const char* what)
{
    if (!ok)
    {
        ++failures;
        std::printf("FAIL: %s\n", what);
    }
}
} // namespace

void Logf(const char* format, ...)
{
    char line[512];
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(line, sizeof(line), format, arguments);
    va_end(arguments);
    if (std::strstr(line, "unknown key"))
        g_unknownKeys.emplace_back(line);
}

int main()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    std::printf("FAIL: this target must be compiled as a player build\n");
    return 1;
#else
    wchar_t folder[MAX_PATH]{}, path[MAX_PATH]{};
    GetTempPathW(MAX_PATH, folder);
    swprintf_s(path, L"%shalomccvr-player-roundtrip-%lu.cfg", folder, GetCurrentProcessId());
    DeleteFileW(path);

    // The first load of a file that is not there leaves the built-in settings; the save writes all of them.
    g_config = Config{};
    ConfigLoad(path);
    g_config.mono_rendering = true;
    ConfigSave();

    g_unknownKeys.clear();
    g_config = Config{};
    ConfigLoad(path);
    DeleteFileW(path);

    for (const std::string& line : g_unknownKeys)
        std::printf("FAIL: written by a player build and not read by one: %s\n", line.c_str());
    failures += static_cast<int>(g_unknownKeys.size());
    Check(g_config.mono_rendering, "mono_rendering is remembered");

    // Upgrade once, including explicit title overrides; later edits survive reload.
    FILE* legacy=nullptr;
    _wfopen_s(&legacy,path,L"wt");
    Check(legacy!=nullptr,"legacy fixture opens");
    if (legacy) {
        std::fputs("left_hand_forward_m = 0.12\nhalo3_left_hand_forward_m = 0.2\n",legacy);
        std::fclose(legacy);
        ConfigLoad(path);
        Check(g_config.left_hand_forward_m==0,"legacy shared offset resets");
        for (const auto& profile:g_config.title_profiles)
            Check(profile.left_hand_forward_m==0,"legacy title offset resets");
        g_config.left_hand_forward_m=0.075f;
        ConfigSave();
        ConfigLoad(path);
        Check(g_config.left_hand_forward_m==0.075f,"post-upgrade edit survives reload");
        DeleteFileW(path);
    }

    if (failures)
        std::printf("%d check(s) failed\n", failures);
    else
        std::printf("config player round trip: all checks passed\n");
    return failures ? 1 : 0;
#endif
}
