//============ Copyright (c) Valve Corporation. All rights reserved. ============
//========================== FlatVR Universal Driver ============================

#include <openvr_driver.h>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <cmath>

#include <atomic>
#include <windows.h>
#include "IniReader\IniReader.h"

using namespace vr;
using namespace std::chrono;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w; quat.x = x; quat.y = y; quat.z = z;
	return quat;
}

inline vr::HmdQuaternion_t EulerAngleToQuaternion(double Yaw, double Pitch, double Roll)
{
	vr::HmdQuaternion_t q;
	double cy = cos(Yaw * 0.5); double sy = sin(Yaw * 0.5); double cp = cos(Pitch * 0.5);
	double sp = sin(Pitch * 0.5); double cr = cos(Roll * 0.5); double sr = sin(Roll * 0.5);

	q.w = cy * cp * cr + sy * sp * sr;
	q.x = cy * sp * cr + sy * cp * sr;
	q.y = sy * cp * cr - cy * sp * sr;
	q.z = cy * cp * sr - sy * sp * cr;

	return q;
}

// Rotation matrix (column vectors: right, up, back) -> quaternion. Shepperd's method.
inline vr::HmdQuaternion_t MatrixToQuaternion(const double m[3][3])
{
	vr::HmdQuaternion_t q;
	double t = m[0][0] + m[1][1] + m[2][2];
	if (t > 0.0) {
		double s = sqrt(t + 1.0) * 2.0;
		q.w = 0.25 * s; q.x = (m[2][1] - m[1][2]) / s; q.y = (m[0][2] - m[2][0]) / s; q.z = (m[1][0] - m[0][1]) / s;
	} else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
		double s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
		q.w = (m[2][1] - m[1][2]) / s; q.x = 0.25 * s; q.y = (m[0][1] + m[1][0]) / s; q.z = (m[0][2] + m[2][0]) / s;
	} else if (m[1][1] > m[2][2]) {
		double s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
		q.w = (m[0][2] - m[2][0]) / s; q.x = (m[0][1] + m[1][0]) / s; q.y = 0.25 * s; q.z = (m[1][2] + m[2][1]) / s;
	} else {
		double s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
		q.w = (m[1][0] - m[0][1]) / s; q.x = (m[0][2] + m[2][0]) / s; q.y = (m[1][2] + m[2][1]) / s; q.z = 0.25 * s;
	}
	return q;
}

// --- FORWARD DECLARATIONS & GLOBAL STATES ---
void LoadConfig();
std::string g_ActiveProfileName = "Default";
std::string g_DriverFolderStr = "";
FILETIME lastIniMod = { 0, 0 };

// --- PATH RESOLUTION ---
std::string GetDriverFolder() {
    char path[MAX_PATH] = { 0 };
    HMODULE hMod = NULL;
    if (!GetModuleHandleExA(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        (LPCSTR)reinterpret_cast<void*>(GetDriverFolder), &hMod))
        return "";
    if (!GetModuleFileNameA(hMod, path, MAX_PATH))
        return "";

    std::string dir(path);
    size_t pos = dir.find_last_of("\\/");
    if (pos == std::string::npos) return "";
    dir = dir.substr(0, pos);

    if (GetFileAttributesA((dir + "/FlatVR.ini").c_str()) != INVALID_FILE_ATTRIBUTES)
        return dir;

    std::string parent = dir;
    for (int i = 0; i < 2; ++i) {
        size_t p = parent.find_last_of("\\/");
        if (p == std::string::npos) break;
        parent = parent.substr(0, p);
    }
    if (GetFileAttributesA((parent + "/FlatVR.ini").c_str()) != INVALID_FILE_ATTRIBUTES)
        return parent;

    return parent;
}

// --- KEY MAPPING ---
int KeyNameToKeyCode(std::string KeyName) {
	std::transform(KeyName.begin(), KeyName.end(), KeyName.begin(), ::toupper);
	if (KeyName == "NONE") return 0;
	else if (KeyName == "MOUSE-LEFT-BTN") return VK_LBUTTON;
	else if (KeyName == "MOUSE-RIGHT-BTN") return VK_RBUTTON;
	else if (KeyName == "MOUSE-MIDDLE-BTN") return VK_MBUTTON;
	else if (KeyName == "MOUSE-SIDE1-BTN") return VK_XBUTTON1;
	else if (KeyName == "MOUSE-SIDE2-BTN") return VK_XBUTTON2;
	else if (KeyName == "ESCAPE") return VK_ESCAPE;
	else if (KeyName == "F1") return VK_F1; else if (KeyName == "F2") return VK_F2;
	else if (KeyName == "F3") return VK_F3; else if (KeyName == "F4") return VK_F4;
	else if (KeyName == "F5") return VK_F5; else if (KeyName == "F6") return VK_F6;
	else if (KeyName == "F7") return VK_F7; else if (KeyName == "F8") return VK_F8;
	else if (KeyName == "F9") return VK_F9; else if (KeyName == "F10") return VK_F10;
	else if (KeyName == "F11") return VK_F11; else if (KeyName == "F12") return VK_F12;
	else if (KeyName == "F13") return VK_F13; else if (KeyName == "F14") return VK_F14;
	else if (KeyName == "F15") return VK_F15; else if (KeyName == "F16") return VK_F16;
	else if (KeyName == "F17") return VK_F17; else if (KeyName == "F18") return VK_F18;
	else if (KeyName == "F19") return VK_F19; else if (KeyName == "F20") return VK_F20;
	else if (KeyName == "F21") return VK_F21; else if (KeyName == "F22") return VK_F22;
	else if (KeyName == "F23") return VK_F23; else if (KeyName == "F24") return VK_F24;
	else if (KeyName == "~") return 192;
	else if (KeyName == "1") return '1'; else if (KeyName == "2") return '2';
	else if (KeyName == "3") return '3'; else if (KeyName == "4") return '4';
	else if (KeyName == "5") return '5'; else if (KeyName == "6") return '6';
	else if (KeyName == "7") return '7'; else if (KeyName == "8") return '8';
	else if (KeyName == "9") return '9'; else if (KeyName == "0") return '0';
	else if (KeyName == "-") return 189; else if (KeyName == "=") return 187;
	else if (KeyName == "TAB") return VK_TAB; else if (KeyName == "CAPS-LOCK") return VK_CAPITAL;
	else if (KeyName == "SHIFT") return VK_SHIFT; else if (KeyName == "CTRL") return VK_CONTROL;
	else if (KeyName == "RCTRL") return VK_RCONTROL; else if (KeyName == "WIN") return VK_LWIN;
	else if (KeyName == "ALT") return VK_MENU; else if (KeyName == "SPACE") return VK_SPACE;
	else if (KeyName == "ENTER") return VK_RETURN; else if (KeyName == "BACKSPACE") return VK_BACK;
	else if (KeyName == "Q") return 'Q'; else if (KeyName == "W") return 'W';
	else if (KeyName == "E") return 'E'; else if (KeyName == "R") return 'R';
	else if (KeyName == "T") return 'T'; else if (KeyName == "Y") return 'Y';
	else if (KeyName == "U") return 'U'; else if (KeyName == "I") return 'I';
	else if (KeyName == "O") return 'O'; else if (KeyName == "P") return 'P';
	else if (KeyName == "[") return 219; else if (KeyName == "]") return 221;
	else if (KeyName == "LBRACKET" || KeyName == "LEFT-BRACKET")   return 219;
	else if (KeyName == "RBRACKET" || KeyName == "RIGHT-BRACKET")  return 221;
	else if (KeyName == "A") return 'A'; else if (KeyName == "S") return 'S';
	else if (KeyName == "D") return 'D'; else if (KeyName == "F") return 'F';
	else if (KeyName == "G") return 'G'; else if (KeyName == "H") return 'H';
	else if (KeyName == "J") return 'J'; else if (KeyName == "K") return 'K';
	else if (KeyName == "L") return 'L'; else if (KeyName == ":" || KeyName == "SEMICOLON" || KeyName == ";") return 186;
	else if (KeyName == "APOSTROPHE") return 222; else if (KeyName == "\\") return 220;
	else if (KeyName == "Z") return 'Z'; else if (KeyName == "X") return 'X';
	else if (KeyName == "C") return 'C'; else if (KeyName == "V") return 'V';
	else if (KeyName == "B") return 'B'; else if (KeyName == "N") return 'N';
	else if (KeyName == "M") return 'M'; else if (KeyName == "<") return 188;
	else if (KeyName == ">") return 190; else if (KeyName == "?") return 191;
	else if (KeyName == "PRINTSCREEN") return VK_SNAPSHOT; else if (KeyName == "SCROLL-LOCK") return VK_SCROLL;
	else if (KeyName == "PAUSE") return VK_PAUSE; else if (KeyName == "INSERT") return VK_INSERT;
	else if (KeyName == "HOME") return VK_HOME; else if (KeyName == "DELETE") return VK_DELETE;
	else if (KeyName == "END") return VK_END; else if (KeyName == "PAGE-UP") return VK_PRIOR;
	else if (KeyName == "PAGE-DOWN") return VK_NEXT; else if (KeyName == "UP") return VK_UP;
	else if (KeyName == "DOWN") return VK_DOWN; else if (KeyName == "LEFT") return VK_LEFT;
	else if (KeyName == "RIGHT") return VK_RIGHT; else if (KeyName == "NUM-LOCK") return VK_NUMLOCK;
	else if (KeyName == "NUMPAD0") return VK_NUMPAD0; else if (KeyName == "NUMPAD1") return VK_NUMPAD1;
	else if (KeyName == "NUMPAD2") return VK_NUMPAD2; else if (KeyName == "NUMPAD3") return VK_NUMPAD3;
	else if (KeyName == "NUMPAD4") return VK_NUMPAD4; else if (KeyName == "NUMPAD5") return VK_NUMPAD5;
	else if (KeyName == "NUMPAD6") return VK_NUMPAD6; else if (KeyName == "NUMPAD7") return VK_NUMPAD7;
	else if (KeyName == "NUMPAD8") return VK_NUMPAD8; else if (KeyName == "NUMPAD9") return VK_NUMPAD9;
	else if (KeyName == "NUMPAD-DIVIDE") return VK_DIVIDE; else if (KeyName == "NUMPAD-MULTIPLY") return VK_MULTIPLY;
	else if (KeyName == "NUMPAD-MINUS") return VK_SUBTRACT; else if (KeyName == "NUMPAD-PLUS") return VK_ADD;
	else if (KeyName == "NUMPAD-DEL") return VK_DECIMAL;
	else if (KeyName == "," || KeyName == "COMMA") return VK_OEM_COMMA;
	else if (KeyName == "." || KeyName == "PERIOD") return VK_OEM_PERIOD;
	else return 0;
}

// --- GLOBALS & STRUCTS ---
static const char* const k_pch_Sample_Section = "FlatVR";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";
static const char* const k_pch_Sample_ZoomWidth_Float = "ZoomWidth";
static const char* const k_pch_Sample_ZoomHeight_Float = "ZoomHeight";
static const char* const k_pch_Sample_FOV_Float = "FOV";
static const char* const k_pch_Sample_Stereo_Bool = "Stereo";
static const char* const k_pch_Sample_DebugMode_Bool = "DebugMode";

double FirstCtrlLastPos[3] = { 0, 0, 0 }, SecondCtrlLastPos[3] = { 0, 0, 0 };
double FirstCtrlVel[3] = { 0, 0, 0 }, SecondCtrlVel[3] = { 0, 0, 0 };
double LastHmdPos[3] = { 0, 0, 0 };
double LastHmdYaw = 0, LastHmdPitch = 0;
double HmdOmegaX = 0, HmdOmegaY = 0;

milliseconds deltaTime;
double g_DtSeconds = 1.0 / 90.0;

typedef struct _HMDData { double X, Y, Z, Yaw, Pitch, Roll; } THMD, * PHMD;

typedef struct _Controller
{
	double	X, Y, Z, Yaw, Pitch, Roll;
	unsigned short	Buttons;
	float	Trigger, AxisX, AxisY;
	float	GripValue;
	bool	ButtonA, ButtonB;
	bool	UseQuat;          // FPS hands: send this quaternion directly (no Euler round-trip = no gimbal lock)
	double	QW, QX, QY, QZ;
} TController, * PController;

double g_PrevGunQ[4] = { 1, 0, 0, 0 }, g_PrevSupQ[4] = { 1, 0, 0, 0 };
double FPS_YawSign = -1.0, FPS_PitchSign = -1.0; // InvertPitch: true = -1. InvertYaw is swapped: false = -1, true = +1

// Builds a hand orientation straight from the camera basis (no yaw/pitch extraction -> no gimbal lock).
// Applies yaw about camera up, then pitch about the resulting right axis, then roll about the hand's own forward.
inline void BuildFpsHandQuat(const double f[3], const double r[3], const double u[3],
                             double yawDeg, double pitchDeg, double rollDeg, TController* out, double prevQ[4])
{
	double gy = yawDeg * (3.14159265358979323846 / 180.0);
	double gp = pitchDeg * (3.14159265358979323846 / 180.0);
	double gr = rollDeg * (3.14159265358979323846 / 180.0);
	double cy = cos(gy), sy = sin(gy), cp = cos(gp), sp = sin(gp), cr = cos(gr), sr = sin(gr);

	double f1[3], r1[3], f2[3], u2[3], nr[3], nu[3];
	for (int i = 0; i < 3; ++i) {
		f1[i] = f[i] * cy + r[i] * sy;
		r1[i] = r[i] * cy - f[i] * sy;
		f2[i] = f1[i] * cp + u[i] * sp;
		u2[i] = u[i] * cp - f1[i] * sp;
		nr[i] = r1[i] * cr + u2[i] * sr;
		nu[i] = -r1[i] * sr + u2[i] * cr;
	}
	double m[3][3];
	for (int i = 0; i < 3; ++i) { m[i][0] = nr[i]; m[i][1] = nu[i]; m[i][2] = -f2[i]; }
	vr::HmdQuaternion_t q = MatrixToQuaternion(m);
	// q and -q are the same rotation; keep the same sign as last frame so the pose never flips/spins at the pole.
	if (q.w * prevQ[0] + q.x * prevQ[1] + q.y * prevQ[2] + q.z * prevQ[3] < 0.0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
	prevQ[0] = q.w; prevQ[1] = q.x; prevQ[2] = q.y; prevQ[3] = q.z;
	out->UseQuat = true; out->QW = q.w; out->QX = q.x; out->QY = q.y; out->QZ = q.z;
}


#define GRIP_BTN	0x0001
#define THUMB_BTN	0x0002
#define MENU_BTN	0x0010
#define SYS_BTN		0x0020

THMD MyHMD;
TController MyCtrl, MyCtrl2;
bool HMDConnected = false, ctrlsConnected = false;
std::string ControllerType = "vive_controller";

double DegToRad(double f) { return f * (3.14159265358979323846 / 180); }

int KEY_ID_LEFT_TRIGGER, KEY_ID_LEFT_MENU, KEY_ID_LEFT_GRIP, KEY_ID_LEFT_SYSTEM;
int KEY_ID_RIGHT_TRIGGER, KEY_ID_RIGHT_MENU, KEY_ID_RIGHT_GRIP, KEY_ID_RIGHT_SYSTEM;
int KEY_ID_CHANGE_WEAPON, KEY_ID_TELEPORT, KEY_ID_MOVE_CONTROLLERS_FORWARD, KEY_ID_MOVE_CONTROLLERS_BACK;
int KEY_ID_PLAYER_RISE_HIGHER, KEY_ID_PLAYER_RISE_LOWER, KEY_ID_PLAYER_RISE_RESET;
int KEY_ID_CROUCH, KEY_ID_PRONE, KEY_ID_LEFT_CTRL_MOTION;
int KEY_ID_ROTATION_CONTROLLERS_RESET;
int KEY_ID_LEFT_HAND_ROTATE, KEY_ID_RIGHT_HAND_ROTATE;
int KEY_ID_HAND_FORWARD, KEY_ID_HAND_BACK, KEY_ID_HAND_LEFT, KEY_ID_HAND_RIGHT;
int KEY_ID_LEFT_TRACKPAD_CLICK, KEY_ID_RIGHT_TRACKPAD_CLICK, KEY_ID_FINE_TUNE;
int KEY_ID_MOTION_MODE_MOVE_UP, KEY_ID_MOTION_MODE_MOVE_DOWN;
int KEY_ID_TURN_LEFT, KEY_ID_TURN_RIGHT;
int KEY_ID_UP, KEY_ID_DOWN, KEY_ID_LEFT, KEY_ID_RIGHT;
int KEY_ID_FPS_MODE_TOGGLE;
int KEY_ID_SAVE_POSE_LEFT = 0, KEY_ID_SAVE_POSE_RIGHT = 0;

bool FPSModeActive = false;
bool PrevFPSModeKey = false;

bool LeftTrackpadPressMode  = true;
bool RightTrackpadPressMode = true;
bool PrevLeftTrackpadToggle  = false;
bool PrevRightTrackpadToggle = false;
int  KEY_ID_LEFT_TRACKPAD_TOGGLE  = 0;
int  KEY_ID_RIGHT_TRACKPAD_TOGGLE = 0;
float SmoothTriggerL = 0.0f;
float SmoothTriggerR = 0.0f;

bool IsSmoothResetting = false;
double SmoothResetTimer = 0.0;
double SmoothResetDuration = 0.4;
double StartResetX_L = 0, StartResetY_L = 0, StartResetZ_L = 0;
double StartResetX_R = 0, StartResetY_R = 0, StartResetZ_R = 0;
double StartResetRotL_X = 0, StartResetRotL_Y = 0;
double StartResetRotL_Z = 0, StartResetRotR_Z = 0;
double StartResetRotR_X = 0, StartResetRotR_Y = 0;
bool PrevSmoothResetKey = false;
int KEY_ID_SMOOTH_HAND_RESET = 0;

double SmoothHMD_X = 0, SmoothHMD_Y = 0, SmoothHMD_Z = 0;
double SmoothL_X = 0, SmoothL_Y = 0, SmoothL_Z = 0;
double SmoothR_X = 0, SmoothR_Y = 0, SmoothR_Z = 0;
float PositionSmoothness = 0.15f;
bool SmoothHMD_Init = false;
bool SmoothL_Init = false;
bool SmoothR_Init = false;

int KEY_ID_FREELOOK;
float FreelookYawOffset = 0.0f;
float FreelookPitchOffset = 0.0f;

bool AutoSwingActive = false;
bool AutoSwingHoldMode = false; 
bool PrevAutoSwingKey = false;
int KEY_ID_AUTO_SWING_TOGGLE;

HHOOK g_hKeyboardHook = NULL;
std::atomic<bool> g_IsDriverKey[256] = {};
std::atomic<bool> g_SwallowedKeyState[256] = { false };
std::map<int, int> g_KeyRemapper;
std::atomic<bool> g_RemappedKeyActive[256] = { false };

bool IsKeyDown(int vkCode) {
    if (vkCode > 0 && vkCode < 256) {
        if (g_IsDriverKey[vkCode].load(std::memory_order_relaxed)) {
            return g_SwallowedKeyState[vkCode].load(std::memory_order_relaxed);
        }
    }
    return (GetAsyncKeyState(vkCode) & 0x8000) != 0;
}

int KEY_ID_PUMP_ACTION;
double PumpTimer = 0.0;
bool PrevPumpKey = false;
double PumpDuration = 0.35;
double PumpDistance = 0.15;

// --- SNIPER / BIPOD MODE GLOBALS ---
bool SniperModeActive = false;
bool PrevSniperModeKey = false;
int KEY_ID_SNIPER_MODE = 0;
bool SniperToggleMode = false;
bool SniperJustStarted = false;
bool SniperExiting = false;
bool SniperJustEnded = false;
double SniperAnchorX = 0, SniperAnchorY = 0, SniperAnchorZ = 0;
double SniperHmdOffsetX = 0, SniperHmdOffsetY = 0, SniperHmdOffsetZ = 0;
double SniperExitTimer = 0.0;

bool LeftHandLocked = false;
bool RightHandLocked = false;
bool PrevLeftLockKey = false;
bool PrevRightLockKey = false;
int KEY_ID_LEFT_HAND_LOCK, KEY_ID_RIGHT_HAND_LOCK;
double LockedLeftPos[3] = { 0, 0, 0 };
double LockedRightPos[3] = { 0, 0, 0 };
double LockedLeftYaw = 0, LockedLeftPitch = 0, LockedLeftRoll = 0;
bool LockedLeftUseQuat = false; double LockedLeftQ[4] = { 1, 0, 0, 0 };
bool LockedRightUseQuat = false; double LockedRightQ[4] = { 1, 0, 0, 0 };
double LockedRightYaw = 0, LockedRightPitch = 0, LockedRightRoll = 0;

int KEY_ID_TRACKPAD_DISTANCE = 0;
float TrackpadHalf = 0.5f;
float TrackpadFull = 1.0f;
float CurrentAxisX_L = 0.0f;
float CurrentAxisY_L = 0.0f;
float CurrentAxisX_R = 0.0f;
float CurrentAxisY_R = 0.0f;

double HMDPosX = 0, HMDPosY = 0;
int KEY_ID_HMD_FORWARD, KEY_ID_HMD_BACK, KEY_ID_HMD_LEFT, KEY_ID_HMD_RIGHT, KEY_ID_HMD_MOVE_RESET;
std::string g_IniPathStr = "";

float FPS_HandCenterOffsetX = 0.0f;
float FPS_HandCenterOffsetZ = 0.0f;
float FPS_GunOffsetX = 0.0f;
float FPS_GunOffsetY = -0.1f;
float FPS_GunOffsetZ = 0.3f;
float FPS_LeftOffsetX = -0.15f;
float FPS_LeftOffsetY = -0.15f;
float FPS_LeftOffsetZ = 0.15f;

float FPS_GunZeroYaw = 0.0f;
float FPS_GunZeroPitch = 0.0f;
float FPS_GunZeroRoll = 0.0f;
float FPS_LeftZeroYaw = 0.0f;
float FPS_LeftZeroPitch = 0.0f;
float FPS_LeftZeroRoll = 0.0f;

// --- GESTURE STRUCTS & ENGINE ---
struct GestureStep {
    std::string command;
    std::string target;
    std::string axis;
    double value = 0.0;
    double duration = 0.0;
    double delay = 0.0;
    bool isParallel = false;
    bool isHmdRelative = false;
	bool ignoreRot = false;
    double poseL[6] = {0,0,0,0,0,0};
    double poseR[6] = {0,0,0,0,0,0};
    bool hasPoseL = false, hasPoseR = false;
};

struct ActiveStep {
    int index;
    double timer;
    bool initialized;
    double initOffsetL[3], initOffsetR[3], initRotL[3], initRotR[3];

    ActiveStep(int i, double t) : index(i), timer(t), initialized(false) {
        memset(initOffsetL, 0, sizeof(initOffsetL));
        memset(initOffsetR, 0, sizeof(initOffsetR));
        memset(initRotL, 0, sizeof(initRotL));
        memset(initRotR, 0, sizeof(initRotR));
    }
};

struct CachedGesture {
    std::vector<GestureStep> steps;
    FILETIME lastModified;
};
std::map<std::string, CachedGesture> g_GestureCache;
std::vector<GestureStep> ActiveSequence;
std::vector<ActiveStep> RunningSteps;
int NextStepIndex = 0;
int g_ActiveGestureKey = 0;
int PrevPressedGesture = 0;
bool g_GestureLoop = false;

double ShakeTimeL = 0.0, ShakeTimeR = 0.0;
double ShakeTimerL = 0.0, ShakeTimerR = 0.0;
double ShakeIntensityL[3] = {0,0,0}, ShakeIntensityR[3] = {0,0,0};
bool ManualShakeL_H = false, ManualShakeR_H = false;
bool ManualShakeL_V = false, ManualShakeR_V = false;
int KEY_ID_SHAKE_LEFT_H = 0, KEY_ID_SHAKE_RIGHT_H = 0;
int KEY_ID_SHAKE_LEFT_V = 0, KEY_ID_SHAKE_RIGHT_V = 0;

int KEY_ID_LEAN_RIGHT = 0, KEY_ID_LEAN_LEFT = 0;
double LeanOffset = 0.0;
double g_LeanOffsetX = 0.0, g_LeanOffsetY = 0.0;

FILETIME GetFileModTime(const std::string& path) {
    WIN32_FILE_ATTRIBUTE_DATA fileInfo;
    if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fileInfo)) {
        return fileInfo.ftLastWriteTime;
    }
    FILETIME empty = {0, 0};
    return empty;
}

double SeqOffsetL[3] = { 0, 0, 0 }, SeqOffsetR[3] = { 0, 0, 0 };
double SeqRotL[3] = { 0, 0, 0 }, SeqRotR[3] = { 0, 0, 0 };

double StartSeqOffsetL[3] = { 0, 0, 0 }, StartSeqOffsetR[3] = { 0, 0, 0 };
double StartSeqRotL[3] = { 0, 0, 0 }, StartSeqRotR[3] = { 0, 0, 0 };
double StepInitialOffsetL[3] = { 0, 0, 0 }, StepInitialOffsetR[3] = { 0, 0, 0 };
double StepInitialRotL[3] = { 0, 0, 0 }, StepInitialRotR[3] = { 0, 0, 0 };

double SeqGripL = -1.0, SeqGripR = -1.0;
double SeqTrigL = -1.0, SeqTrigR = -1.0;
double SeqMenuL = -1.0, SeqMenuR = -1.0;
double SeqSysL = -1.0, SeqSysR = -1.0;
double SeqTrackpadCL = -1.0, SeqTrackpadCR = -1.0;
double SeqTrackpadUL = -1.0, SeqTrackpadUR = -1.0;
double SeqTrackpadDL = -1.0, SeqTrackpadDR = -1.0;
double SeqTrackpadLL = -1.0, SeqTrackpadLR = -1.0;
double SeqTrackpadRL = -1.0, SeqTrackpadRR = -1.0;
double SeqBtnAL = -1.0, SeqBtnAR = -1.0;
double SeqBtnBL = -1.0, SeqBtnBR = -1.0;

int KEY_ID_GESTURE_CUSTOM_5, KEY_ID_GESTURE_CUSTOM_6, KEY_ID_GESTURE_CUSTOM_7, KEY_ID_GESTURE_CUSTOM_8, KEY_ID_GESTURE_CUSTOM_9, KEY_ID_GESTURE_CUSTOM_0;
int KEY_ID_GESTURE_MINUS = 0, KEY_ID_GESTURE_EQUALS = 0;

void LoadGestureSequence(const std::string& path) {
    ActiveSequence.clear();
    g_GestureLoop = false;
    std::ifstream file(path);
    if (!file.is_open()) {
        vr::VRDriverLog()->Log(("FlatVR: Failed to open gesture: " + path + "\n").c_str());
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();

    size_t hashPos;
    while ((hashPos = content.find('#')) != std::string::npos) {
        size_t newlinePos = content.find('\n', hashPos);
        if (newlinePos == std::string::npos) content.erase(hashPos);
        else content.erase(hashPos, newlinePos - hashPos);
    }

    std::stringstream ss(content);
    std::string token;
    bool inPar = false;
    bool mirror = false;
    double currentDelay = 0.0;

    while (ss >> token) {
        if (token == "PAR") { inPar = true; continue; }
        if (token == "END") { inPar = false; continue; }
        if (token == "MIRROR") { mirror = !mirror; continue; }
        if (token == "LOOP") { g_GestureLoop = true; continue; }

        if (token == "DELAY") {
            ss >> currentDelay;
            continue;
        }

        GestureStep step;
        step.command = token;
        step.isParallel = inPar;
        step.delay = currentDelay;
        currentDelay = 0.0;

        if (token == "WAIT" || token == "RETURN") {
            ss >> step.duration;
        }
        else if (token == "MOVE" || token == "ROTATE" || token == "BUTTON" || token == "SHAKE") {
            ss >> step.target >> step.axis >> step.value >> step.duration;
        }
        else if (token == "POSE") {
            std::string filename, relFlag;
            ss >> step.target >> filename >> step.duration;
            
            // Check for REL=HMD flag
            if (ss >> relFlag && relFlag == "REL=HMD") {
                step.isHmdRelative = true;
            }
			
            std::string posePath = g_DriverFolderStr + "/Resources/Pose_Config/" + g_ActiveProfileName + "/" + filename + ".pose";
            if (GetFileAttributesA(posePath.c_str()) == INVALID_FILE_ATTRIBUTES)
                posePath = g_DriverFolderStr + "/Resources/Pose_Config/Default/" + filename + ".pose";

            std::ifstream pf(posePath);
            if (pf.is_open()) {
                std::string pline;
                while (std::getline(pf, pline)) {
                    if (pline.empty() || pline[0] == '#') continue;
                    std::stringstream pss(pline);
                    std::string hand;
                    pss >> hand;
                    double v[6] = {0,0,0,0,0,0};

                    if (hand == "L" || hand == "R") {
                        pss >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
                        
                        std::string flag;
                        while (pss >> flag) {
                            if (flag == "REL=HMD") step.isHmdRelative = true;
                            if (flag == "ROT=IGNORE") step.ignoreRot = true;
                        }

                        bool wantL = (step.target == "L" || step.target == "B");
                        bool wantR = (step.target == "R" || step.target == "B");

                        if (hand == "L") {
                            if (wantL) {
                                for(int i=0;i<6;i++) step.poseL[i] = v[i];
                                step.hasPoseL = true;
                            } else if (wantR) {
                                // FAILSAFE: File says L, but gesture wants R. Force it to R.
                                vr::VRDriverLog()->Log(("FlatVR: WARNING - Pose " + filename + " has 'L' data, but target was 'R'. Forcing to R.\n").c_str());
                                for(int i=0;i<6;i++) step.poseR[i] = v[i];
                                step.hasPoseR = true;
                            }
                        } else if (hand == "R") {
                            if (wantR) {
                                for(int i=0;i<6;i++) step.poseR[i] = v[i];
                                step.hasPoseR = true;
                            } else if (wantL) {
                                // FAILSAFE: File says R, but gesture wants L. Force it to L.
                                vr::VRDriverLog()->Log(("FlatVR: WARNING - Pose " + filename + " has 'R' data, but target was 'L'. Forcing to L.\n").c_str());
                                for(int i=0;i<6;i++) step.poseL[i] = v[i];
                                step.hasPoseL = true;
                            }
                        }
                    }
				}
                pf.close();
            } else {
                vr::VRDriverLog()->Log(("FlatVR: Pose file not found: " + posePath + "\n").c_str());
            }
        }
        else {
            continue;
        }

        if (mirror) {
            if (step.target == "L") step.target = "R";
            else if (step.target == "R") step.target = "L";

            if (step.axis == "X" || step.axis == "YAW") {
                step.value = -step.value;
            }

            if (step.command == "POSE") {
                bool tempHas = step.hasPoseL;
                step.hasPoseL = step.hasPoseR;
                step.hasPoseR = tempHas;
                for (int i=0; i<6; i++) {
                    double temp = step.poseL[i];
                    step.poseL[i] = step.poseR[i];
                    step.poseR[i] = temp;
                }
                if (step.hasPoseL) { step.poseL[0] = -step.poseL[0]; step.poseL[3] = -step.poseL[3]; }
                if (step.hasPoseR) { step.poseR[0] = -step.poseR[0]; step.poseR[3] = -step.poseR[3]; }
            }
        }

        if (step.duration > 0) {
            ActiveSequence.push_back(step);
        }
    }
    vr::VRDriverLog()->Log(("FlatVR: Loaded " + std::to_string(ActiveSequence.size()) + " steps from " + path + "\n").c_str());
}

struct ButtonState {
    bool isToggle = false;
    bool state = false;
    bool prev = false;
};

ButtonState Btn_LeftTrigger, Btn_LeftGrip, Btn_LeftMenu, Btn_LeftSystem;
ButtonState Btn_RightTrigger, Btn_RightGrip, Btn_RightMenu, Btn_RightSystem;
ButtonState Btn_LeftA, Btn_LeftB, Btn_RightA, Btn_RightB;
ButtonState Btn_LeftTrackpadClick, Btn_RightTrackpadClick;
ButtonState Btn_Up, Btn_Down, Btn_Left, Btn_Right;
ButtonState Btn_TurnLeft, Btn_TurnRight, Btn_Teleport, Btn_ChangeWeapon;
ButtonState Btn_Crouch, Btn_Prone;
ButtonState Btn_TrackpadDistance;

bool CheckButton(int keyCode, ButtonState& state) {
    if (keyCode == 0) return false;
    bool isPressed = IsKeyDown(keyCode);

    if (state.isToggle) {
        if (isPressed && !state.prev) {
            state.state = !state.state;
        }
        state.prev = isPressed;
        return state.state;
    } else {
        return isPressed;
    }
}

double ArmSwingTime = 0.0;
double ArmSwingSpeed = 3.0;
double ArmSwingDistance = 0.35;

int KEY_ID_LEFT_A, KEY_ID_LEFT_B, KEY_ID_RIGHT_A, KEY_ID_RIGHT_B;

#define StepPos 0.01;
double HMDPosZ = 0;
double FirstCtrlPos[3] = { 0, 0, 0 }, SecondCtrlPos[3] = { 0, 0, 0 };
double CtrlsRoll = 0, CtrlsPitch = 0;
float CrouchOffsetZ = 0;
double LeftZOffset_Target = 0.0, RightZOffset_Target = 0.0;

THMD mouseHMD;
THMD cameraHMD;
bool firstCP = true;
int m_HalfWidth = 1920 / 2, m_HalfHeight = 1080 / 2;
float mouseSensetivePitch = 0.04, mouseSensetiveYaw = 0.05;
float mouseYaw = 0, mousePitch = 0;
float HandMoveSensitivity = 5.0f;
float HandCenterOffsetX = -0.05f;

float RotationSpeed = 0.05f;
float FineTuneMultiplier = 0.1f;
float ScrollPushPullSpeed = 0.0005f;
float OculusHeight = 0.05f;
float OculusSeparation = 0.08f;

HWND SteamVRHeadsetWindow = 0;
bool HeadsetWindowFocused = false;
bool VRMode = false;
float VRAimingAngleOffset;
float oldPitch = 0, deltaPitch = 0, oldRoll = 0, deltaRoll = 0;
double CtrlPosOffset[3] = { 0, 0, 0 };
double SecondCtrlYPROffset[3] = { 0, 0, 0 };
double SecondCtrlLastYPR[3] = { 0, 0, 0 };

float LeftHandYaw = 0, LeftHandPitch = 0, LeftAngleDist = 0, LeftRadDist = 0, LeftYOffset = 0;
float RightAngleDist = 0;
float RightHandYaw = 0, RightHandPitch = 0, RightRadDist = 0;

double LeftHandRotOffset[3] = { 0, 0, 0 };
double RightHandRotOffset[3] = { 0, 0, 0 };
bool LeftHandIsRotating = false, RightHandIsRotating = false;
bool PrevLeftRotateKey = false, PrevRightRotateKey = false;
double RightYOffset = 0.0, LeftRotateYOffset = 0.0;
double LeftXOffset = 0.0, LeftZOffset = 0.0, RightXOffset = 0.0, RightZOffset = 0.0;
float HandRadDist = 0;
float MotionLeftRadDist = 0, MotionLeftAngleDist = 0;
#define OffsetPitchByDefault -45
float ZPosSensetive = 0.009;

HHOOK g_hMouseHook = NULL;
std::atomic<int> g_MouseWheelDelta(0);
DWORD g_HookThreadId = 0;
std::thread g_HookThread;

bool NeckModelEnabled = true;
bool PrevNeckToggleKey = false;
bool PrevProfileCycleKey = false;
int KEY_ID_PROFILE_CYCLE = 0;
int KEY_ID_NECK_MODEL_TOGGLE = 0;

bool PrevSavePoseL = false, PrevSavePoseR = false;

std::string GetAppIdentifierFromPid(DWORD pid) {
    if (pid == 0) return "Default";
    char name[MAX_PATH] = {0};
    HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
    if (hProcess) {
        DWORD size = MAX_PATH;
        if (QueryFullProcessImageNameA(hProcess, 0, name, &size)) {
            std::string path(name);
            size_t pos = path.find_last_of("\\/");
            if (pos != std::string::npos) path = path.substr(pos + 1);
            size_t ext = path.find_last_of(".");
            if (ext != std::string::npos) path = path.substr(0, ext);
            CloseHandle(hProcess);

            if (path == "vrserver" || path == "vrmonitor" || path == "steam" || path == "steamwebhelper") {
                return "Default";
            }
            return path;
        }
        CloseHandle(hProcess);
    }
    return "Default";
}

void ResetRuntimeState() {
    FPSModeActive = false;
    LeftHandLocked = false;
    RightHandLocked = false;
    LeftHandIsRotating = false;
    RightHandIsRotating = false;
    IsSmoothResetting = false;

    RunningSteps.clear();
    NextStepIndex = 0;
    ActiveSequence.clear();
    SeqOffsetL[0] = SeqOffsetL[1] = SeqOffsetL[2] = 0;
    SeqOffsetR[0] = SeqOffsetR[1] = SeqOffsetR[2] = 0;
    SeqRotL[0] = SeqRotL[1] = SeqRotL[2] = 0;
    SeqRotR[0] = SeqRotR[1] = SeqRotR[2] = 0;

    LeftXOffset = LeftRotateYOffset = LeftZOffset = 0.0;
    RightXOffset = RightYOffset = RightZOffset = 0.0;
    LeftZOffset_Target = RightZOffset_Target = 0.0;
    LeftHandRotOffset[0] = LeftHandRotOffset[1] = LeftHandRotOffset[2] = 0.0;
    RightHandRotOffset[0] = RightHandRotOffset[1] = RightHandRotOffset[2] = 0.0;

    SeqGripL = SeqGripR = SeqTrigL = SeqTrigR = -1.0;
    SeqMenuL = SeqMenuR = SeqSysL = SeqSysR = -1.0;
    SeqTrackpadCL = SeqTrackpadCR = SeqTrackpadUL = SeqTrackpadUR = -1.0;
    SeqTrackpadDL = SeqTrackpadDR = SeqTrackpadLL = SeqTrackpadLR = -1.0;
    SeqTrackpadRL = SeqTrackpadRR = SeqBtnAL = SeqBtnAR = SeqBtnBL = SeqBtnBR = -1.0;
}

void LoadProfile(const std::string& profileName) {
    if (g_ActiveProfileName == profileName && !g_IniPathStr.empty()) return;
    g_ActiveProfileName = profileName;

    std::string profilePath = g_DriverFolderStr + "/Profiles/" + profileName + ".ini";

    if (GetFileAttributesA(profilePath.c_str()) != INVALID_FILE_ATTRIBUTES) {
        g_IniPathStr = profilePath;
    } else {
        g_IniPathStr = g_DriverFolderStr + "/FlatVR.ini";
    }

    ResetRuntimeState();
    LoadConfig();
    lastIniMod = GetFileModTime(g_IniPathStr);

    vr::VRDriverLog()->Log(("FlatVR: Loaded profile: " + g_ActiveProfileName + " -> " + g_IniPathStr + "\n").c_str());
}

void CycleProfile() {
    WIN32_FIND_DATAA fd;
    std::string searchPath = g_DriverFolderStr + "/Profiles/*.ini";
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &fd);
    std::vector<std::string> profiles;
    profiles.push_back("Default");

    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            std::string name = fd.cFileName;
            size_t ext = name.find_last_of('.');
            if (ext != std::string::npos) name = name.substr(0, ext);
            if (name != "Default" && name != "." && name != "..") {
                profiles.push_back(name);
            }
        } while (FindNextFileA(hFind, &fd));
        FindClose(hFind);
    }

    std::sort(profiles.begin(), profiles.end());

    auto it = std::find(profiles.begin(), profiles.end(), g_ActiveProfileName);
    if (it != profiles.end()) {
        it++;
        if (it == profiles.end()) it = profiles.begin();
        LoadProfile(*it);
    } else {
        LoadProfile(profiles[0]);
    }
}

LRESULT CALLBACK MouseHookProc(int nCode, WPARAM wParam, LPARAM lParam) {
	if (nCode >= 0 && wParam == WM_MOUSEWHEEL) {
		MSLLHOOKSTRUCT* pMouseStruct = (MSLLHOOKSTRUCT*)lParam;
		short delta = (short)HIWORD(pMouseStruct->mouseData);
		g_MouseWheelDelta += delta;
	}
	return CallNextHookEx(g_hMouseHook, nCode, wParam, lParam);
}

LRESULT CALLBACK KeyboardHookProc(int nCode, WPARAM wParam, LPARAM lParam) {
    if (nCode >= 0) {
        KBDLLHOOKSTRUCT* pKey = (KBDLLHOOKSTRUCT*)lParam;
        int vkCode = pKey->vkCode;

        // ONLY DO THIS IF THE HEADSET WINDOW IS FOCUSED
        if (SteamVRHeadsetWindow != 0 && SteamVRHeadsetWindow == GetForegroundWindow()) {
            
            // --- KEY REMAPPER ---
            auto it = g_KeyRemapper.find(vkCode);
            if (it != g_KeyRemapper.end()) {
                int mappedVk = it->second;
                
                INPUT input = {0};
                input.type = INPUT_KEYBOARD;
                input.ki.wVk = mappedVk;
                
                if (wParam == WM_KEYUP || wParam == WM_SYSKEYUP) {
                    input.ki.dwFlags = KEYEVENTF_KEYUP;
                    g_RemappedKeyActive[vkCode] = false; // Mark as released
                } else {
                    g_RemappedKeyActive[vkCode] = true;  // Mark as held
                }
                
                SendInput(1, &input, sizeof(INPUT));
                return 1; // Swallow the original key
            }
            // --------------------

            // Existing SwallowKeys logic
            if (vkCode > 0 && vkCode < 256 && g_IsDriverKey[vkCode].load(std::memory_order_relaxed)) {
                if (wParam == WM_KEYDOWN || wParam == WM_SYSKEYDOWN) {
                    g_SwallowedKeyState[vkCode] = true;
                } else if (wParam == WM_KEYUP || wParam == WM_SYSKEYUP) {
                    g_SwallowedKeyState[vkCode] = false;
                }
                return 1;
            }
        }
    }
    return CallNextHookEx(g_hKeyboardHook, nCode, wParam, lParam);
}

double OffsetYPR(float f, float f2)
{
	f -= f2;
	if (f < -180) f += 360;
	else if (f > 180) f -= 360;
	return f;
}

void MouseToYawPitch()
{
	POINT mousePos;
	if (firstCP) { SetCursorPos(m_HalfWidth, m_HalfHeight); firstCP = false; }
	GetCursorPos(&mousePos);

	mouseYaw += (mousePos.x - m_HalfWidth) * mouseSensetiveYaw * -1;
	if (mouseYaw < -180) mouseYaw += 360;
	else if (mouseYaw > 180) mouseYaw -= 360;
	mouseHMD.Yaw = mouseYaw;

	mousePitch += (mousePos.y - m_HalfHeight) * mouseSensetivePitch * -1;

	if (!LeftHandIsRotating && !RightHandIsRotating) {
		if (mousePitch > 90) mousePitch = 90;
		else if (mousePitch < -90) mousePitch = -90;
	}
	mouseHMD.Pitch = mousePitch;

	SetCursorPos(m_HalfWidth, m_HalfHeight);
}

// --- HMD POSE CALCULATION ---
void GetHMDData(__out THMD* HMD)
{
	if (IsKeyDown(KEY_ID_PLAYER_RISE_HIGHER)) HMDPosZ += StepPos;
	if (IsKeyDown(KEY_ID_PLAYER_RISE_LOWER)) HMDPosZ -= StepPos;
	if (IsKeyDown(KEY_ID_PLAYER_RISE_RESET)) HMDPosZ = 0;

	// --- LEAN / PEEK ---
	if (IsKeyDown(KEY_ID_LEAN_RIGHT)) LeanOffset += 0.8 * g_DtSeconds;
	if (IsKeyDown(KEY_ID_LEAN_LEFT)) LeanOffset -= 0.8 * g_DtSeconds;
	if (LeanOffset > 0.4) LeanOffset = 0.4;
	if (LeanOffset < -0.4) LeanOffset = -0.4;

	double leanYaw = DegToRad(cameraHMD.Yaw);
	g_LeanOffsetX = LeanOffset * cos(leanYaw);
	g_LeanOffsetY = -LeanOffset * sin(leanYaw);

	double moveSpeed = 0.01;
	double yawRad = DegToRad(cameraHMD.Yaw);

	if (IsKeyDown(KEY_ID_HMD_FORWARD)) { HMDPosX -= sin(yawRad) * moveSpeed; HMDPosY -= cos(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_BACK))    { HMDPosX += sin(yawRad) * moveSpeed; HMDPosY += cos(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_LEFT))    { HMDPosX -= cos(yawRad) * moveSpeed; HMDPosY += sin(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_RIGHT))   { HMDPosX += cos(yawRad) * moveSpeed; HMDPosY -= sin(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_MOVE_RESET)) { HMDPosX = 0; HMDPosY = 0; LeanOffset = 0.0; }

	HMD->X = HMDPosX + g_LeanOffsetX + SniperHmdOffsetX;
	HMD->Y = HMDPosY + g_LeanOffsetY + SniperHmdOffsetZ;
	HMD->Z = HMDPosZ - CrouchOffsetZ + SniperHmdOffsetY;
	HMD->Yaw = (cameraHMD.Yaw + FreelookYawOffset) * -1;
    HMD->Pitch = (cameraHMD.Pitch + FreelookPitchOffset) * -1;
    HMD->Roll = -LeanOffset * 20.0;
}

// --- CONTROLLER POSE CALCULATION & GESTURES ---
void GetControllersData(__out TController* FirstController, __out TController* SecondController)
{
	float dt = (float)g_DtSeconds;

	if (ShakeTimerL > 0) {
		ShakeTimeL += dt * 40.0;
		ShakeTimerL -= dt;
		if (ShakeTimerL <= 0) { ShakeIntensityL[0] = ShakeIntensityL[1] = ShakeIntensityL[2] = 0; }
	} else if (ManualShakeL_H || ManualShakeL_V) {
		ShakeTimeL += dt * 40.0;
	} else {
		ShakeTimeL = 0;
	}

	if (ShakeTimerR > 0) {
		ShakeTimeR += dt * 40.0;
		ShakeTimerR -= dt;
		if (ShakeTimerR <= 0) { ShakeIntensityR[0] = ShakeIntensityR[1] = ShakeIntensityR[2] = 0; }
	} else if (ManualShakeR_H || ManualShakeR_V) {
		ShakeTimeR += dt * 40.0;
	} else {
		ShakeTimeR = 0;
	}

	FirstController->AxisX = 0; FirstController->AxisY = 0; FirstController->Buttons = 0; FirstController->Trigger = 0;
	SecondController->AxisX = 0; SecondController->AxisY = 0; SecondController->Buttons = 0; SecondController->Trigger = 0;
	SecondCtrlYPROffset[0] = 0; SecondCtrlYPROffset[1] = 0; SecondCtrlYPROffset[2] = 0;
	FirstController->Roll = 0;
	SecondController->Roll = 0;
	FirstController->UseQuat = false;
	SecondController->UseQuat = false;

	if (HeadsetWindowFocused)
	{
		if (IsKeyDown(KEY_ID_ROTATION_CONTROLLERS_RESET)) {
			CtrlsRoll = 0; CtrlsPitch = 0;
			LeftHandRotOffset[0] = LeftHandRotOffset[1] = LeftHandRotOffset[2] = 0.0;
			RightHandRotOffset[0] = RightHandRotOffset[1] = RightHandRotOffset[2] = 0.0;
			LeftXOffset = 0.0; LeftRotateYOffset = 0.0; LeftZOffset = 0.0; LeftZOffset_Target = 0.0;
			RightXOffset = 0.0; RightYOffset = 0.0; RightZOffset = 0.0; RightZOffset_Target = 0.0;

            SeqOffsetL[0] = SeqOffsetL[1] = SeqOffsetL[2] = 0;
            SeqOffsetR[0] = SeqOffsetR[1] = SeqOffsetR[2] = 0;
            SeqRotL[0] = SeqRotL[1] = SeqRotL[2] = 0;
            SeqRotR[0] = SeqRotR[1] = SeqRotR[2] = 0;
		}
	}

	LeftHandYaw = 0; LeftHandPitch = 0; LeftAngleDist = 0; LeftRadDist = 0;
	RightHandYaw = 0; RightHandPitch = 0; RightAngleDist = 0; RightRadDist = 0;

	bool isCrouching = HeadsetWindowFocused && CheckButton(KEY_ID_CROUCH, Btn_Crouch);
	bool isProne = HeadsetWindowFocused && CheckButton(KEY_ID_PRONE, Btn_Prone);
	if (isCrouching || isProne) HMDPosZ = 0;

	float targetHeightOffset = 0.0f;
	if (isCrouching) targetHeightOffset = 0.8f;
	if (isProne) targetHeightOffset = 1.4f;

	float crouchSpeed = 15.0f;

	CrouchOffsetZ += (targetHeightOffset - CrouchOffsetZ) * crouchSpeed * dt;
	if (std::abs(CrouchOffsetZ - targetHeightOffset) < 0.001f) CrouchOffsetZ = targetHeightOffset;

	deltaPitch = mouseHMD.Yaw - oldPitch; oldPitch = mouseHMD.Yaw;
	deltaRoll = mouseHMD.Pitch - oldRoll; oldRoll = mouseHMD.Pitch;

	if (deltaPitch > 360.0f) deltaPitch = 360.0f; if (deltaPitch < -360.0f) deltaPitch = -360.0f;
	if (deltaRoll > 360.0f) deltaRoll = 360.0f; if (deltaRoll < -360.0f) deltaRoll = -360.0f;

	if (LeftHandIsRotating) { LeftHandRotOffset[0] += deltaPitch; LeftHandRotOffset[1] += deltaRoll; }
	if (RightHandIsRotating) { RightHandRotOffset[0] += deltaPitch; RightHandRotOffset[1] += deltaRoll; }

	float currentSensitivity = HandMoveSensitivity;
	float wheelSensitivity = RotationSpeed;
	if (IsKeyDown(KEY_ID_FINE_TUNE)) {
		currentSensitivity *= FineTuneMultiplier;
		wheelSensitivity *= FineTuneMultiplier;
	}

	int wheelDelta = g_MouseWheelDelta.exchange(0);
	if (!HeadsetWindowFocused) wheelDelta = 0;

	if (!LeftHandIsRotating && !RightHandIsRotating) {
		if (AutoSwingActive) {
			if (wheelDelta != 0) {
				ArmSwingSpeed += wheelDelta * 0.005;
				if (ArmSwingSpeed < 0.5) ArmSwingSpeed = 0.5;
				if (ArmSwingSpeed > 15.0) ArmSwingSpeed = 15.0;
			}
		} else if (!IsSmoothResetting) {
			if (wheelDelta != 0) {
				LeftZOffset_Target -= wheelDelta * ScrollPushPullSpeed;
				RightZOffset_Target -= wheelDelta * ScrollPushPullSpeed;
				if (LeftZOffset_Target < -0.8) LeftZOffset_Target = -0.8;
				if (LeftZOffset_Target > 0.8) LeftZOffset_Target = 0.8;
				if (RightZOffset_Target < -0.8) RightZOffset_Target = -0.8;
				if (RightZOffset_Target > 0.8) RightZOffset_Target = 0.8;
			}
			LeftZOffset += (LeftZOffset_Target - LeftZOffset) * 0.5;
			RightZOffset += (RightZOffset_Target - RightZOffset) * 0.5;
		}
	} else {
		if (LeftHandIsRotating && wheelDelta != 0) LeftHandRotOffset[2] += wheelDelta * wheelSensitivity;
		if (RightHandIsRotating && wheelDelta != 0) RightHandRotOffset[2] += wheelDelta * wheelSensitivity;
	}

	if (LeftHandIsRotating) {
		if (!IsSmoothResetting) {
			if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_UP)) LeftRotateYOffset += currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_DOWN)) LeftRotateYOffset -= currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_HAND_FORWARD)) { LeftZOffset -= currentSensitivity * dt; LeftZOffset_Target -= currentSensitivity * dt; }
			if (IsKeyDown(KEY_ID_HAND_BACK)) { LeftZOffset += currentSensitivity * dt; LeftZOffset_Target += currentSensitivity * dt; }
			if (IsKeyDown(KEY_ID_HAND_LEFT)) LeftXOffset -= currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_HAND_RIGHT)) LeftXOffset += currentSensitivity * dt;
			if (wheelDelta != 0) LeftHandRotOffset[2] += wheelDelta * wheelSensitivity;
		}
	}
	if (RightHandIsRotating) {
		if (!IsSmoothResetting) {
			if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_UP)) RightYOffset += currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_DOWN)) RightYOffset -= currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_HAND_FORWARD)) { RightZOffset -= currentSensitivity * dt; RightZOffset_Target -= currentSensitivity * dt; }
			if (IsKeyDown(KEY_ID_HAND_BACK)) { RightZOffset += currentSensitivity * dt; RightZOffset_Target += currentSensitivity * dt; }
			if (IsKeyDown(KEY_ID_HAND_LEFT)) RightXOffset -= currentSensitivity * dt;
			if (IsKeyDown(KEY_ID_HAND_RIGHT)) RightXOffset += currentSensitivity * dt;
			if (wheelDelta != 0) RightHandRotOffset[2] += wheelDelta * wheelSensitivity;
		}
	}

	// Vanilla HandRadDist
	if (IsKeyDown(KEY_ID_MOVE_CONTROLLERS_FORWARD)) HandRadDist += 0.005f;
	if (IsKeyDown(KEY_ID_MOVE_CONTROLLERS_BACK)) HandRadDist -= 0.005f;
	if (!IsKeyDown(KEY_ID_MOVE_CONTROLLERS_FORWARD) && !IsKeyDown(KEY_ID_MOVE_CONTROLLERS_BACK)) HandRadDist = 0;

	if (IsSmoothResetting) {
		SmoothResetTimer += dt;
		double progress = SmoothResetTimer / SmoothResetDuration;
		if (progress >= 1.0) {
			progress = 1.0;
			IsSmoothResetting = false;
		}

		LeftXOffset = StartResetX_L * (1.0 - progress);
		LeftRotateYOffset = StartResetY_L * (1.0 - progress);
		LeftZOffset = StartResetZ_L * (1.0 - progress);

		RightXOffset = StartResetX_R * (1.0 - progress);
		RightYOffset = StartResetY_R * (1.0 - progress);
		RightZOffset = StartResetZ_R * (1.0 - progress);

		LeftHandRotOffset[0] = StartResetRotL_X * (1.0 - progress);
		LeftHandRotOffset[1] = StartResetRotL_Y * (1.0 - progress);
		LeftHandRotOffset[2] = StartResetRotL_Z * (1.0 - progress);
		RightHandRotOffset[0] = StartResetRotR_X * (1.0 - progress);
		RightHandRotOffset[1] = StartResetRotR_Y * (1.0 - progress);
		RightHandRotOffset[2] = StartResetRotR_Z * (1.0 - progress);

		LeftZOffset_Target = 0.0;
		RightZOffset_Target = 0.0;
	}
	double yawRad = DegToRad(cameraHMD.Yaw);

	if (FPSModeActive) {
		double gunX = FPS_GunOffsetX + RightXOffset;
		double gunY = FPS_GunOffsetY + RightYOffset;
		double gunZ = FPS_GunOffsetZ - RightZOffset;

		double supX = FPS_LeftOffsetX + LeftXOffset;
		double supY = FPS_LeftOffsetY + LeftRotateYOffset;
		double supZ = FPS_LeftOffsetZ - LeftZOffset;

		gunX += SeqOffsetR[0];
		gunY += SeqOffsetR[2];
		gunZ -= SeqOffsetR[1];

		supX += SeqOffsetL[0];
		supY += SeqOffsetL[2];
		supZ -= SeqOffsetL[1];

		if (AutoSwingActive) {
			ArmSwingTime += dt * ArmSwingSpeed;
			double swingForward = sin(ArmSwingTime) * ArmSwingDistance;
			gunZ += swingForward;
			supZ += swingForward;
		}

		if (PumpTimer > 0.0) {
			PumpTimer -= dt;
			if (PumpTimer < 0.0) PumpTimer = 0.0;
			double pumpProgress = 1.0 - (PumpTimer / PumpDuration);
			double pumpOffset = sin(pumpProgress * 3.141592653589793) * PumpDistance;
			supZ -= pumpOffset;
		}

		double hmdWX = HMDPosX;
		double hmdWY = HMDPosZ - CrouchOffsetZ;
		double hmdWZ = HMDPosY;

		double effYawRad   = DegToRad(cameraHMD.Yaw);
		double effPitchRad = DegToRad(cameraHMD.Pitch);

		double fx = -sin(effYawRad) * cos(effPitchRad);
		double fy =  sin(effPitchRad);
		double fz = -cos(effYawRad) * cos(effPitchRad);

		double rx =  cos(effYawRad);
		double ry =  0;
		double rz = -sin(effYawRad);

		double ux =  sin(effYawRad) * sin(effPitchRad);
		double uy =  cos(effPitchRad);
		double uz =  cos(effYawRad) * sin(effPitchRad);

		// --- SNIPER / BIPOD MODE ---
		if (SniperJustStarted) {
			SniperAnchorX = hmdWX + rx*supX + ux*supY + fx*supZ;
			SniperAnchorY = hmdWY + ry*supX + uy*supY + fy*supZ;
			SniperAnchorZ = hmdWZ + rz*supX + uz*supY + fz*supZ;
			SniperJustStarted = false;
			vr::VRDriverLog()->Log("FlatVR: Bipod Locked. Anchoring Support Hand.\n");
		}

		if (SniperModeActive) {
			double reqHmdWX = SniperAnchorX - (rx*supX + ux*supY + fx*supZ);
			double reqHmdWY = SniperAnchorY - (ry*supX + uy*supY + fy*supZ);
			double reqHmdWZ = SniperAnchorZ - (rz*supX + uz*supY + fz*supZ);

			SniperHmdOffsetX = reqHmdWX - HMDPosX;
			SniperHmdOffsetY = reqHmdWY - (HMDPosZ - CrouchOffsetZ);
			SniperHmdOffsetZ = reqHmdWZ - HMDPosY;

			hmdWX = HMDPosX + SniperHmdOffsetX;
			hmdWY = HMDPosZ - CrouchOffsetZ + SniperHmdOffsetY;
			hmdWZ = HMDPosY + SniperHmdOffsetZ;
		} else if (SniperExiting) {
			SniperExitTimer += dt;
			double progress = SniperExitTimer / 0.5;
			if (progress >= 1.0) {
				progress = 1.0;
				SniperExiting = false;
				SniperHmdOffsetX = SniperHmdOffsetY = SniperHmdOffsetZ = 0.0;
				vr::VRDriverLog()->Log("FlatVR: Bipod Released. HMD Returned.\n");
			}

			SniperHmdOffsetX = SniperHmdOffsetX * (1.0 - progress);
			SniperHmdOffsetY = SniperHmdOffsetY * (1.0 - progress);
			SniperHmdOffsetZ = SniperHmdOffsetZ * (1.0 - progress);

			hmdWX = HMDPosX + SniperHmdOffsetX;
			hmdWY = HMDPosZ - CrouchOffsetZ + SniperHmdOffsetY;
			hmdWZ = HMDPosY + SniperHmdOffsetZ;
		} else {
			SniperHmdOffsetX = SniperHmdOffsetY = SniperHmdOffsetZ = 0.0;
		}
		// --- END SNIPER MODE ---

		double gunWX = hmdWX + rx*gunX + ux*gunY + fx*gunZ;
		double gunWY = hmdWY + ry*gunX + uy*gunY + fy*gunZ;
		double gunWZ = hmdWZ + rz*gunX + uz*gunY + fz*gunZ;

		double supWX = hmdWX + rx*supX + ux*supY + fx*supZ;
		double supWY = hmdWY + ry*supX + uy*supY + fy*supZ;
		double supWZ = hmdWZ + rz*supX + uz*supY + fz*supZ;

		SecondController->X = gunWX;
		SecondController->Y = gunWZ;
		SecondController->Z = gunWY;

		FirstController->X = supWX;
		FirstController->Y = supWZ;
		FirstController->Z = supWY;

		double cam_fx = fx, cam_fy = fy, cam_fz = fz;
		double cam_rx = rx, cam_ry = ry, cam_rz = rz;
		double cam_ux = ux, cam_uy = uy, cam_uz = uz;

		double gYaw = DegToRad(FPS_GunZeroYaw - RightHandRotOffset[0] + SeqRotR[0]); 
		double gPitch = DegToRad(FPS_GunZeroPitch + RightHandRotOffset[1] + SeqRotR[1]);

		double f1x = cam_fx * cos(gYaw) + cam_rx * sin(gYaw);
		double f1y = cam_fy * cos(gYaw) + cam_ry * sin(gYaw);
		double f1z = cam_fz * cos(gYaw) + cam_rz * sin(gYaw);

		double f2x = f1x * cos(gPitch) + cam_ux * sin(gPitch);
		double f2y = f1y * cos(gPitch) + cam_uy * sin(gPitch);
		double f2z = f1z * cos(gPitch) + cam_uz * sin(gPitch);

		// Gimbal lock protection for Gun Hand
		double clampF2Y = f2y;
		if (clampF2Y > 1.0) clampF2Y = 1.0;
		if (clampF2Y < -1.0) clampF2Y = -1.0;
		
		// FIX 1: Just clamp the pitch. No custom fallback. (Up is positive)
		SecondController->Pitch = asin(clampF2Y) * 57.29577951308232;

		if (std::abs(f2y) > 0.999) {
			// FIX 2: Use mouseHMD.Yaw (the absolute mouse position) instead of the frozen camera. 
			// This forces the hand to track your mouse continuously, even in Gimbal Lock.
			SecondController->Yaw = -mouseHMD.Yaw; 
		} else {
			double rawYaw = atan2(f2x, f2z) * 57.29577951308232;
			SecondController->Yaw = rawYaw - 180.0; // FIX 3: Normal non-inverted output
		}
		
		SecondController->Roll  = -(FPS_GunZeroRoll + RightHandRotOffset[2] + SeqRotR[2]); 

		// Gimbal-free orientation: sent to SteamVR as-is, the Euler values above are only kept for pose-saving/lock fallbacks.
		{
			double cf[3] = { cam_fx, cam_fy, cam_fz }, cr_[3] = { cam_rx, cam_ry, cam_rz }, cu[3] = { cam_ux, cam_uy, cam_uz };
			BuildFpsHandQuat(cf, cr_, cu,
				FPS_GunZeroYaw + FPS_YawSign * (-RightHandRotOffset[0] + SeqRotR[0]),
				FPS_GunZeroPitch + FPS_PitchSign * (RightHandRotOffset[1] + SeqRotR[1]),
				-(FPS_GunZeroRoll + RightHandRotOffset[2] + SeqRotR[2]), SecondController, g_PrevGunQ);
		}

		double lYaw = DegToRad(FPS_LeftZeroYaw - LeftHandRotOffset[0] + SeqRotL[0]); 
		double lPitch = DegToRad(FPS_LeftZeroPitch + LeftHandRotOffset[1] + SeqRotL[1]);

		double lf1x = cam_fx * cos(lYaw) + cam_rx * sin(lYaw);
		double lf1y = cam_fy * cos(lYaw) + cam_ry * sin(lYaw);
		double lf1z = cam_fz * cos(lYaw) + cam_rz * sin(lYaw);

		double lf2x = lf1x * cos(lPitch) + cam_ux * sin(lPitch);
		double lf2y = lf1y * cos(lPitch) + cam_uy * sin(lPitch);
		double lf2z = lf1z * cos(lPitch) + cam_uz * sin(lPitch);

		// Gimbal lock protection for Support Hand
		double clampLf2Y = lf2y;
		if (clampLf2Y > 1.0) clampLf2Y = 1.0;
		if (clampLf2Y < -1.0) clampLf2Y = -1.0;
		
		FirstController->Pitch = asin(clampLf2Y) * 57.29577951308232;

		if (std::abs(lf2y) > 0.999) {
			// FIX 2 (Left Hand): Use the absolute mouse position
			FirstController->Yaw = -mouseHMD.Yaw;
		} else {
			double rawYawL = atan2(lf2x, lf2z) * 57.29577951308232;
			FirstController->Yaw = rawYawL - 180.0;
		}

		FirstController->Roll = -(FPS_LeftZeroRoll + LeftHandRotOffset[2] + SeqRotL[2]);
		{
			double cf[3] = { cam_fx, cam_fy, cam_fz }, cr_[3] = { cam_rx, cam_ry, cam_rz }, cu[3] = { cam_ux, cam_uy, cam_uz };
			BuildFpsHandQuat(cf, cr_, cu,
				FPS_LeftZeroYaw + FPS_YawSign * (-LeftHandRotOffset[0] + SeqRotL[0]),
				FPS_LeftZeroPitch + FPS_PitchSign * (LeftHandRotOffset[1] + SeqRotL[1]),
				-(FPS_LeftZeroRoll + LeftHandRotOffset[2] + SeqRotL[2]), FirstController, g_PrevSupQ);
		}

	} else {
		FirstCtrlPos[0] = (0.4 - LeftRadDist + HandRadDist) * sin(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, -17), LeftAngleDist)));
		FirstCtrlPos[1] = (0.4 - LeftRadDist + HandRadDist) * cos(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, -17), LeftAngleDist)));
		FirstCtrlPos[2] = cameraHMD.Pitch * ZPosSensetive - CrouchOffsetZ + HMDPosZ;

		double moveX_L = LeftXOffset * cos(yawRad) + LeftZOffset * sin(yawRad);
		double moveZ_L = -LeftXOffset * sin(yawRad) + LeftZOffset * cos(yawRad);

		FirstController->X = FirstCtrlPos[0] + moveX_L + HandCenterOffsetX + HMDPosX + g_LeanOffsetX;
		FirstController->Y = FirstCtrlPos[1] + moveZ_L + HMDPosY + g_LeanOffsetY;
		FirstController->Z = FirstCtrlPos[2] + LeftRotateYOffset - 0.15;

		SecondCtrlPos[0] = (0.4 - RightRadDist + HandRadDist) * sin(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, 17), RightAngleDist)));
		SecondCtrlPos[1] = (0.4 - RightRadDist + HandRadDist) * cos(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, 17), RightAngleDist)));
		SecondCtrlPos[2] = cameraHMD.Pitch * ZPosSensetive - CrouchOffsetZ + HMDPosZ;

		double moveX_R = RightXOffset * cos(yawRad) + RightZOffset * sin(yawRad);
		double moveZ_R = -RightXOffset * sin(yawRad) + RightZOffset * cos(yawRad);

		SecondController->X = SecondCtrlPos[0] + moveX_R + HandCenterOffsetX + HMDPosX + g_LeanOffsetX;
		SecondController->Y = SecondCtrlPos[1] + moveZ_R + HMDPosY + g_LeanOffsetY;
		SecondController->Z = SecondCtrlPos[2] + RightYOffset - 0.15;

		FirstController->Yaw = OffsetYPR(cameraHMD.Yaw, LeftHandYaw);
		FirstController->Pitch = OffsetYPR(OffsetYPR(cameraHMD.Pitch, OffsetPitchByDefault), LeftHandPitch);
		SecondController->Yaw = OffsetYPR(OffsetYPR(cameraHMD.Yaw, SecondCtrlYPROffset[0]), RightHandYaw);
		SecondController->Pitch = OffsetYPR(OffsetYPR(OffsetYPR(cameraHMD.Pitch, OffsetPitchByDefault), SecondCtrlYPROffset[1]), RightHandPitch);
	}

	if (ControllerType == "oculus_touch") {
		FirstController->Z += OculusHeight;
		SecondController->Z += OculusHeight;
		FirstController->X -= OculusSeparation * cos(yawRad);
		FirstController->Y += OculusSeparation * sin(yawRad);
		SecondController->X += OculusSeparation * cos(yawRad);
		SecondController->Y -= OculusSeparation * sin(yawRad);
	}

	if (CheckButton(KEY_ID_LEFT_GRIP, Btn_LeftGrip)) FirstController->Buttons |= GRIP_BTN;
	if (CheckButton(KEY_ID_LEFT_MENU, Btn_LeftMenu)) FirstController->Buttons |= MENU_BTN;
	float targetTrigL = CheckButton(KEY_ID_LEFT_TRIGGER, Btn_LeftTrigger) ? 1.0f : 0.0f;
	SmoothTriggerL += (targetTrigL - SmoothTriggerL) * 15.0f * dt;
	if (SmoothTriggerL < 0.01f) SmoothTriggerL = 0.0f;
	if (SmoothTriggerL > 0.99f) SmoothTriggerL = 1.0f;
	FirstController->Trigger = SmoothTriggerL;
	if (CheckButton(KEY_ID_LEFT_SYSTEM, Btn_LeftSystem)) FirstController->Buttons |= SYS_BTN;

	// --- LEFT TRACKPAD (Virtual Thumbstick) ---
    float targetX_L = 0.0f, targetY_L = 0.0f;
    if (CheckButton(KEY_ID_UP, Btn_Up))       targetY_L =  1.0f;
    if (CheckButton(KEY_ID_DOWN, Btn_Down))   targetY_L = -1.0f;
    if (CheckButton(KEY_ID_LEFT, Btn_Left))   targetX_L = -1.0f;
    if (CheckButton(KEY_ID_RIGHT, Btn_Right)) targetX_L =  1.0f;

    bool distanceActive = CheckButton(KEY_ID_TRACKPAD_DISTANCE, Btn_TrackpadDistance);
    
    // Calculate magnitude for diagonal movement normalization
    float magnitude_L = sqrt(targetX_L*targetX_L + targetY_L*targetY_L);
    if (magnitude_L > 0.0f) {
        float cap = distanceActive ? TrackpadFull : TrackpadHalf;
        targetX_L = (targetX_L / magnitude_L) * cap;
        targetY_L = (targetY_L / magnitude_L) * cap;
    }

    // Smoothly glide the axes back to center when released
    CurrentAxisX_L += (targetX_L - CurrentAxisX_L) * 15.0f * dt;
    CurrentAxisY_L += (targetY_L - CurrentAxisY_L) * 15.0f * dt;

    if (std::abs(CurrentAxisX_L) < 0.01f) CurrentAxisX_L = 0.0f;
    if (std::abs(CurrentAxisY_L) < 0.01f) CurrentAxisY_L = 0.0f;

    if (CurrentAxisX_L != 0.0f || CurrentAxisY_L != 0.0f) {
        FirstController->AxisX = CurrentAxisX_L;
        FirstController->AxisY = CurrentAxisY_L;
        FirstController->Buttons |= THUMB_BTN;
    }
    
    if (CheckButton(KEY_ID_LEFT_TRACKPAD_CLICK, Btn_LeftTrackpadClick)) {
        FirstController->Buttons |= THUMB_BTN; 
    }

	// --- RIGHT TRACKPAD (Virtual Thumbstick) ---
    float targetX_R = 0.0f, targetY_R = 0.0f;
    if (CheckButton(KEY_ID_TURN_LEFT, Btn_TurnLeft))           targetX_R = -1.0f;
    if (CheckButton(KEY_ID_TURN_RIGHT, Btn_TurnRight))         targetX_R =  1.0f;
    if (CheckButton(KEY_ID_TELEPORT, Btn_Teleport))            targetY_R = -1.0f;
    if (CheckButton(KEY_ID_CHANGE_WEAPON, Btn_ChangeWeapon))   targetY_R =  1.0f;

    float magnitude_R = sqrt(targetX_R*targetX_R + targetY_R*targetY_R);
    if (magnitude_R > 0.0f) {
        float cap = distanceActive ? TrackpadFull : TrackpadHalf;
        targetX_R = (targetX_R / magnitude_R) * cap;
        targetY_R = (targetY_R / magnitude_R) * cap;
    }

    CurrentAxisX_R += (targetX_R - CurrentAxisX_R) * 15.0f * dt;
    CurrentAxisY_R += (targetY_R - CurrentAxisY_R) * 15.0f * dt;

    if (std::abs(CurrentAxisX_R) < 0.01f) CurrentAxisX_R = 0.0f;
    if (std::abs(CurrentAxisY_R) < 0.01f) CurrentAxisY_R = 0.0f;

    if (CurrentAxisX_R != 0.0f || CurrentAxisY_R != 0.0f) {
        SecondController->AxisX = CurrentAxisX_R;
        SecondController->AxisY = CurrentAxisY_R;
        SecondController->Buttons |= THUMB_BTN;
    }
    
    if (CheckButton(KEY_ID_RIGHT_TRACKPAD_CLICK, Btn_RightTrackpadClick)) {
        SecondController->Buttons |= THUMB_BTN;
    }

	if (CheckButton(KEY_ID_RIGHT_MENU, Btn_RightMenu)) SecondController->Buttons |= MENU_BTN;
	if (CheckButton(KEY_ID_RIGHT_GRIP, Btn_RightGrip)) SecondController->Buttons |= GRIP_BTN;
	if (CheckButton(KEY_ID_RIGHT_SYSTEM, Btn_RightSystem)) SecondController->Buttons |= SYS_BTN;
	float targetTrigR = CheckButton(KEY_ID_RIGHT_TRIGGER, Btn_RightTrigger) ? 1.0f : 0.0f;
	SmoothTriggerR += (targetTrigR - SmoothTriggerR) * 15.0f * dt;
	if (SmoothTriggerR < 0.01f) SmoothTriggerR = 0.0f;
	if (SmoothTriggerR > 0.99f) SmoothTriggerR = 1.0f;
	SecondController->Trigger = SmoothTriggerR;

	if (ControllerType == "oculus_touch") {
		FirstController->ButtonA = CheckButton(KEY_ID_LEFT_A, Btn_LeftA);
		FirstController->ButtonB = CheckButton(KEY_ID_LEFT_B, Btn_LeftB);
		SecondController->ButtonA = CheckButton(KEY_ID_RIGHT_A, Btn_RightA);
		SecondController->ButtonB = CheckButton(KEY_ID_RIGHT_B, Btn_RightB);

		FirstController->GripValue = CheckButton(KEY_ID_LEFT_GRIP, Btn_LeftGrip) ? 1.0f : 0.0f;
		SecondController->GripValue = CheckButton(KEY_ID_RIGHT_GRIP, Btn_RightGrip) ? 1.0f : 0.0f;
	}

	if (HeadsetWindowFocused == false)
	{
		FirstController->Buttons = 0; FirstController->Trigger = 0;
		FirstController->AxisX = 0; FirstController->AxisY = 0;
		FirstController->ButtonA = FirstController->ButtonB = false;
		FirstController->GripValue = 0.0f;

		SecondController->Buttons = 0; SecondController->Trigger = 0;
		SecondController->AxisX = 0; SecondController->AxisY = 0;
		SecondController->ButtonA = SecondController->ButtonB = false;
		SecondController->GripValue = 0.0f;
	}

	if (!FPSModeActive) {
		FirstController->Yaw = OffsetYPR(FirstController->Yaw, LeftHandRotOffset[0]);
		FirstController->Pitch = OffsetYPR(FirstController->Pitch, LeftHandRotOffset[1]);
		FirstController->Roll = OffsetYPR(FirstController->Roll, LeftHandRotOffset[2]);
		SecondController->Yaw = OffsetYPR(SecondController->Yaw, RightHandRotOffset[0]);
		SecondController->Pitch = OffsetYPR(SecondController->Pitch, RightHandRotOffset[1]);
		SecondController->Roll = OffsetYPR(SecondController->Roll, RightHandRotOffset[2]);

		FirstController->Yaw += SeqRotL[0]; FirstController->Pitch += SeqRotL[1]; FirstController->Roll += SeqRotL[2];
		SecondController->Yaw += SeqRotR[0]; SecondController->Pitch += SeqRotR[1]; SecondController->Roll += SeqRotR[2];

		FirstController->Yaw = FirstController->Yaw * -1; FirstController->Pitch = FirstController->Pitch * -1;
		SecondController->Yaw = SecondController->Yaw * -1; SecondController->Pitch = SecondController->Pitch * -1;
	}

	if (AutoSwingActive && !FPSModeActive) {
		ArmSwingTime += dt * ArmSwingSpeed;
		double swingForward = sin(ArmSwingTime) * ArmSwingDistance;
		double swingX = swingForward * sin(yawRad);
		double swingY = swingForward * cos(yawRad);

		FirstController->X += swingX;
		FirstController->Y += swingY;
		SecondController->X -= swingX;
		SecondController->Y -= swingY;
	}

	// --- PARALLEL GESTURE ENGINE ---
	if (HeadsetWindowFocused) {
		int currentPressed = 0;
		if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_5)) currentPressed = 1;
		else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_6)) currentPressed = 2;
		else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_7)) currentPressed = 3;
		else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_8)) currentPressed = 4;
		else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_9)) currentPressed = 5;
		else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_0)) currentPressed = 6;
		else if (IsKeyDown(KEY_ID_GESTURE_MINUS)) currentPressed = 7;
		else if (IsKeyDown(KEY_ID_GESTURE_EQUALS)) currentPressed = 8;

		bool justPressed = (currentPressed != 0 && currentPressed != PrevPressedGesture);
		PrevPressedGesture = currentPressed;

		if (justPressed) {
			if (currentPressed == g_ActiveGestureKey && (NextStepIndex > 0 || !RunningSteps.empty())) {
				RunningSteps.clear();
				ActiveSequence.clear();
				NextStepIndex = 0;
				g_ActiveGestureKey = 0;

				SeqOffsetL[0] = SeqOffsetL[1] = SeqOffsetL[2] = 0;
				SeqOffsetR[0] = SeqOffsetR[1] = SeqOffsetR[2] = 0;
				SeqRotL[0] = SeqRotL[1] = SeqRotL[2] = 0;
				SeqRotR[0] = SeqRotR[1] = SeqRotR[2] = 0;

				SeqGripL = SeqGripR = SeqTrigL = SeqTrigR = -1;
				SeqMenuL = SeqMenuR = SeqSysL = SeqSysR = -1;
				SeqTrackpadCL = SeqTrackpadCR = SeqTrackpadUL = SeqTrackpadUR = -1;
				SeqTrackpadDL = SeqTrackpadDR = SeqTrackpadLL = SeqTrackpadLR = -1;
				SeqTrackpadRL = SeqTrackpadRR = SeqBtnAL = SeqBtnAR = SeqBtnBL = SeqBtnBR = -1;

				vr::VRDriverLog()->Log("FlatVR: Gesture Cancelled\n");
			} else {
				RunningSteps.clear();
				ActiveSequence.clear();
				NextStepIndex = 0;

				std::string slot = "";
				if (currentPressed == 1) slot = "5";
				else if (currentPressed == 2) slot = "6";
				else if (currentPressed == 3) slot = "7";
				else if (currentPressed == 4) slot = "8";
				else if (currentPressed == 5) slot = "9";
				else if (currentPressed == 6) slot = "0";
				else if (currentPressed == 7) slot = "minus";
				else if (currentPressed == 8) slot = "equals";

				if (!slot.empty()) {
					std::string prof = g_DriverFolderStr + "/Resources/Custom_Gestures/" + g_ActiveProfileName + "/" + slot + ".gesture";
					std::string def = g_DriverFolderStr + "/Resources/Custom_Gestures/Default/" + slot + ".gesture";
					std::string old = g_DriverFolderStr + "/Resources/Custom_Gestures/" + slot + ".gesture";

					std::string path = old;
					if (GetFileAttributesA(prof.c_str()) != INVALID_FILE_ATTRIBUTES) path = prof;
					else if (GetFileAttributesA(def.c_str()) != INVALID_FILE_ATTRIBUTES) path = def;

					LoadGestureSequence(path);

					if (!ActiveSequence.empty()) {
						g_ActiveGestureKey = currentPressed;

						StartSeqOffsetL[0] = SeqOffsetL[0]; StartSeqOffsetL[1] = SeqOffsetL[1]; StartSeqOffsetL[2] = SeqOffsetL[2];
						StartSeqOffsetR[0] = SeqOffsetR[0]; StartSeqOffsetR[1] = SeqOffsetR[1]; StartSeqOffsetR[2] = SeqOffsetR[2];
						StartSeqRotL[0] = SeqRotL[0]; StartSeqRotL[1] = SeqRotL[1]; StartSeqRotL[2] = SeqRotL[2];
						StartSeqRotR[0] = SeqRotR[0]; StartSeqRotR[1] = SeqRotR[1]; StartSeqRotR[2] = SeqRotR[2];

						NextStepIndex = 0;
						RunningSteps.push_back({ NextStepIndex, 0.0 });
						NextStepIndex++;
						while (NextStepIndex < ActiveSequence.size() && ActiveSequence[NextStepIndex].isParallel) {
							RunningSteps.push_back({ NextStepIndex, 0.0 });
							NextStepIndex++;
						}
					}
				}
			}
		}
	}

	if (!RunningSteps.empty()) {
		for (auto& rs : RunningSteps) {
			GestureStep& step = ActiveSequence[rs.index];

			rs.timer += dt;

			if (rs.timer < step.delay) {
				continue;
			}

			if (!rs.initialized) {
                if (step.isHmdRelative) {
                    double effYaw = DegToRad(cameraHMD.Yaw);
                    double effPitch = DegToRad(cameraHMD.Pitch);
                    double rx =  cos(effYaw), ry = -sin(effYaw), rz =  0;
                    double fx = -sin(effYaw) * cos(effPitch), fy = -cos(effYaw) * cos(effPitch), fz =  sin(effPitch);
                    double ux =  sin(effYaw) * sin(effPitch), uy =  cos(effYaw) * sin(effPitch), uz =  cos(effPitch);

                    double relX = FirstController->X - HMDPosX;
                    double relY = FirstController->Y - HMDPosY;
                    double relZ = FirstController->Z - (HMDPosZ - CrouchOffsetZ);
                    rs.initOffsetL[0] = relX*rx + relY*ry + relZ*rz;
                    rs.initOffsetL[1] = relX*fx + relY*fy + relZ*fz;
                    rs.initOffsetL[2] = relX*ux + relY*uy + relZ*uz;

                    relX = SecondController->X - HMDPosX;
                    relY = SecondController->Y - HMDPosY;
                    relZ = SecondController->Z - (HMDPosZ - CrouchOffsetZ);
                    rs.initOffsetR[0] = relX*rx + relY*ry + relZ*rz;
                    rs.initOffsetR[1] = relX*fx + relY*fy + relZ*fz;
                    rs.initOffsetR[2] = relX*ux + relY*uy + relZ*uz;
                } else {
                    rs.initOffsetL[0] = SeqOffsetL[0]; rs.initOffsetL[1] = SeqOffsetL[1]; rs.initOffsetL[2] = SeqOffsetL[2];
                    rs.initOffsetR[0] = SeqOffsetR[0]; rs.initOffsetR[1] = SeqOffsetR[1]; rs.initOffsetR[2] = SeqOffsetR[2];
                }
				rs.initRotL[0] = SeqRotL[0]; rs.initRotL[1] = SeqRotL[1]; rs.initRotL[2] = SeqRotL[2];
				rs.initRotR[0] = SeqRotR[0]; rs.initRotR[1] = SeqRotR[1]; rs.initRotR[2] = SeqRotR[2];
				rs.initialized = true;
			}

			double activeTime = rs.timer - step.delay;
			double progress = activeTime / step.duration;
			if (progress > 1.0) progress = 1.0;

			if (step.command == "MOVE") {
				if (step.target == "L" || step.target == "B") {
					if (step.axis == "X") SeqOffsetL[0] = rs.initOffsetL[0] + (step.value * progress);
					if (step.axis == "Y") SeqOffsetL[1] = rs.initOffsetL[1] + (step.value * progress);
					if (step.axis == "Z") SeqOffsetL[2] = rs.initOffsetL[2] + (step.value * progress);
				}
				if (step.target == "R" || step.target == "B") {
					if (step.axis == "X") SeqOffsetR[0] = rs.initOffsetR[0] + (step.value * progress);
					if (step.axis == "Y") SeqOffsetR[1] = rs.initOffsetR[1] + (step.value * progress);
					if (step.axis == "Z") SeqOffsetR[2] = rs.initOffsetR[2] + (step.value * progress);
				}
			}
			else if (step.command == "ROTATE") {
				if (step.target == "L" || step.target == "B") {
					if (step.axis == "YAW") SeqRotL[0] = rs.initRotL[0] + (step.value * progress);
					if (step.axis == "PITCH") SeqRotL[1] = rs.initRotL[1] + (step.value * progress);
					if (step.axis == "ROLL") SeqRotL[2] = rs.initRotL[2] + (step.value * progress);
				}
				if (step.target == "R" || step.target == "B") {
					if (step.axis == "YAW") SeqRotR[0] = rs.initRotR[0] + (step.value * progress);
					if (step.axis == "PITCH") SeqRotR[1] = rs.initRotR[1] + (step.value * progress);
					if (step.axis == "ROLL") SeqRotR[2] = rs.initRotR[2] + (step.value * progress);
				}
			}
			else if (step.command == "RETURN") {
				SeqOffsetL[0] = rs.initOffsetL[0] + (StartSeqOffsetL[0] - rs.initOffsetL[0]) * progress;
				SeqOffsetL[1] = rs.initOffsetL[1] + (StartSeqOffsetL[1] - rs.initOffsetL[1]) * progress;
				SeqOffsetL[2] = rs.initOffsetL[2] + (StartSeqOffsetL[2] - rs.initOffsetL[2]) * progress;
				SeqRotL[0] = rs.initRotL[0] + (StartSeqRotL[0] - rs.initRotL[0]) * progress;
				SeqRotL[1] = rs.initRotL[1] + (StartSeqRotL[1] - rs.initRotL[1]) * progress;
				SeqRotL[2] = rs.initRotL[2] + (StartSeqRotL[2] - rs.initRotL[2]) * progress;
				SeqOffsetR[0] = rs.initOffsetR[0] + (StartSeqOffsetR[0] - rs.initOffsetR[0]) * progress;
				SeqOffsetR[1] = rs.initOffsetR[1] + (StartSeqOffsetR[1] - rs.initOffsetR[1]) * progress;
				SeqOffsetR[2] = rs.initOffsetR[2] + (StartSeqOffsetR[2] - rs.initOffsetR[2]) * progress;
				SeqRotR[0] = rs.initRotR[0] + (StartSeqRotR[0] - rs.initRotR[0]) * progress;
				SeqRotR[1] = rs.initRotR[1] + (StartSeqRotR[1] - rs.initRotR[1]) * progress;
				SeqRotR[2] = rs.initRotR[2] + (StartSeqRotR[2] - rs.initRotR[2]) * progress;
			}
			else if (step.command == "POSE") {
				if (step.hasPoseL) {
					SeqOffsetL[0] = rs.initOffsetL[0] + (step.poseL[0] - rs.initOffsetL[0]) * progress;
					SeqOffsetL[1] = rs.initOffsetL[1] + (step.poseL[1] - rs.initOffsetL[1]) * progress;
					SeqOffsetL[2] = rs.initOffsetL[2] + (step.poseL[2] - rs.initOffsetL[2]) * progress;
					if (!step.ignoreRot) { // <--- ADD THIS CHECK
						SeqRotL[0] = rs.initRotL[0] + (step.poseL[3] - rs.initRotL[0]) * progress;
						SeqRotL[1] = rs.initRotL[1] + (step.poseL[4] - rs.initRotL[1]) * progress;
						SeqRotL[2] = rs.initRotL[2] + (step.poseL[5] - rs.initRotL[2]) * progress;
					}
				}
				if (step.hasPoseR) {
					SeqOffsetR[0] = rs.initOffsetR[0] + (step.poseR[0] - rs.initOffsetR[0]) * progress;
					SeqOffsetR[1] = rs.initOffsetR[1] + (step.poseR[1] - rs.initOffsetR[1]) * progress;
					SeqOffsetR[2] = rs.initOffsetR[2] + (step.poseR[2] - rs.initOffsetR[2]) * progress;
					if (!step.ignoreRot) { // <--- ADD THIS CHECK
						SeqRotR[0] = rs.initRotR[0] + (step.poseR[3] - rs.initRotR[0]) * progress;
						SeqRotR[1] = rs.initRotR[1] + (step.poseR[4] - rs.initRotR[1]) * progress;
						SeqRotR[2] = rs.initRotR[2] + (step.poseR[5] - rs.initRotR[2]) * progress;
					}
				}
			}
			else if (step.command == "SHAKE") {
				int axisIdx = 0;
				if (step.axis == "X") axisIdx = 0;
				else if (step.axis == "Y") axisIdx = 1;
				else if (step.axis == "Z") axisIdx = 2;

				if (step.target == "L" || step.target == "B") {
					ShakeTimerL = step.duration;
					ShakeIntensityL[axisIdx] = step.value;
				}
				if (step.target == "R" || step.target == "B") {
					ShakeTimerR = step.duration;
					ShakeIntensityR[axisIdx] = step.value;
				}
			}
			else if (step.command == "BUTTON") {
				double btnState = step.value;
				if (step.target == "L" || step.target == "B") {
					if (step.axis == "GRIP") SeqGripL = btnState;
					else if (step.axis == "TRIG") SeqTrigL = btnState;
					else if (step.axis == "MENU") SeqMenuL = btnState;
					else if (step.axis == "SYSTEM") SeqSysL = btnState;
					else if (step.axis == "TRACKPAD_C") SeqTrackpadCL = btnState;
					else if (step.axis == "TRACKPAD_R") SeqTrackpadRL = btnState;
					else if (step.axis == "TRACKPAD_L") SeqTrackpadLL = btnState;
					else if (step.axis == "TRACKPAD_U") SeqTrackpadUL = btnState;
					else if (step.axis == "TRACKPAD_D") SeqTrackpadDL = btnState;
					else if (step.axis == "BTN_A") SeqBtnAL = btnState;
					else if (step.axis == "BTN_B") SeqBtnBL = btnState;
				}
				if (step.target == "R" || step.target == "B") {
					if (step.axis == "GRIP") SeqGripR = btnState;
					else if (step.axis == "TRIG") SeqTrigR = btnState;
					else if (step.axis == "MENU") SeqMenuR = btnState;
					else if (step.axis == "SYSTEM") SeqSysR = btnState;
					else if (step.axis == "TRACKPAD_C") SeqTrackpadCR = btnState;
					else if (step.axis == "TRACKPAD_R") SeqTrackpadRR = btnState;
					else if (step.axis == "TRACKPAD_L") SeqTrackpadLR = btnState;
					else if (step.axis == "TRACKPAD_U") SeqTrackpadUR = btnState;
					else if (step.axis == "TRACKPAD_D") SeqTrackpadDR = btnState;
					else if (step.axis == "BTN_A") SeqBtnAR = btnState;
					else if (step.axis == "BTN_B") SeqBtnBR = btnState;
				}
			}
		}

		RunningSteps.erase(std::remove_if(RunningSteps.begin(), RunningSteps.end(),
			[&](const ActiveStep& rs) {
				return rs.timer >= (ActiveSequence[rs.index].delay + ActiveSequence[rs.index].duration);
			}), RunningSteps.end());

		if (RunningSteps.empty() && NextStepIndex < ActiveSequence.size()) {
			RunningSteps.push_back({ NextStepIndex, 0.0 });
			NextStepIndex++;
			while (NextStepIndex < ActiveSequence.size() && ActiveSequence[NextStepIndex].isParallel) {
				RunningSteps.push_back({ NextStepIndex, 0.0 });
				NextStepIndex++;
			}
		}

		if (RunningSteps.empty() && NextStepIndex >= ActiveSequence.size()) {
			if (g_GestureLoop) {
				NextStepIndex = 0;
				RunningSteps.push_back({ NextStepIndex, 0.0 });
				NextStepIndex++;
				while (NextStepIndex < ActiveSequence.size() && ActiveSequence[NextStepIndex].isParallel) {
					RunningSteps.push_back({ NextStepIndex, 0.0 });
					NextStepIndex++;
				}
			} else {
				NextStepIndex = 0;
				ActiveSequence.clear();
				g_ActiveGestureKey = 0;
				SeqGripL = SeqGripR = SeqTrigL = SeqTrigR = -1;
				SeqMenuL = SeqMenuR = SeqSysL = SeqSysR = -1;
				SeqTrackpadCL = SeqTrackpadCR = SeqTrackpadUL = SeqTrackpadUR = -1;
				SeqTrackpadDL = SeqTrackpadDR = SeqTrackpadLL = SeqTrackpadLR = -1;
				SeqTrackpadRL = SeqTrackpadRR = SeqBtnAL = SeqBtnAR = SeqBtnBL = SeqBtnBR = -1;
			}
		}
	}

	if (!FPSModeActive) {
		double gestMoveX_L = SeqOffsetL[0] * cos(yawRad) + SeqOffsetL[1] * sin(yawRad);
		double gestMoveY_L = -SeqOffsetL[0] * sin(yawRad) + SeqOffsetL[1] * cos(yawRad);
		FirstController->X += gestMoveX_L;
		FirstController->Y += gestMoveY_L;
		FirstController->Z += SeqOffsetL[2];

		double gestMoveX_R = SeqOffsetR[0] * cos(yawRad) + SeqOffsetR[1] * sin(yawRad);
		double gestMoveY_R = -SeqOffsetR[0] * sin(yawRad) + SeqOffsetR[1] * cos(yawRad);
		SecondController->X += gestMoveX_R;
		SecondController->Y += gestMoveY_R;
		SecondController->Z += SeqOffsetR[2];
	}

    // --- GLOBAL HMD-RELATIVE OVERRIDE ---
    bool activePoseL = false;
    bool activePoseR = false;

    for (const auto& rs : RunningSteps) {
        if (ActiveSequence[rs.index].command == "POSE") {
            if (ActiveSequence[rs.index].hasPoseL) activePoseL = true;
            if (ActiveSequence[rs.index].hasPoseR) activePoseR = true;
        }
    }

    if (activePoseL || activePoseR) {
        double effYaw = DegToRad(cameraHMD.Yaw);
        double effPitch = DegToRad(cameraHMD.Pitch);
        
        double rx =  cos(effYaw);
        double ry = -sin(effYaw);
        double rz =  0;
        
        double fx = -sin(effYaw) * cos(effPitch);
        double fy = -cos(effYaw) * cos(effPitch);
        double fz =  sin(effPitch);
        
        double ux =  sin(effYaw) * sin(effPitch);
        double uy =  cos(effYaw) * sin(effPitch);
        double uz =  cos(effPitch);

        if (activePoseL) {
            FirstController->X = HMDPosX + rx*SeqOffsetL[0] + fx*SeqOffsetL[1] + ux*SeqOffsetL[2];
            FirstController->Y = HMDPosY + ry*SeqOffsetL[0] + fy*SeqOffsetL[1] + uy*SeqOffsetL[2];
            FirstController->Z = (HMDPosZ - CrouchOffsetZ) + rz*SeqOffsetL[0] + fz*SeqOffsetL[1] + uz*SeqOffsetL[2];
        }
        if (activePoseR) {
            SecondController->X = HMDPosX + rx*SeqOffsetR[0] + fx*SeqOffsetR[1] + ux*SeqOffsetR[2];
            SecondController->Y = HMDPosY + ry*SeqOffsetR[0] + fy*SeqOffsetR[1] + uy*SeqOffsetR[2];
            SecondController->Z = (HMDPosZ - CrouchOffsetZ) + rz*SeqOffsetR[0] + fz*SeqOffsetR[1] + uz*SeqOffsetR[2];
        }
    } else if (!FPSModeActive) {
        // Standard 2D Yaw-relative positioning ONLY runs if no Pose is active
        double gestMoveX_L = SeqOffsetL[0] * cos(yawRad) + SeqOffsetL[1] * sin(yawRad);
        double gestMoveY_L = -SeqOffsetL[0] * sin(yawRad) + SeqOffsetL[1] * cos(yawRad);
        FirstController->X += gestMoveX_L;
        FirstController->Y += gestMoveY_L;
        FirstController->Z += SeqOffsetL[2];

        double gestMoveX_R = SeqOffsetR[0] * cos(yawRad) + SeqOffsetR[1] * sin(yawRad);
        double gestMoveY_R = -SeqOffsetR[0] * sin(yawRad) + SeqOffsetR[1] * cos(yawRad);
        SecondController->X += gestMoveX_R;
        SecondController->Y += gestMoveY_R;
        SecondController->Z += SeqOffsetR[2];
    }
    // --- END GLOBAL HMD-RELATIVE OVERRIDE ---

	if (SeqGripL >= 0.0) {
		if (SeqGripL >= 0.5) FirstController->Buttons |= GRIP_BTN;
		else FirstController->Buttons &= ~GRIP_BTN;
		FirstController->GripValue = (float)SeqGripL;
	}
	if (SeqGripR >= 0.0) {
		if (SeqGripR >= 0.5) SecondController->Buttons |= GRIP_BTN;
		else SecondController->Buttons &= ~GRIP_BTN;
		SecondController->GripValue = (float)SeqGripR;
	}

	if (SeqTrigL >= 0.0) FirstController->Trigger = (float)SeqTrigL;
	if (SeqTrigR >= 0.0) SecondController->Trigger = (float)SeqTrigR;

	if (SeqMenuL >= 0.0) { if (SeqMenuL >= 0.5) FirstController->Buttons |= MENU_BTN; else FirstController->Buttons &= ~MENU_BTN; }
	if (SeqSysL >= 0.0) { if (SeqSysL >= 0.5) FirstController->Buttons |= SYS_BTN; else FirstController->Buttons &= ~SYS_BTN; }
	if (SeqMenuR >= 0.0) { if (SeqMenuR >= 0.5) SecondController->Buttons |= MENU_BTN; else SecondController->Buttons &= ~MENU_BTN; }
	if (SeqSysR >= 0.0) { if (SeqSysR >= 0.5) SecondController->Buttons |= SYS_BTN; else SecondController->Buttons &= ~SYS_BTN; }

	if (SeqBtnAL >= 0.0) FirstController->ButtonA = (SeqBtnAL >= 0.5);
	if (SeqBtnBL >= 0.0) FirstController->ButtonB = (SeqBtnBL >= 0.5);
	if (SeqBtnAR >= 0.0) SecondController->ButtonA = (SeqBtnAR >= 0.5);
	if (SeqBtnBR >= 0.0) SecondController->ButtonB = (SeqBtnBR >= 0.5);

	if (SeqTrackpadUL >= 0.0 || SeqTrackpadDL >= 0.0) {
		float yVal = 0.0f;
		if (SeqTrackpadUL >= 0.0) yVal += (float)SeqTrackpadUL;
		if (SeqTrackpadDL >= 0.0) yVal -= (float)SeqTrackpadDL;
		FirstController->AxisY = yVal;
	}
	if (SeqTrackpadRL >= 0.0 || SeqTrackpadLL >= 0.0) {
		float xVal = 0.0f;
		if (SeqTrackpadRL >= 0.0) xVal += (float)SeqTrackpadRL;
		if (SeqTrackpadLL >= 0.0) xVal -= (float)SeqTrackpadLL;
		FirstController->AxisX = xVal;
	}
	if (SeqTrackpadUR >= 0.0 || SeqTrackpadDR >= 0.0) {
		float yVal = 0.0f;
		if (SeqTrackpadUR >= 0.0) yVal += (float)SeqTrackpadUR;
		if (SeqTrackpadDR >= 0.0) yVal -= (float)SeqTrackpadDR;
		SecondController->AxisY = yVal;
	}
	if (SeqTrackpadRR >= 0.0 || SeqTrackpadLR >= 0.0) {
		float xVal = 0.0f;
		if (SeqTrackpadRR >= 0.0) xVal += (float)SeqTrackpadRR;
		if (SeqTrackpadLR >= 0.0) xVal -= (float)SeqTrackpadLR;
		SecondController->AxisX = xVal;
	}

	if (SeqTrackpadCL >= 0.0) { if (SeqTrackpadCL >= 0.5) FirstController->Buttons |= THUMB_BTN; else if (SeqTrackpadUL < 0.5 && SeqTrackpadDL < 0.5 && SeqTrackpadLL < 0.5 && SeqTrackpadRL < 0.5) FirstController->Buttons &= ~THUMB_BTN; }
	if (SeqTrackpadCR >= 0.0) { if (SeqTrackpadCR >= 0.5) SecondController->Buttons |= THUMB_BTN; else if (SeqTrackpadUR < 0.5 && SeqTrackpadDR < 0.5 && SeqTrackpadLR < 0.5 && SeqTrackpadRR < 0.5) SecondController->Buttons &= ~THUMB_BTN; }

	if (LeftHandLocked) {
		FirstController->X = LockedLeftPos[0];
		FirstController->Y = LockedLeftPos[1];
		FirstController->Z = LockedLeftPos[2];
		if (!LeftHandIsRotating) {
			FirstController->Yaw = LockedLeftYaw;
			FirstController->Pitch = LockedLeftPitch;
			FirstController->Roll = LockedLeftRoll;
			FirstController->UseQuat = LockedLeftUseQuat;
			FirstController->QW = LockedLeftQ[0]; FirstController->QX = LockedLeftQ[1]; FirstController->QY = LockedLeftQ[2]; FirstController->QZ = LockedLeftQ[3];
		}
	}
	if (RightHandLocked) {
		SecondController->X = LockedRightPos[0];
		SecondController->Y = LockedRightPos[1];
		SecondController->Z = LockedRightPos[2];
		if (!RightHandIsRotating) {
			SecondController->Yaw = LockedRightYaw;
			SecondController->Pitch = LockedRightPitch;
			SecondController->Roll = LockedRightRoll;
			SecondController->UseQuat = LockedRightUseQuat;
			SecondController->QW = LockedRightQ[0]; SecondController->QX = LockedRightQ[1]; SecondController->QY = LockedRightQ[2]; SecondController->QZ = LockedRightQ[3];
		}
	}
}

// --- INI CONFIGURATION LOADER ---
void LoadConfig() {
    if (g_IniPathStr.empty()) return;
    CIniReader IniFile((char*)g_IniPathStr.c_str());

    ControllerType = IniFile.ReadString("Main", "ControllerType", "vive_controller");
    HMDConnected = IniFile.ReadBoolean("Main", "HMD", true);
    VRMode = IniFile.ReadBoolean("Main", "VRMode", false);
    VRAimingAngleOffset = IniFile.ReadFloat("Main", "VRAimingAngleOffset", 0);

    ScrollPushPullSpeed = IniFile.ReadFloat("Main", "ScrollPushPullSpeed", 0.0005f);
    HandMoveSensitivity = IniFile.ReadFloat("Main", "HandMoveSensitivity", 0.5f);
    RotationSpeed = IniFile.ReadFloat("Main", "RotationSpeed", 0.05f);
    FineTuneMultiplier = IniFile.ReadFloat("Main", "FineTuneMultiplier", 0.1f);
    mouseSensetiveYaw = IniFile.ReadFloat("Main", "AxisX", 0.05f);
    mouseSensetivePitch = IniFile.ReadFloat("Main", "AxisY", 0.04f);
    ZPosSensetive = IniFile.ReadFloat("Main", "AxisZ", 0.009f);
    HandCenterOffsetX = IniFile.ReadFloat("Main", "HandCenterOffsetX", 0.1f);
	PositionSmoothness = IniFile.ReadFloat("Main", "PositionSmoothness", 0.15f);
	NeckModelEnabled = IniFile.ReadBoolean("Main", "NeckModel", true);
    TrackpadHalf = IniFile.ReadFloat("Main", "TrackpadHalf", 0.5f);
    TrackpadFull = IniFile.ReadFloat("Main", "TrackpadFull", 1.0f);

    FPS_HandCenterOffsetX = IniFile.ReadFloat("FPS_Mode", "FPS_HandCenterOffsetX", 0.0f);
    FPS_HandCenterOffsetZ = IniFile.ReadFloat("FPS_Mode", "FPS_HandCenterOffsetZ", 0.0f);
    FPS_GunOffsetX = IniFile.ReadFloat("FPS_Mode", "GunOffsetX", 0.0f);
    FPS_GunOffsetY = IniFile.ReadFloat("FPS_Mode", "GunOffsetY", -0.1f);
    FPS_GunOffsetZ = IniFile.ReadFloat("FPS_Mode", "GunOffsetZ", 0.3f);
    FPS_LeftOffsetX = IniFile.ReadFloat("FPS_Mode", "LeftOffsetX", -0.15f);
    FPS_LeftOffsetY = IniFile.ReadFloat("FPS_Mode", "LeftOffsetY", -0.15f);
    FPS_LeftOffsetZ = IniFile.ReadFloat("FPS_Mode", "LeftOffsetZ", 0.15f);
    FPS_GunZeroYaw = IniFile.ReadFloat("FPS_Mode", "GunZeroYaw", 0.0f);
    FPS_GunZeroPitch = IniFile.ReadFloat("FPS_Mode", "GunZeroPitch", 0.0f);
    FPS_GunZeroRoll = IniFile.ReadFloat("FPS_Mode", "GunZeroRoll", 0.0f);
    FPS_LeftZeroYaw = IniFile.ReadFloat("FPS_Mode", "LeftZeroYaw", 0.0f);
    FPS_LeftZeroPitch = IniFile.ReadFloat("FPS_Mode", "LeftZeroPitch", 0.0f);
    FPS_LeftZeroRoll = IniFile.ReadFloat("FPS_Mode", "LeftZeroRoll", 0.0f);

    FPS_YawSign   = IniFile.ReadBoolean("FPS_Mode", "InvertYaw", false)  ? 1.0 : -1.0; // swapped: false = the "good" yaw, true = flipped
    FPS_PitchSign = IniFile.ReadBoolean("FPS_Mode", "InvertPitch", true) ? -1.0 : 1.0;
    PumpDuration = IniFile.ReadFloat("FPS_Mode", "PumpDuration", 0.35f);
    PumpDistance = IniFile.ReadFloat("FPS_Mode", "PumpDistance", 0.15f);

    bool tmp[256] = {};
    std::string swallowStr = IniFile.ReadString("SwallowKeys", "Keys", "");
    std::stringstream ss(swallowStr);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty()) {
            int code = KeyNameToKeyCode(token);
            if (code > 0 && code < 256) tmp[code] = true;
        }
    }

    // --- KEY REMAPPER ---
    g_KeyRemapper.clear();
    char remapBuffer[4096] = {0};
    GetPrivateProfileSectionA("KeyRemapper", remapBuffer, sizeof(remapBuffer), g_IniPathStr.c_str());
    char* p = remapBuffer;
    while (*p) {
        std::string line(p);
        size_t eq = line.find('=');
        if (eq != std::string::npos) {
            std::string fromStr = line.substr(0, eq);
            std::string toStr = line.substr(eq + 1);
            int fromCode = KeyNameToKeyCode(fromStr);
            int toCode = KeyNameToKeyCode(toStr);
            if (fromCode > 0 && toCode > 0) {
                g_KeyRemapper[fromCode] = toCode;
            }
        }
        p += strlen(p) + 1;
    }

    for (int i = 0; i < 256; ++i)
        g_IsDriverKey[i].store(tmp[i], std::memory_order_relaxed);

    KEY_ID_LEFT_TRIGGER = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_TRIGGER", "MOUSE-LEFT-BTN"));
    KEY_ID_LEFT_MENU = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_MENU", "TAB"));
    KEY_ID_LEFT_GRIP = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_GRIP", "Q"));
    KEY_ID_LEFT_SYSTEM = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_SYSTEM", "3"));
    KEY_ID_RIGHT_TRIGGER = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_TRIGGER", "MOUSE-RIGHT-BTN"));
    KEY_ID_RIGHT_MENU = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_MENU", "R"));
    KEY_ID_RIGHT_GRIP = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_GRIP", "E"));
    KEY_ID_RIGHT_SYSTEM = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_SYSTEM", "4"));
    KEY_ID_CHANGE_WEAPON = KeyNameToKeyCode(IniFile.ReadString("Keys", "CHANGE_WEAPON", "MOUSE-MIDDLE-BTN"));
    KEY_ID_TELEPORT = KeyNameToKeyCode(IniFile.ReadString("Keys", "TELEPORT", "SPACE"));
    KEY_ID_MOVE_CONTROLLERS_FORWARD = KeyNameToKeyCode(IniFile.ReadString("Keys", "MOVE_CONTROLLERS_FORWARD", "INSERT"));
    KEY_ID_MOVE_CONTROLLERS_BACK = KeyNameToKeyCode(IniFile.ReadString("Keys", "MOVE_CONTROLLERS_BACK", "HOME"));
    KEY_ID_PLAYER_RISE_HIGHER = KeyNameToKeyCode(IniFile.ReadString("Keys", "PLAYER_RISE_HIGHER", "PAGE-UP"));
    KEY_ID_PLAYER_RISE_LOWER = KeyNameToKeyCode(IniFile.ReadString("Keys", "PLAYER_RISE_LOWER", "PAGE-DOWN"));
    KEY_ID_PLAYER_RISE_RESET = KeyNameToKeyCode(IniFile.ReadString("Keys", "PLAYER_RISE_RESET", "NUMPAD-MINUS"));
    KEY_ID_CROUCH = KeyNameToKeyCode(IniFile.ReadString("Keys", "CROUCH", "CTRL"));
    KEY_ID_PRONE = KeyNameToKeyCode(IniFile.ReadString("Keys", "PRONE", "~"));
    KEY_ID_LEFT_CTRL_MOTION = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_CTRL_MOTION", "NONE"));
    KEY_ID_ROTATION_CONTROLLERS_RESET = KeyNameToKeyCode(IniFile.ReadString("Keys", "ROTATION_CONTROLLERS_RESET", "Y"));
	KEY_ID_SMOOTH_HAND_RESET = KeyNameToKeyCode(IniFile.ReadString("Keys", "SMOOTH_HAND_RESET", "ENTER"));
    KEY_ID_FINE_TUNE = KeyNameToKeyCode(IniFile.ReadString("Keys", "FINE_TUNE", "H"));
    KEY_ID_FPS_MODE_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "FPS_MODE_TOGGLE", "J"));
    KEY_ID_AUTO_SWING_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "AUTO_SWING_TOGGLE", "SHIFT"));
    KEY_ID_PUMP_ACTION = KeyNameToKeyCode(IniFile.ReadString("Keys", "PUMP_ACTION", "MOUSE-SIDE1-BTN"));
    KEY_ID_SNIPER_MODE = KeyNameToKeyCode(IniFile.ReadString("Keys", "SNIPER_MODE", "MOUSE-SIDE2-BTN"));

    KEY_ID_LEFT_HAND_LOCK = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_HAND_LOCK", "L"));
    KEY_ID_RIGHT_HAND_LOCK = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_HAND_LOCK", "SEMICOLON"));
	KEY_ID_LEFT_TRACKPAD_TOGGLE  = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_TOUCH_PRESS_TOGGLE",  "["));
	KEY_ID_RIGHT_TRACKPAD_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_TOUCH_PRESS_TOGGLE", "]"));

    KEY_ID_HMD_FORWARD = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_FORWARD", "NUMPAD8"));
    KEY_ID_HMD_BACK = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_BACK", "NUMPAD5"));
    KEY_ID_HMD_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_LEFT", "NUMPAD4"));
    KEY_ID_HMD_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_RIGHT", "NUMPAD6"));
	KEY_ID_LEAN_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEAN_LEFT", "NUMPAD7"));
    KEY_ID_LEAN_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEAN_RIGHT", "NUMPAD9"));
    KEY_ID_HMD_MOVE_RESET = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_MOVE_RESET", "U"));
    KEY_ID_FREELOOK = KeyNameToKeyCode(IniFile.ReadString("Keys", "FREELOOK", "ALT"));

    KEY_ID_LEFT_HAND_ROTATE = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_HAND_ROTATE", "X"));
    KEY_ID_RIGHT_HAND_ROTATE = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_HAND_ROTATE", "C"));
    KEY_ID_HAND_FORWARD = KeyNameToKeyCode(IniFile.ReadString("Keys", "HAND_FORWARD", "W"));
    KEY_ID_HAND_BACK = KeyNameToKeyCode(IniFile.ReadString("Keys", "HAND_BACK", "Z"));
    KEY_ID_HAND_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HAND_LEFT", "A"));
    KEY_ID_HAND_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HAND_RIGHT", "D"));
    KEY_ID_MOTION_MODE_MOVE_UP = KeyNameToKeyCode(IniFile.ReadString("Keys", "MOTION_MODE_MOVE_UP", "1"));
    KEY_ID_MOTION_MODE_MOVE_DOWN = KeyNameToKeyCode(IniFile.ReadString("Keys", "MOTION_MODE_MOVE_DOWN", "2"));
    KEY_ID_LEFT_TRACKPAD_CLICK = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_TRACKPAD_CLICK", "F"));
    KEY_ID_RIGHT_TRACKPAD_CLICK = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_TRACKPAD_CLICK", "V"));
    KEY_ID_TURN_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "TURN_LEFT", "B"));
    KEY_ID_TURN_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "TURN_RIGHT", "N"));
    KEY_ID_UP = KeyNameToKeyCode(IniFile.ReadString("Keys", "UP", "UP"));
    KEY_ID_DOWN = KeyNameToKeyCode(IniFile.ReadString("Keys", "DOWN", "DOWN"));
    KEY_ID_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT", "LEFT"));
    KEY_ID_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT", "RIGHT"));
    KEY_ID_TRACKPAD_DISTANCE = KeyNameToKeyCode(IniFile.ReadString("Keys", "TRACKPAD_DISTANCE_BTN", "Z"));

    KEY_ID_LEFT_A = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_A", "T"));
    KEY_ID_LEFT_B = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_B", "TAB"));
    KEY_ID_RIGHT_A = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_A", "G"));
    KEY_ID_RIGHT_B = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_B", "R"));

    KEY_ID_GESTURE_CUSTOM_5 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_5", "5"));
    KEY_ID_GESTURE_CUSTOM_6 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_6", "6"));
    KEY_ID_GESTURE_CUSTOM_7 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_7", "7"));
    KEY_ID_GESTURE_CUSTOM_8 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_8", "8"));
    KEY_ID_GESTURE_CUSTOM_9 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_9", "9"));
    KEY_ID_GESTURE_CUSTOM_0 = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_0", "0"));
    KEY_ID_GESTURE_MINUS = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_MINUS", "-"));
    KEY_ID_GESTURE_EQUALS = KeyNameToKeyCode(IniFile.ReadString("Keys", "GESTURE_CUSTOM_EQUALS", "="));
    
    KEY_ID_SAVE_POSE_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "SAVE_POSE_LEFT", "PERIOD"));
    KEY_ID_SAVE_POSE_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "SAVE_POSE_RIGHT", "COMMA"));
    
    // DIAGNOSTIC LOG:
    vr::VRDriverLog()->Log(("FlatVR: Config Loaded -> SaveLeft Key: " + std::to_string(KEY_ID_SAVE_POSE_LEFT) + " | SaveRight Key: " + std::to_string(KEY_ID_SAVE_POSE_RIGHT) + "\n").c_str());

    KEY_ID_PROFILE_CYCLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "PROFILE_CYCLE", "F9"));
    KEY_ID_SHAKE_LEFT_H = KeyNameToKeyCode(IniFile.ReadString("Keys", "SHAKE_LEFT_H", "F1"));
    KEY_ID_SHAKE_RIGHT_H = KeyNameToKeyCode(IniFile.ReadString("Keys", "SHAKE_RIGHT_H", "F2"));
    KEY_ID_SHAKE_LEFT_V = KeyNameToKeyCode(IniFile.ReadString("Keys", "SHAKE_LEFT_V", "F3"));
    KEY_ID_SHAKE_RIGHT_V = KeyNameToKeyCode(IniFile.ReadString("Keys", "SHAKE_RIGHT_V", "F4"));
    KEY_ID_NECK_MODEL_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "NECK_MODEL_TOGGLE", "F10"));

    std::string modeStr;
    modeStr = IniFile.ReadString("Modes", "LEFT_TRIGGER_MODE", IniFile.ReadString("Toggles", "LEFT_TRIGGER_MODE", "HOLD")); Btn_LeftTrigger.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_GRIP_MODE", IniFile.ReadString("Toggles", "LEFT_GRIP_MODE", "HOLD")); Btn_LeftGrip.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_MENU_MODE", IniFile.ReadString("Toggles", "LEFT_MENU_MODE", "HOLD")); Btn_LeftMenu.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_SYSTEM_MODE", IniFile.ReadString("Toggles", "LEFT_SYSTEM_MODE", "HOLD")); Btn_LeftSystem.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_TRIGGER_MODE", IniFile.ReadString("Toggles", "RIGHT_TRIGGER_MODE", "HOLD")); Btn_RightTrigger.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_GRIP_MODE", IniFile.ReadString("Toggles", "RIGHT_GRIP_MODE", "HOLD")); Btn_RightGrip.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_MENU_MODE", IniFile.ReadString("Toggles", "RIGHT_MENU_MODE", "HOLD")); Btn_RightMenu.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_SYSTEM_MODE", IniFile.ReadString("Toggles", "RIGHT_SYSTEM_MODE", "HOLD")); Btn_RightSystem.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_A_MODE", IniFile.ReadString("Toggles", "LEFT_A_MODE", "HOLD")); Btn_LeftA.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_B_MODE", IniFile.ReadString("Toggles", "LEFT_B_MODE", "HOLD")); Btn_LeftB.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_A_MODE", IniFile.ReadString("Toggles", "RIGHT_A_MODE", "HOLD")); Btn_RightA.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_B_MODE", IniFile.ReadString("Toggles", "RIGHT_B_MODE", "HOLD")); Btn_RightB.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_TRACKPAD_CLICK_MODE", IniFile.ReadString("Toggles", "LEFT_TRACKPAD_CLICK_MODE", "HOLD")); Btn_LeftTrackpadClick.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_TRACKPAD_CLICK_MODE", IniFile.ReadString("Toggles", "RIGHT_TRACKPAD_CLICK_MODE", "HOLD")); Btn_RightTrackpadClick.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "UP_MODE", IniFile.ReadString("Toggles", "UP_MODE", "HOLD")); Btn_Up.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "DOWN_MODE", IniFile.ReadString("Toggles", "DOWN_MODE", "HOLD")); Btn_Down.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "LEFT_MODE", IniFile.ReadString("Toggles", "LEFT_MODE", "HOLD")); Btn_Left.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "RIGHT_MODE", IniFile.ReadString("Toggles", "RIGHT_MODE", "HOLD")); Btn_Right.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "TURN_LEFT_MODE", IniFile.ReadString("Toggles", "TURN_LEFT_MODE", "HOLD")); Btn_TurnLeft.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "TURN_RIGHT_MODE", IniFile.ReadString("Toggles", "TURN_RIGHT_MODE", "HOLD")); Btn_TurnRight.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "TELEPORT_MODE", IniFile.ReadString("Toggles", "TELEPORT_MODE", "HOLD")); Btn_Teleport.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "CHANGE_WEAPON_MODE", IniFile.ReadString("Toggles", "CHANGE_WEAPON_MODE", "HOLD")); Btn_ChangeWeapon.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "CROUCH_MODE", IniFile.ReadString("Toggles", "CROUCH_MODE", "TOGGLE")); Btn_Crouch.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "PRONE_MODE", IniFile.ReadString("Toggles", "PRONE_MODE", "TOGGLE")); Btn_Prone.isToggle = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "SNIPER_TOGGLE", IniFile.ReadString("Toggles", "SNIPER_TOGGLE", "HOLD")); SniperToggleMode = (modeStr == "TOGGLE");
    modeStr = IniFile.ReadString("Modes", "AUTO_SWING_TOGGLE", IniFile.ReadString("Toggles", "AUTO_SWING_TOGGLE", "HOLD")); AutoSwingHoldMode = (modeStr == "HOLD");
    modeStr = IniFile.ReadString("Modes", "TRACKPAD_DISTANCE_MODE", IniFile.ReadString("Toggles", "TRACKPAD_DISTANCE_MODE", "HOLD")); Btn_TrackpadDistance.isToggle = (modeStr == "TOGGLE");
}

// --- HMD DEVICE DRIVER CLASS ---
class CSampleDeviceDriver : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	CSampleDeviceDriver() {
		HMDIndex_t = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
		m_flIPD = vr::VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);
		char buf[1024];
		vr::VRSettings()->GetString(k_pch_Sample_Section, k_pch_Sample_SerialNumber_String, buf, sizeof(buf)); m_sSerialNumber = buf;
		vr::VRSettings()->GetString(k_pch_Sample_Section, k_pch_Sample_ModelNumber_String, buf, sizeof(buf)); m_sModelNumber = buf;
		m_nWindowX = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowX_Int32); m_nWindowY = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowY_Int32);
		m_nWindowWidth = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowWidth_Int32); m_nWindowHeight = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_WindowHeight_Int32);
		if (m_nWindowWidth == 0) m_nWindowWidth = GetSystemMetrics(SM_CXSCREEN); if (m_nWindowHeight == 0) m_nWindowHeight = GetSystemMetrics(SM_CYSCREEN);
		m_nRenderWidth = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderWidth_Int32); m_nRenderHeight = vr::VRSettings()->GetInt32(k_pch_Sample_Section, k_pch_Sample_RenderHeight_Int32);
		m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_SecondsFromVsyncToPhotons_Float); m_flDisplayFrequency = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_DisplayFrequency_Float);
		m_fZoomWidth = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_ZoomWidth_Float); m_fZoomHeight = vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_ZoomHeight_Float);
		m_fFOV = (vr::VRSettings()->GetFloat(k_pch_Sample_Section, k_pch_Sample_FOV_Float) * 3.14159265358979323846 / 180);
		m_bStereoMode = vr::VRSettings()->GetBool(k_pch_Sample_Section, k_pch_Sample_Stereo_Bool); m_bDebugMode = vr::VRSettings()->GetBool(k_pch_Sample_Section, k_pch_Sample_DebugMode_Bool);
	}
	virtual ~CSampleDeviceDriver() {}
	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId) {
		HMDIndex_t = unObjectId; m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(HMDIndex_t);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, 0.0f);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
		vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DisplayDebugMode_Bool, m_bDebugMode);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, m_sSerialNumber.c_str());
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_WillDriftInYaw_Bool, false);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingFirmwareVersion_String, "1.0");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_HardwareRevision_String, "1.0");
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DeviceIsWireless_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DeviceIsCharging_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_CanUnifyCoordinateSystemWithHmd_Bool, true);
		
		// Ensure SteamVR knows this device has a proximity sensor
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_ContainsProximitySensor_Bool, true);
		
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_DeviceCanPowerOff_Bool, false);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_HMD);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCamera_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_NeverTracked_Bool, false);
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Identifiable_Bool, false);

		if (ControllerType == "oculus_touch") {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ManufacturerName_String, "Oculus");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RegisteredDeviceType_String, "oculus/Facebook");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "oculus_cv1_headset");
		} else {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ManufacturerName_String, "HTC");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RegisteredDeviceType_String, "HMD");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
		}

		// Create the dummy proximity sensor and force it to true
		vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/proximity", &m_proximitySensorHandle);
		vr::VRDriverInput()->UpdateBooleanComponent(m_proximitySensorHandle, true, 0.0);

		return VRInitError_None;
	}
	virtual void Deactivate() { HMDIndex_t = vr::k_unTrackedDeviceIndexInvalid; }
	virtual void EnterStandby() {}
	void* GetComponent(const char* pchComponentNameAndVersion) { if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version)) return (vr::IVRDisplayComponent*)this; return NULL; }
	virtual void PowerOff() {}
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) { if (unResponseBufferSize >= 1) pchResponseBuffer[0] = 0; }
	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) { *pnX = m_nWindowX; *pnY = m_nWindowY; *pnWidth = m_nWindowWidth; *pnHeight = m_nWindowHeight; }
	virtual bool IsDisplayOnDesktop() { return true; } virtual bool IsDisplayRealDisplay() { return true; }
	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) { *pnWidth = m_nRenderWidth; *pnHeight = m_nRenderHeight; }

	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) {
		if (m_bStereoMode) {
			*pnY = 0;
			*pnWidth = m_nWindowWidth / 2;
			*pnHeight = m_nWindowHeight;
			if (eEye == Eye_Left) *pnX = 0;
			else *pnX = (m_nWindowWidth / 2);
		}
		else {
			*pnY = 0;
			*pnWidth = m_nWindowWidth;
			*pnHeight = m_nWindowHeight;

			if (eEye == Eye_Left) *pnX = 0;
			else *pnX = m_nWindowWidth;
		}
	}

	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) {
		if (m_bStereoMode) { *pfLeft = -m_fFOV; *pfRight = m_fFOV; *pfTop = -m_fFOV; *pfBottom = m_fFOV; }
		else { *pfLeft = (float)m_nRenderWidth / m_nRenderHeight * -1; *pfRight = (float)m_nRenderWidth / m_nRenderHeight; *pfTop = -1.0; *pfBottom = 1.0; }
	}
	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV) {
		DistortionCoordinates_t coordinates; float hX, hY; double rr, r2, theta;
		rr = sqrt((fU - 0.5f) * (fU - 0.5f) + (fV - 0.5f) * (fV - 0.5f)); r2 = rr * (1 + 0 * (rr * rr) + 0 * (rr * rr * rr * rr));
		theta = atan2(fU - 0.5f, fV - 0.5f); hX = sin(theta) * r2 * m_fZoomWidth; hY = cos(theta) * r2 * m_fZoomHeight;
		coordinates.rfBlue[0] = hX + 0.5f; coordinates.rfBlue[1] = hY + 0.5f; coordinates.rfGreen[0] = hX + 0.5f; coordinates.rfGreen[1] = hY + 0.5f;
		coordinates.rfRed[0] = hX + 0.5f; coordinates.rfRed[1] = hY + 0.5f; return coordinates;
	}
	virtual DriverPose_t GetPose() {
		DriverPose_t pose = { 0 }; if (HMDConnected) { pose.poseIsValid = true; pose.result = TrackingResult_Running_OK; pose.deviceIsConnected = true; }
		else { pose.poseIsValid = false; pose.result = TrackingResult_Uninitialized; pose.deviceIsConnected = false; }
		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0); pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
		if (HMDConnected) {
			GetHMDData(&MyHMD);
			
			if (!SmoothHMD_Init) {
				SmoothHMD_X = MyHMD.X; SmoothHMD_Y = MyHMD.Z; SmoothHMD_Z = MyHMD.Y;
				SmoothHMD_Init = true;
			}
			SmoothHMD_X += (MyHMD.X - SmoothHMD_X) * PositionSmoothness;
			SmoothHMD_Y += (MyHMD.Z - SmoothHMD_Y) * PositionSmoothness;
			SmoothHMD_Z += (MyHMD.Y - SmoothHMD_Z) * PositionSmoothness;
			if (std::abs(MyHMD.X - SmoothHMD_X) < 0.0001) SmoothHMD_X = MyHMD.X;
			if (std::abs(MyHMD.Z - SmoothHMD_Y) < 0.0001) SmoothHMD_Y = MyHMD.Z;
			if (std::abs(MyHMD.Y - SmoothHMD_Z) < 0.0001) SmoothHMD_Z = MyHMD.Y;

			pose.vecPosition[0] = SmoothHMD_X;
			pose.vecPosition[1] = SmoothHMD_Y;
			pose.vecPosition[2] = SmoothHMD_Z;
			
			pose.qRotation = EulerAngleToQuaternion(DegToRad(-MyHMD.Yaw), DegToRad(-MyHMD.Pitch), DegToRad(MyHMD.Roll));

			double dt = g_DtSeconds;
			double dYaw = cameraHMD.Yaw - LastHmdYaw;
			double dPitch = cameraHMD.Pitch - LastHmdPitch;
			if (dYaw > 180) dYaw -= 360; if (dYaw < -180) dYaw += 360;
			if (dPitch > 180) dPitch -= 360; if (dPitch < -180) dPitch += 360;

			double targetOmegaX = DegToRad(dPitch) / dt;
			double targetOmegaY = DegToRad(dYaw) / dt;

			HmdOmegaX = (HmdOmegaX * 0.85) + (targetOmegaX * 0.15);
			HmdOmegaY = (HmdOmegaY * 0.85) + (targetOmegaY * 0.15);

			LastHmdYaw = cameraHMD.Yaw;
			LastHmdPitch = cameraHMD.Pitch;

			pose.vecVelocity[0] = 0; pose.vecVelocity[1] = 0; pose.vecVelocity[2] = 0;
			pose.vecAngularVelocity[0] = HmdOmegaX; pose.vecAngularVelocity[1] = HmdOmegaY; pose.vecAngularVelocity[2] = 0;
		}
		pose.shouldApplyHeadModel = NeckModelEnabled && !FPSModeActive;
		pose.poseTimeOffset = 0; pose.willDriftInYaw = false; return pose;
	}
	void RunFrame() { if (HMDIndex_t != vr::k_unTrackedDeviceIndexInvalid) vr::VRServerDriverHost()->TrackedDevicePoseUpdated(HMDIndex_t, GetPose(), sizeof(DriverPose_t)); }
	std::string GetSerialNumber() const { return m_sSerialNumber; }
private:
	vr::TrackedDeviceIndex_t HMDIndex_t; vr::PropertyContainerHandle_t m_ulPropertyContainer;
	vr::VRInputComponentHandle_t m_proximitySensorHandle;
	std::string m_sSerialNumber, m_sModelNumber;
	int32_t m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight, m_nRenderWidth, m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons, m_flDisplayFrequency, m_flIPD, m_fZoomWidth, m_fZoomHeight, m_fFOV;
	bool m_bStereoMode, m_bDebugMode;
};

// --- CONTROLLER DEVICE DRIVER CLASS ---
class CSampleControllerDriver : public vr::ITrackedDeviceServerDriver
{
	int32_t ControllerIndex;
public:
	CSampleControllerDriver() { Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid; Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid; m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer; }
	virtual void SetControllerIndex(int32_t CtrlIndex) { ControllerIndex = CtrlIndex; }
	virtual ~CSampleControllerDriver() {}

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		switch (ControllerIndex) {
		case 1: Ctrl1Index_t = unObjectId; m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl1Index_t); vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL1Serial"); vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand); break;
		case 2: Ctrl2Index_t = unObjectId; m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(Ctrl2Index_t); vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "CTRL2Serial"); vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand); break;
		}

		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		if (ControllerType == "oculus_touch") {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "oculus_touch");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "Oculus Touch");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Oculus");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, ControllerIndex == 1 ? "oculus_cv1_controller_left" : "oculus_cv1_controller_right");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "Oculus");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
			vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 0xFFFFFFFF);

			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x", &HAnalog[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y", &HAnalog[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &HAnalog[3], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &HButtons[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &HButtons[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/x/click", &HButtons[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/y/click", &HButtons[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/click", &HButtons[4]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/menu/click", &HButtons[5]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[6]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtons[7]);
		} else {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");
			vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, 0xA4);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{htc}/input/controller_profile.json");

			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/x", &HAnalog[0], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trackpad/y", &HAnalog[1], vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &HAnalog[2], vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &HButtons[0]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &HButtons[2]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &HButtons[1]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/click", &HButtons[5]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/click", &HButtons[3]);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trackpad/touch", &HButtons[4]);
		}

		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);
		return VRInitError_None;
	}

	virtual void Deactivate() { switch (ControllerIndex) { case 1: Ctrl1Index_t = vr::k_unTrackedDeviceIndexInvalid; break; case 2: Ctrl2Index_t = vr::k_unTrackedDeviceIndexInvalid; break; } }
	virtual void EnterStandby() {}
	void* GetComponent(const char* pchComponentNameAndVersion) { return NULL; }
	virtual void PowerOff() {}
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) { if (unResponseBufferSize >= 1) pchResponseBuffer[0] = 0; }

	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 }; pose.poseIsValid = true; pose.result = TrackingResult_Running_OK; pose.deviceIsConnected = true;
		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0); pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		double dt = g_DtSeconds;

		if (ControllerIndex == 1) {
			if (!SmoothL_Init) {
				SmoothL_X = MyCtrl.X; SmoothL_Y = MyCtrl.Z; SmoothL_Z = MyCtrl.Y;
				SmoothL_Init = true;
			}
			
			// FIX: Frame-Rate Independent Smoothing
			double alpha = 1.0 - std::pow(1.0 - PositionSmoothness, dt * 90.0);
			SmoothL_X += (MyCtrl.X - SmoothL_X) * alpha;
			SmoothL_Y += (MyCtrl.Z - SmoothL_Y) * alpha;
			SmoothL_Z += (MyCtrl.Y - SmoothL_Z) * alpha;
			
			if (std::abs(MyCtrl.X - SmoothL_X) < 0.0001) SmoothL_X = MyCtrl.X;
			if (std::abs(MyCtrl.Z - SmoothL_Y) < 0.0001) SmoothL_Y = MyCtrl.Z;
			if (std::abs(MyCtrl.Y - SmoothL_Z) < 0.0001) SmoothL_Z = MyCtrl.Y;
			
			pose.vecPosition[0] = SmoothL_X;
			pose.vecPosition[1] = SmoothL_Y;
			pose.vecPosition[2] = SmoothL_Z;

			double rx = MyCtrl.X - HMDPosX;
			double ry = MyCtrl.Z - (HMDPosZ - CrouchOffsetZ);
			double rz = MyCtrl.Y - HMDPosY;

			double fakeVX = HmdOmegaY * rz;
			double fakeVY = -HmdOmegaX * rz;
			double fakeVZ = HmdOmegaX * ry - HmdOmegaY * rx;

			double trueVX = (pose.vecPosition[0] - FirstCtrlLastPos[0]) / dt;
			double trueVY = (pose.vecPosition[1] - FirstCtrlLastPos[1]) / dt;
			double trueVZ = (pose.vecPosition[2] - FirstCtrlLastPos[2]) / dt;

			FirstCtrlLastPos[0] = pose.vecPosition[0]; FirstCtrlLastPos[1] = pose.vecPosition[1]; FirstCtrlLastPos[2] = pose.vecPosition[2];

			// FIX: Robust Velocity Calculation (Low-pass + Clamp)
			bool headMoving = (HmdOmegaX*HmdOmegaX + HmdOmegaY*HmdOmegaY) > 0.01;
			if (!headMoving || std::abs(trueVX - fakeVX) > 0.5 || std::abs(trueVY - fakeVY) > 0.5 || std::abs(trueVZ - fakeVZ) > 0.5) {
				FirstCtrlVel[0] += (trueVX - FirstCtrlVel[0]) * 0.3; // Smooth out noise
				FirstCtrlVel[1] += (trueVY - FirstCtrlVel[1]) * 0.3;
				FirstCtrlVel[2] += (trueVZ - FirstCtrlVel[2]) * 0.3;
			} else {
				FirstCtrlVel[0] += (fakeVX - FirstCtrlVel[0]) * 0.3;
				FirstCtrlVel[1] += (fakeVY - FirstCtrlVel[1]) * 0.3;
				FirstCtrlVel[2] += (fakeVZ - FirstCtrlVel[2]) * 0.3;
			}

			// Clamp max velocity to 15 m/s to prevent physics explosions
			double velMag = sqrt(FirstCtrlVel[0]*FirstCtrlVel[0] + FirstCtrlVel[1]*FirstCtrlVel[1] + FirstCtrlVel[2]*FirstCtrlVel[2]);
			if (velMag > 15.0) {
				FirstCtrlVel[0] *= (15.0 / velMag);
				FirstCtrlVel[1] *= (15.0 / velMag);
				FirstCtrlVel[2] *= (15.0 / velMag);
			}

			pose.vecVelocity[0] = FirstCtrlVel[0]; pose.vecVelocity[1] = FirstCtrlVel[1]; pose.vecVelocity[2] = FirstCtrlVel[2];

			if (ShakeTimerL > 0 || ManualShakeL_H || ManualShakeL_V) {
				double activeIntX = ShakeIntensityL[0];
				double activeIntY = ShakeIntensityL[1];
				double activeIntZ = ShakeIntensityL[2];

				if (ManualShakeL_H) activeIntX = 0.03;
				if (ManualShakeL_V) activeIntY = 0.03;

				pose.vecPosition[0] += sin(ShakeTimeL) * activeIntX;
				pose.vecPosition[1] += sin(ShakeTimeL * 1.3) * activeIntY;
				pose.vecPosition[2] += cos(ShakeTimeL) * activeIntZ;
			}

			pose.vecAngularVelocity[0] = HmdOmegaX; pose.vecAngularVelocity[1] = HmdOmegaY; pose.vecAngularVelocity[2] = 0;

			pose.qRotation = MyCtrl.UseQuat ? HmdQuaternion_Init(MyCtrl.QW, MyCtrl.QX, MyCtrl.QY, MyCtrl.QZ)
			                                : EulerAngleToQuaternion(DegToRad(-MyCtrl.Yaw), DegToRad(-MyCtrl.Pitch), DegToRad(MyCtrl.Roll));

			double linSq = pose.vecVelocity[0]*pose.vecVelocity[0]
			             + pose.vecVelocity[1]*pose.vecVelocity[1]
			             + pose.vecVelocity[2]*pose.vecVelocity[2];
			if (linSq < 0.0025) {
				pose.vecVelocity[0] = 0.0;
				pose.vecVelocity[1] = 0.0;
				pose.vecVelocity[2] = 0.0;
			}
			double angSq = pose.vecAngularVelocity[0]*pose.vecAngularVelocity[0]
			             + pose.vecAngularVelocity[1]*pose.vecAngularVelocity[1]
			             + pose.vecAngularVelocity[2]*pose.vecAngularVelocity[2];
			if (angSq < 0.0025) {
				pose.vecAngularVelocity[0] = 0.0;
				pose.vecAngularVelocity[1] = 0.0;
				pose.vecAngularVelocity[2] = 0.0;
			}
		}
		else {
			if (!SmoothR_Init) {
				SmoothR_X = MyCtrl2.X; SmoothR_Y = MyCtrl2.Z; SmoothR_Z = MyCtrl2.Y;
				SmoothR_Init = true;
			}

			// FIX: Frame-Rate Independent Smoothing
			double alpha = 1.0 - std::pow(1.0 - PositionSmoothness, dt * 90.0);
			SmoothR_X += (MyCtrl2.X - SmoothR_X) * alpha;
			SmoothR_Y += (MyCtrl2.Z - SmoothR_Y) * alpha;
			SmoothR_Z += (MyCtrl2.Y - SmoothR_Z) * alpha;

			if (std::abs(MyCtrl2.X - SmoothR_X) < 0.0001) SmoothR_X = MyCtrl2.X;
			if (std::abs(MyCtrl2.Z - SmoothR_Y) < 0.0001) SmoothR_Y = MyCtrl2.Z;
			if (std::abs(MyCtrl2.Y - SmoothR_Z) < 0.0001) SmoothR_Z = MyCtrl2.Y;
			
			pose.vecPosition[0] = SmoothR_X;
			pose.vecPosition[1] = SmoothR_Y;
			pose.vecPosition[2] = SmoothR_Z;

			double rx = MyCtrl2.X - HMDPosX;
			double ry = MyCtrl2.Z - (HMDPosZ - CrouchOffsetZ);
			double rz = MyCtrl2.Y - HMDPosY;

			double fakeVX = HmdOmegaY * rz;
			double fakeVY = -HmdOmegaX * rz;
			double fakeVZ = HmdOmegaX * ry - HmdOmegaY * rx;

			double trueVX = (pose.vecPosition[0] - SecondCtrlLastPos[0]) / dt;
			double trueVY = (pose.vecPosition[1] - SecondCtrlLastPos[1]) / dt;
			double trueVZ = (pose.vecPosition[2] - SecondCtrlLastPos[2]) / dt;

			SecondCtrlLastPos[0] = pose.vecPosition[0]; SecondCtrlLastPos[1] = pose.vecPosition[1]; SecondCtrlLastPos[2] = pose.vecPosition[2];

			// FIX: Robust Velocity Calculation (Low-pass + Clamp)
			bool headMoving = (HmdOmegaX*HmdOmegaX + HmdOmegaY*HmdOmegaY) > 0.01;
			if (!headMoving || std::abs(trueVX - fakeVX) > 0.5 || std::abs(trueVY - fakeVY) > 0.5 || std::abs(trueVZ - fakeVZ) > 0.5) {
				SecondCtrlVel[0] += (trueVX - SecondCtrlVel[0]) * 0.3;
				SecondCtrlVel[1] += (trueVY - SecondCtrlVel[1]) * 0.3;
				SecondCtrlVel[2] += (trueVZ - SecondCtrlVel[2]) * 0.3;
			} else {
				SecondCtrlVel[0] += (fakeVX - SecondCtrlVel[0]) * 0.3;
				SecondCtrlVel[1] += (fakeVY - SecondCtrlVel[1]) * 0.3;
				SecondCtrlVel[2] += (fakeVZ - SecondCtrlVel[2]) * 0.3;
			}

			// Clamp max velocity
			double velMag = sqrt(SecondCtrlVel[0]*SecondCtrlVel[0] + SecondCtrlVel[1]*SecondCtrlVel[1] + SecondCtrlVel[2]*SecondCtrlVel[2]);
			if (velMag > 15.0) {
				SecondCtrlVel[0] *= (15.0 / velMag);
				SecondCtrlVel[1] *= (15.0 / velMag);
				SecondCtrlVel[2] *= (15.0 / velMag);
			}

			pose.vecVelocity[0] = SecondCtrlVel[0]; pose.vecVelocity[1] = SecondCtrlVel[1]; pose.vecVelocity[2] = SecondCtrlVel[2];

			if (ShakeTimerR > 0 || ManualShakeR_H || ManualShakeR_V) {
				double activeIntX = ShakeIntensityR[0];
				double activeIntY = ShakeIntensityR[1];
				double activeIntZ = ShakeIntensityR[2];

				if (ManualShakeR_H) activeIntX = 0.03;
				if (ManualShakeR_V) activeIntY = 0.03;

				pose.vecPosition[0] += sin(ShakeTimeR) * activeIntX;
				pose.vecPosition[1] += sin(ShakeTimeR * 1.3) * activeIntY;
				pose.vecPosition[2] += cos(ShakeTimeR) * activeIntZ;
			}

			pose.vecAngularVelocity[0] = HmdOmegaX; pose.vecAngularVelocity[1] = HmdOmegaY; pose.vecAngularVelocity[2] = 0;

			pose.qRotation = MyCtrl2.UseQuat ? HmdQuaternion_Init(MyCtrl2.QW, MyCtrl2.QX, MyCtrl2.QY, MyCtrl2.QZ)
			                                 : EulerAngleToQuaternion(DegToRad(-MyCtrl2.Yaw), DegToRad(-MyCtrl2.Pitch), DegToRad(MyCtrl2.Roll));

			double linSq = pose.vecVelocity[0]*pose.vecVelocity[0]
			             + pose.vecVelocity[1]*pose.vecVelocity[1]
			             + pose.vecVelocity[2]*pose.vecVelocity[2];
			if (linSq < 0.0025) {
				pose.vecVelocity[0] = 0.0;
				pose.vecVelocity[1] = 0.0;
				pose.vecVelocity[2] = 0.0;
			}
			double angSq = pose.vecAngularVelocity[0]*pose.vecAngularVelocity[0]
			             + pose.vecAngularVelocity[1]*pose.vecAngularVelocity[1]
			             + pose.vecAngularVelocity[2]*pose.vecAngularVelocity[2];
			if (angSq < 0.0025) {
				pose.vecAngularVelocity[0] = 0.0;
				pose.vecAngularVelocity[1] = 0.0;
				pose.vecAngularVelocity[2] = 0.0;
			}
		}
		return pose;
	}

	void RunFrame()
	{
		switch (ControllerIndex) {
		case 1: if (Ctrl1Index_t != vr::k_unTrackedDeviceIndexInvalid) vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl1Index_t, GetPose(), sizeof(DriverPose_t)); break;
		case 2: if (Ctrl2Index_t != vr::k_unTrackedDeviceIndexInvalid) vr::VRServerDriverHost()->TrackedDevicePoseUpdated(Ctrl2Index_t, GetPose(), sizeof(DriverPose_t)); break;
		}

		if (ControllerIndex == 1) {
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], MyCtrl.AxisX, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], MyCtrl.AxisY, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], MyCtrl.Trigger, 0);

			if (ControllerType == "oculus_touch") {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], MyCtrl.GripValue, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (MyCtrl.Buttons & THUMB_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], (MyCtrl.Buttons & MENU_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[6], (MyCtrl.Buttons & SYS_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[7], MyCtrl.Trigger > 0.1f, 0);

				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], MyCtrl.ButtonA, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], MyCtrl.ButtonB, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], (MyCtrl.Buttons & MENU_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], (MyCtrl.Buttons & GRIP_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], (MyCtrl.Buttons & SYS_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], ((MyCtrl.Buttons & THUMB_BTN) != 0) && LeftTrackpadPressMode, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (((MyCtrl.AxisX != 0 || MyCtrl.AxisY != 0) || (MyCtrl.Buttons & THUMB_BTN) != 0) ? true : false), 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], MyCtrl.Trigger > 0.1f, 0);
			}
		}
		else {
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[0], MyCtrl2.AxisX, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[1], MyCtrl2.AxisY, 0);
			vr::VRDriverInput()->UpdateScalarComponent(HAnalog[2], MyCtrl2.Trigger, 0);

			if (ControllerType == "oculus_touch") {
				vr::VRDriverInput()->UpdateScalarComponent(HAnalog[3], MyCtrl2.GripValue, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (MyCtrl2.Buttons & THUMB_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], (MyCtrl2.Buttons & MENU_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[6], (MyCtrl2.Buttons & SYS_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[7], MyCtrl2.Trigger > 0.1f, 0);

				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], MyCtrl2.ButtonA, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], MyCtrl2.ButtonB, 0);
			}
			else {
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[0], (MyCtrl2.Buttons & MENU_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[1], (MyCtrl2.Buttons & GRIP_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[2], (MyCtrl2.Buttons & SYS_BTN) != 0, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], ((MyCtrl2.Buttons & THUMB_BTN) != 0) && RightTrackpadPressMode, 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[4], (((MyCtrl2.AxisX != 0 || MyCtrl2.AxisY != 0) || (MyCtrl2.Buttons & THUMB_BTN) != 0) ? true : false), 0);
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[5], MyCtrl2.Trigger > 0.1f, 0);
			}
		}
	}

	void ProcessEvent(const vr::VREvent_t& vrEvent) {}
	std::string GetSerialNumber() const { return (ControllerIndex == 1) ? "CTRL1Serial" : "CTRL2Serial"; }

private:
	vr::TrackedDeviceIndex_t Ctrl1Index_t, Ctrl2Index_t;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;
	vr::VRInputComponentHandle_t m_compHaptic;
	vr::VRInputComponentHandle_t HButtons[10];
	vr::VRInputComponentHandle_t HAnalog[5];
};

// --- SERVER DRIVER CLASS ---
class CServerDriver_Sample : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
private:
	CSampleDeviceDriver* m_pNullHmdLatest = nullptr;
	CSampleControllerDriver* m_pController = nullptr;
	CSampleControllerDriver* m_pController2 = nullptr;
};

CServerDriver_Sample g_serverDriverNull;

// --- INITIALIZATION ---
EVRInitError CServerDriver_Sample::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	g_DriverFolderStr = GetDriverFolder();
	if (g_DriverFolderStr.empty()) {
		vr::VRDriverLog()->Log("FlatVR: FATAL - could not determine driver folder path\n");
		return VRInitError_Init_Internal;
	}

	vr::VRDriverLog()->Log(("FlatVR: driver folder = " + g_DriverFolderStr + "\n").c_str());

	ctrlsConnected = true;

	LoadProfile("Default");

    // FIX: Dynamic Screen Resolution
    m_HalfWidth = GetSystemMetrics(SM_CXSCREEN) / 2;
    m_HalfHeight = GetSystemMetrics(SM_CYSCREEN) / 2;

	g_HookThread = std::thread([]() {
		g_HookThreadId = GetCurrentThreadId();
		HMODULE hMod = NULL;
		GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCTSTR)reinterpret_cast<void*>(MouseHookProc), &hMod);

		g_hMouseHook = SetWindowsHookEx(WH_MOUSE_LL, MouseHookProc, hMod, 0);
		g_hKeyboardHook = SetWindowsHookEx(WH_KEYBOARD_LL, KeyboardHookProc, hMod, 0);

		if (g_hMouseHook || g_hKeyboardHook) {
			MSG msg;
			while (GetMessage(&msg, NULL, 0, 0)) {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
			if (g_hMouseHook) { UnhookWindowsHookEx(g_hMouseHook); g_hMouseHook = NULL; }
			if (g_hKeyboardHook) { UnhookWindowsHookEx(g_hKeyboardHook); g_hKeyboardHook = NULL; }
		}
	});
	g_HookThread.detach();

	if (HMDConnected) {
		m_pNullHmdLatest = new CSampleDeviceDriver();
		vr::VRServerDriverHost()->TrackedDeviceAdded(m_pNullHmdLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pNullHmdLatest);
	}
	if (ctrlsConnected) {
		m_pController = new CSampleControllerDriver(); m_pController->SetControllerIndex(1); vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);
		m_pController2 = new CSampleControllerDriver(); m_pController2->SetControllerIndex(2); vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController2->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController2);
	}
	return VRInitError_None;
}

void CServerDriver_Sample::Cleanup()
{
	if (g_HookThreadId != 0) {
		PostThreadMessage(g_HookThreadId, WM_QUIT, 0, 0);
		g_HookThreadId = 0;
	}
}

// --- MAIN RUNFRAME & INPUT STATE ---
void CServerDriver_Sample::RunFrame()
{
	static steady_clock::time_point lastFrame = steady_clock::now();
	auto now = steady_clock::now();
	auto raw_dt = now - lastFrame;
	lastFrame = now;

	deltaTime = duration_cast<milliseconds>(raw_dt);
	g_DtSeconds = duration<double>(raw_dt).count();
	
	// FIX: Clamp dt on BOTH sides to prevent velocity spikes on high framerates
	if (g_DtSeconds <= 0.001) g_DtSeconds = 0.001; // Min 1ms (1000 FPS)
	else if (g_DtSeconds > 0.25) g_DtSeconds = 1.0 / 90.0; // Max 250ms (4 FPS)

	FILETIME currentIniMod = GetFileModTime(g_IniPathStr);

	if (CompareFileTime(&lastIniMod, &currentIniMod) != 0) {
		lastIniMod = currentIniMod;
		LoadConfig();
	}

	SteamVRHeadsetWindow = FindWindow(NULL, "Headset Window");

	if (IsKeyDown(VK_F7)) SetForegroundWindow(SteamVRHeadsetWindow);

	HeadsetWindowFocused = SteamVRHeadsetWindow != 0 && SteamVRHeadsetWindow == GetForegroundWindow() && IsWindowVisible(SteamVRHeadsetWindow);

	if (HeadsetWindowFocused) {
		bool lr = IsKeyDown(KEY_ID_LEFT_HAND_ROTATE);
		bool rr = IsKeyDown(KEY_ID_RIGHT_HAND_ROTATE);
		bool resetKey = IsKeyDown(KEY_ID_SMOOTH_HAND_RESET);
		if (resetKey && !PrevSmoothResetKey) {
			IsSmoothResetting = true;
			SmoothResetTimer = 0.0;
			StartResetX_L = LeftXOffset; StartResetY_L = LeftRotateYOffset; StartResetZ_L = LeftZOffset;
			StartResetX_R = RightXOffset; StartResetY_R = RightYOffset; StartResetZ_R = RightZOffset;

			StartResetRotL_X = LeftHandRotOffset[0];
			StartResetRotL_Y = LeftHandRotOffset[1];
			StartResetRotL_Z = LeftHandRotOffset[2];
			StartResetRotR_X = RightHandRotOffset[0];
			StartResetRotR_Y = RightHandRotOffset[1];
			StartResetRotR_Z = RightHandRotOffset[2];
		}
		PrevSmoothResetKey = resetKey;

		if (lr && !PrevLeftRotateKey) {
			LeftHandIsRotating = !LeftHandIsRotating;
			if (!LeftHandIsRotating) {
				mouseYaw = cameraHMD.Yaw;
				mousePitch = cameraHMD.Pitch;
				mouseHMD = cameraHMD;
				oldPitch = mouseHMD.Yaw;
				oldRoll = mouseHMD.Pitch;
			}
		}
		if (rr && !PrevRightRotateKey) {
			RightHandIsRotating = !RightHandIsRotating;
			if (!RightHandIsRotating) {
				mouseYaw = cameraHMD.Yaw;
				mousePitch = cameraHMD.Pitch;
				mouseHMD = cameraHMD;
				oldPitch = mouseHMD.Yaw;
				oldRoll = mouseHMD.Pitch;
			}
		}
		PrevLeftRotateKey = lr; PrevRightRotateKey = rr;

		bool fpsToggle = IsKeyDown(KEY_ID_FPS_MODE_TOGGLE);
		if (fpsToggle && !PrevFPSModeKey) {
			FPSModeActive = !FPSModeActive;
		}
		PrevFPSModeKey = fpsToggle;

		// --- SNIPER MODE TOGGLE ---
		bool sniperKey = IsKeyDown(KEY_ID_SNIPER_MODE);
		if (SniperToggleMode) {
			if (sniperKey && !PrevSniperModeKey) {
				if (FPSModeActive) {
					SniperModeActive = !SniperModeActive;
					if (SniperModeActive) {
						SniperJustStarted = true;
						SniperExiting = false;
					} else {
						SniperExiting = true;
						SniperExitTimer = 0.0;
						SniperJustEnded = true;
					}
				} else {
					vr::VRDriverLog()->Log("FlatVR: Sniper Mode requires FPS Mode active.\n");
				}
			}
		} else {
			if (sniperKey && !PrevSniperModeKey) {
				if (FPSModeActive) {
					SniperModeActive = true;
					SniperJustStarted = true;
					SniperExiting = false;
				} else {
					vr::VRDriverLog()->Log("FlatVR: Sniper Mode requires FPS Mode active.\n");
				}
			} else if (!sniperKey && PrevSniperModeKey) {
				if (SniperModeActive) {
					SniperModeActive = false;
					SniperExiting = true;
					SniperExitTimer = 0.0;
					SniperJustEnded = true;
				}
			}
		}
		PrevSniperModeKey = sniperKey;
		// --- END SNIPER TOGGLE ---

		ManualShakeL_H = IsKeyDown(KEY_ID_SHAKE_LEFT_H);
		ManualShakeR_H = IsKeyDown(KEY_ID_SHAKE_RIGHT_H);
		ManualShakeL_V = IsKeyDown(KEY_ID_SHAKE_LEFT_V);
		ManualShakeR_V = IsKeyDown(KEY_ID_SHAKE_RIGHT_V);

		bool cycleKey = IsKeyDown(KEY_ID_PROFILE_CYCLE);
		if (cycleKey && !PrevProfileCycleKey) {
			CycleProfile();
		}
		PrevProfileCycleKey = cycleKey;

		bool neckToggle = IsKeyDown(KEY_ID_NECK_MODEL_TOGGLE);
		if (neckToggle && !PrevNeckToggleKey) {
			NeckModelEnabled = !NeckModelEnabled;
			vr::VRDriverLog()->Log(NeckModelEnabled ? "FlatVR: Neck Model ON\n" : "FlatVR: Neck Model OFF\n");
		}
		PrevNeckToggleKey = neckToggle;

        // --- SAVE POSE HOTKEYS ---
        bool saveL = IsKeyDown(KEY_ID_SAVE_POSE_LEFT);
        if (saveL && !PrevSavePoseL) {
            std::string poseDir = g_DriverFolderStr + "/Resources/Pose_Config/" + g_ActiveProfileName;
            CreateDirectoryA((g_DriverFolderStr + "/Resources").c_str(), NULL);
            CreateDirectoryA((g_DriverFolderStr + "/Resources/Pose_Config").c_str(), NULL);
            CreateDirectoryA(poseDir.c_str(), NULL);

            int idx = 1;
            std::string path;
            do {
                path = poseDir + "/RSAVE_" + std::to_string(idx) + ".pose";
                idx++;
            } while (GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES);

            // Calculate HMD Local Space for the Left Hand (Corrected Math)
            double effYaw = DegToRad(cameraHMD.Yaw);
            double effPitch = DegToRad(cameraHMD.Pitch);
            
            double rx =  cos(effYaw);
            double ry = -sin(effYaw);
            double rz =  0;
            
            double fx = -sin(effYaw) * cos(effPitch);
            double fy = -cos(effYaw) * cos(effPitch);
            double fz =  sin(effPitch);
            
            double ux =  sin(effYaw) * sin(effPitch);
            double uy =  cos(effYaw) * sin(effPitch);
            double uz =  cos(effPitch);

            double relX = MyCtrl.X - HMDPosX;
            double relY = MyCtrl.Y - HMDPosY;
            double relZ = MyCtrl.Z - (HMDPosZ - CrouchOffsetZ);

            double locX = relX*rx + relY*ry + relZ*rz; // Right
            double locY = relX*fx + relY*fy + relZ*fz; // Forward
            double locZ = relX*ux + relY*uy + relZ*uz; // Up

            std::ofstream out(path);
            if (out.is_open()) {
                out << "L " << locX << " " << locY << " " << locZ << " "
                    << (MyCtrl.Yaw - cameraHMD.Yaw) << " " << (MyCtrl.Pitch - cameraHMD.Pitch) << " " << MyCtrl.Roll << " REL=HMD\n";
                out.close();
                vr::VRDriverLog()->Log(("FlatVR: Saved Left Hand Pose to " + path + "\n").c_str());
            }
        }
        PrevSavePoseL = saveL;

        bool saveR = IsKeyDown(KEY_ID_SAVE_POSE_RIGHT);
        if (saveR && !PrevSavePoseR) {
            std::string poseDir = g_DriverFolderStr + "/Resources/Pose_Config/" + g_ActiveProfileName;
            CreateDirectoryA((g_DriverFolderStr + "/Resources").c_str(), NULL);
            CreateDirectoryA((g_DriverFolderStr + "/Resources/Pose_Config").c_str(), NULL);
            CreateDirectoryA(poseDir.c_str(), NULL);

            int idx = 1;
            std::string path;
            do {
                path = poseDir + "/LSAVE_" + std::to_string(idx) + ".pose";
                idx++;
            } while (GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES);

            // Calculate HMD Local Space for the Right Hand (Corrected Math)
            double effYaw = DegToRad(cameraHMD.Yaw);
            double effPitch = DegToRad(cameraHMD.Pitch);
            
            double rx =  cos(effYaw);
            double ry = -sin(effYaw);
            double rz =  0;
            
            double fx = -sin(effYaw) * cos(effPitch);
            double fy = -cos(effYaw) * cos(effPitch);
            double fz =  sin(effPitch);
            
            double ux =  sin(effYaw) * sin(effPitch);
            double uy =  cos(effYaw) * sin(effPitch);
            double uz =  cos(effPitch);

            double relX = MyCtrl.X - HMDPosX;
            double relY = MyCtrl.Y - HMDPosY;
            double relZ = MyCtrl.Z - (HMDPosZ - CrouchOffsetZ);

            double locX = relX*rx + relY*ry + relZ*rz; // Right
            double locY = relX*fx + relY*fy + relZ*fz; // Forward
            double locZ = relX*ux + relY*uy + relZ*uz; // Up

            std::ofstream out(path);
            if (out.is_open()) {
                out << "R " << locX << " " << locY << " " << locZ << " "
                    << (MyCtrl2.Yaw - cameraHMD.Yaw) << " " << (MyCtrl2.Pitch - cameraHMD.Pitch) << " " << MyCtrl2.Roll << " REL=HMD\n";
                out.close();
                vr::VRDriverLog()->Log(("FlatVR: Saved Right Hand Pose to " + path + "\n").c_str());
            }
        }
        PrevSavePoseR = saveR;
        // -------------------------

		if (ControllerType == "vive_controller") {
			bool ltp = (KEY_ID_LEFT_TRACKPAD_TOGGLE != 0) && IsKeyDown(KEY_ID_LEFT_TRACKPAD_TOGGLE);
			if (ltp && !PrevLeftTrackpadToggle) {
				LeftTrackpadPressMode = !LeftTrackpadPressMode;
				vr::VRDriverLog()->Log(LeftTrackpadPressMode
					? "FlatVR: Left touchpad -> PRESS (click enabled)\n"
					: "FlatVR: Left touchpad -> TOUCH (click suppressed)\n");
			}
			PrevLeftTrackpadToggle = ltp;

			bool rtp = (KEY_ID_RIGHT_TRACKPAD_TOGGLE != 0) && IsKeyDown(KEY_ID_RIGHT_TRACKPAD_TOGGLE);
			if (rtp && !PrevRightTrackpadToggle) {
				RightTrackpadPressMode = !RightTrackpadPressMode;
				vr::VRDriverLog()->Log(RightTrackpadPressMode
					? "FlatVR: Right touchpad -> PRESS (click enabled)\n"
					: "FlatVR: Right touchpad -> TOUCH (click suppressed)\n");
			}
			PrevRightTrackpadToggle = rtp;
		}

		bool swingKey = IsKeyDown(KEY_ID_AUTO_SWING_TOGGLE);
		if (AutoSwingHoldMode) {
			AutoSwingActive = swingKey;
		} else {
			if (swingKey && !PrevAutoSwingKey) {
				AutoSwingActive = !AutoSwingActive;
			}
		}
		PrevAutoSwingKey = swingKey;

		bool lockL = IsKeyDown(KEY_ID_LEFT_HAND_LOCK);
		if (lockL && !PrevLeftLockKey) {
			LeftHandLocked = !LeftHandLocked;
			if (LeftHandLocked) {
				LockedLeftPos[0] = MyCtrl.X; LockedLeftPos[1] = MyCtrl.Y; LockedLeftPos[2] = MyCtrl.Z;
				LockedLeftYaw = MyCtrl.Yaw; LockedLeftPitch = MyCtrl.Pitch; LockedLeftRoll = MyCtrl.Roll;
				LockedLeftUseQuat = MyCtrl.UseQuat; LockedLeftQ[0] = MyCtrl.QW; LockedLeftQ[1] = MyCtrl.QX; LockedLeftQ[2] = MyCtrl.QY; LockedLeftQ[3] = MyCtrl.QZ;
			}
		}
		PrevLeftLockKey = lockL;

		bool lockR = IsKeyDown(KEY_ID_RIGHT_HAND_LOCK);
		if (lockR && !PrevRightLockKey) {
			RightHandLocked = !RightHandLocked;
			if (RightHandLocked) {
				LockedRightPos[0] = MyCtrl2.X; LockedRightPos[1] = MyCtrl2.Y; LockedRightPos[2] = MyCtrl2.Z;
				LockedRightYaw = MyCtrl2.Yaw; LockedRightPitch = MyCtrl2.Pitch; LockedRightRoll = MyCtrl2.Roll;
				LockedRightUseQuat = MyCtrl2.UseQuat; LockedRightQ[0] = MyCtrl2.QW; LockedRightQ[1] = MyCtrl2.QX; LockedRightQ[2] = MyCtrl2.QY; LockedRightQ[3] = MyCtrl2.QZ;
			}
		}
		PrevRightLockKey = lockR;

		bool pumpKey = IsKeyDown(KEY_ID_PUMP_ACTION);
		if (pumpKey && !PrevPumpKey) {
			if (PumpTimer <= 0.0) {
				PumpTimer = PumpDuration;
			}
		}
		PrevPumpKey = pumpKey;

		float rdt = (float)g_DtSeconds;
		bool freelookHeld = IsKeyDown(KEY_ID_FREELOOK) && !LeftHandIsRotating && !RightHandIsRotating;

		if (freelookHeld) {
			POINT mousePos;
			GetCursorPos(&mousePos);
			FreelookYawOffset += (mousePos.x - m_HalfWidth) * mouseSensetiveYaw * -1;
			FreelookPitchOffset += (mousePos.y - m_HalfHeight) * mouseSensetivePitch * -1;
			SetCursorPos(m_HalfWidth, m_HalfHeight);

			if (cameraHMD.Pitch + FreelookPitchOffset > 90) FreelookPitchOffset = 90 - cameraHMD.Pitch;
			if (cameraHMD.Pitch + FreelookPitchOffset < -90) FreelookPitchOffset = -90 - cameraHMD.Pitch;
		}
		else {
			if (FreelookYawOffset != 0.0f || FreelookPitchOffset != 0.0f) {
				FreelookYawOffset -= FreelookYawOffset * 15.0f * rdt;
				FreelookPitchOffset -= FreelookPitchOffset * 15.0f * rdt;

				if (std::abs(FreelookYawOffset) < 0.1f) FreelookYawOffset = 0.0f;
				if (std::abs(FreelookPitchOffset) < 0.1f) FreelookPitchOffset = 0.0f;

				SetCursorPos(m_HalfWidth, m_HalfHeight);

				mouseYaw = cameraHMD.Yaw;
				mousePitch = cameraHMD.Pitch;
				mouseHMD = cameraHMD;
			} else {
				MouseToYawPitch();
				if (!LeftHandIsRotating && !RightHandIsRotating) cameraHMD = mouseHMD;
			}
		}
	}
	else {
		LeftHandIsRotating = false;
		RightHandIsRotating = false;
		PrevLeftRotateKey = false;
		PrevRightRotateKey = false;
		if (AutoSwingHoldMode) AutoSwingActive = false;

		// FIX: Release remapped keys on focus loss
		for (int i = 0; i < 256; ++i) {
			if (g_RemappedKeyActive[i].load(std::memory_order_relaxed)) {
				auto it = g_KeyRemapper.find(i);
				if (it != g_KeyRemapper.end()) {
					INPUT input = {0};
					input.type = INPUT_KEYBOARD;
					input.ki.wVk = it->second;
					input.ki.dwFlags = KEYEVENTF_KEYUP;
					SendInput(1, &input, sizeof(INPUT));
				}
				g_RemappedKeyActive[i] = false;
			}
		}

		// FIX: clear swallowed-key state so held keys don't stick after alt-tab
		for (int i = 0; i < 256; ++i)
			g_SwallowedKeyState[i].store(false, std::memory_order_relaxed);
	}

	if (m_pNullHmdLatest) m_pNullHmdLatest->RunFrame();
	if (ctrlsConnected) {
		GetControllersData(&MyCtrl, &MyCtrl2);
		if (m_pController) m_pController->RunFrame();
		if (m_pController2) m_pController2->RunFrame();
		vr::VREvent_t vrEvent;
		while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
			if (vrEvent.eventType == vr::VREvent_SceneApplicationChanged) {
				DWORD pid = vrEvent.data.process.pid;
				std::string appName = GetAppIdentifierFromPid(pid);
				LoadProfile(appName);
			}
			if (m_pController) m_pController->ProcessEvent(vrEvent);
			if (m_pController2) m_pController2->ProcessEvent(vrEvent);
		}
	}
}

HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName)) return &g_serverDriverNull;
	if (pReturnCode) *pReturnCode = VRInitError_Init_InterfaceNotFound;
	return NULL;
}
