//============ Copyright (c) Valve Corporation, All rights reserved. ============
//===================== FlatVR Universal Driver =======================

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
#include <atlbase.h>
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
	else return 0;
}

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
double LastHmdPos[3] = { 0, 0, 0 };
double LastHmdYaw = 0, LastHmdPitch = 0;
double HmdOmegaX = 0, HmdOmegaY = 0;

milliseconds deltaTime;

typedef struct _HMDData { double X, Y, Z, Yaw, Pitch, Roll; } THMD, * PHMD;

typedef struct _Controller
{
	double	X, Y, Z, Yaw, Pitch, Roll;
	unsigned short	Buttons;
	float	Trigger, AxisX, AxisY;
	float	GripValue;
	bool	ButtonA, ButtonB;
} TController, * PController;

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

bool FPSModeActive = false;
bool PrevFPSModeKey = false;

int KEY_ID_FREELOOK;
float FreelookYawOffset = 0.0f;
float FreelookPitchOffset = 0.0f;

bool AutoSwingActive = false;
bool AutoSwingHoldMode = false; 
bool PrevAutoSwingKey = false;
int KEY_ID_AUTO_SWING_TOGGLE;

HHOOK g_hKeyboardHook = NULL;
std::vector<int> g_DriverKeys;
std::atomic<bool> g_SwallowedKeyState[256] = { false };

bool IsKeyDown(int vkCode) {
    if (std::find(g_DriverKeys.begin(), g_DriverKeys.end(), vkCode) != g_DriverKeys.end()) {
        return g_SwallowedKeyState[vkCode];
    }
    return (GetAsyncKeyState(vkCode) & 0x8000) != 0;
}

int KEY_ID_PUMP_ACTION;
double PumpTimer = 0.0;
bool PrevPumpKey = false;

bool LeftHandLocked = false;
bool RightHandLocked = false;
bool PrevLeftLockKey = false;
bool PrevRightLockKey = false;
int KEY_ID_LEFT_HAND_LOCK, KEY_ID_RIGHT_HAND_LOCK;
double LockedLeftPos[3] = { 0, 0, 0 };
double LockedRightPos[3] = { 0, 0, 0 };
double LockedLeftYaw = 0, LockedLeftPitch = 0, LockedLeftRoll = 0;
double LockedRightYaw = 0, LockedRightPitch = 0, LockedRightRoll = 0;

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

struct GestureStep {
    std::string command;
    std::string target;
    std::string axis;
    double value = 0.0;
    double duration = 0.0;
};

struct CachedGesture {
    std::vector<GestureStep> steps;
    FILETIME lastModified;
};
std::map<std::string, CachedGesture> g_GestureCache;

FILETIME GetFileModTime(const std::string& path) {
    WIN32_FILE_ATTRIBUTE_DATA fileInfo;
    if (GetFileAttributesExA(path.c_str(), GetFileExInfoStandard, &fileInfo)) {
        return fileInfo.ftLastWriteTime;
    }
    FILETIME empty = {0, 0};
    return empty;
}

std::vector<GestureStep> ActiveSequence; 
int CurrentStepIndex = -1;               
double StepTimer = 0.0;

double SeqOffsetL[3] = { 0, 0, 0 }, SeqOffsetR[3] = { 0, 0, 0 };
double SeqRotL[3] = { 0, 0, 0 }, SeqRotR[3] = { 0, 0, 0 };

double StartSeqOffsetL[3] = { 0, 0, 0 }, StartSeqOffsetR[3] = { 0, 0, 0 };
double StartSeqRotL[3] = { 0, 0, 0 }, StartSeqRotR[3] = { 0, 0, 0 };
double StepInitialOffsetL[3] = { 0, 0, 0 }, StepInitialOffsetR[3] = { 0, 0, 0 };
double StepInitialRotL[3] = { 0, 0, 0 }, StepInitialRotR[3] = { 0, 0, 0 };

int SeqGripL = -1, SeqGripR = -1;
int SeqTrigL = -1, SeqTrigR = -1;
int SeqMenuL = -1, SeqMenuR = -1;
int SeqSysL = -1, SeqSysR = -1;
int SeqTrackpadCL = -1, SeqTrackpadCR = -1;
int SeqTrackpadUL = -1, SeqTrackpadUR = -1;
int SeqTrackpadDL = -1, SeqTrackpadDR = -1;
int SeqTrackpadLL = -1, SeqTrackpadLR = -1;
int SeqTrackpadRL = -1, SeqTrackpadRR = -1;
int SeqBtnAL = -1, SeqBtnAR = -1;
int SeqBtnBL = -1, SeqBtnBR = -1;

int KEY_ID_GESTURE_CUSTOM_5, KEY_ID_GESTURE_CUSTOM_6, KEY_ID_GESTURE_CUSTOM_7, KEY_ID_GESTURE_CUSTOM_8, KEY_ID_GESTURE_CUSTOM_9, KEY_ID_GESTURE_CUSTOM_0;

void LoadGestureSequence(const std::string& path) {
    FILETIME currentTime = GetFileModTime(path);
    
    auto it = g_GestureCache.find(path);
    if (it != g_GestureCache.end()) {
        if (CompareFileTime(&it->second.lastModified, &currentTime) == 0) {
            ActiveSequence = it->second.steps;
            return;
        }
    }

    ActiveSequence.clear();
    std::ifstream file(path);
    std::string line;
    
    if (!file.is_open()) return; 
    
    while (std::getline(file, line)) {
        size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) line = line.substr(0, commentPos);
        
        std::stringstream ss(line);
        GestureStep step;
        ss >> step.command;
        
        if (step.command == "WAIT") {
            ss >> step.duration;
        } else if (step.command == "RETURN") {
            ss >> step.duration;
        } else if (step.command == "MOVE" || step.command == "ROTATE") {
            ss >> step.target >> step.axis >> step.value >> step.duration;
        } else if (step.command == "BUTTON") {
            ss >> step.target >> step.axis >> step.value >> step.duration;
        } else continue; 

        if (step.duration > 0) {
            ActiveSequence.push_back(step);
        }
    }
    file.close();

    g_GestureCache[path] = { ActiveSequence, currentTime };
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

std::string g_SteamPathStr = ""; 

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

        if (SteamVRHeadsetWindow != 0 && SteamVRHeadsetWindow == GetForegroundWindow()) {
            if (std::find(g_DriverKeys.begin(), g_DriverKeys.end(), vkCode) != g_DriverKeys.end()) {
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

void GetHMDData(__out THMD* HMD)
{
	if (IsKeyDown(KEY_ID_PLAYER_RISE_HIGHER)) HMDPosZ += StepPos;
	if (IsKeyDown(KEY_ID_PLAYER_RISE_LOWER)) HMDPosZ -= StepPos;
	if (IsKeyDown(KEY_ID_PLAYER_RISE_RESET)) HMDPosZ = 0;

	double moveSpeed = 0.01;
	double yawRad = DegToRad(cameraHMD.Yaw);

	if (IsKeyDown(KEY_ID_HMD_FORWARD)) { HMDPosX -= sin(yawRad) * moveSpeed; HMDPosY -= cos(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_BACK))    { HMDPosX += sin(yawRad) * moveSpeed; HMDPosY += cos(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_LEFT))    { HMDPosX -= cos(yawRad) * moveSpeed; HMDPosY += sin(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_RIGHT))   { HMDPosX += cos(yawRad) * moveSpeed; HMDPosY -= sin(yawRad) * moveSpeed; }
	if (IsKeyDown(KEY_ID_HMD_MOVE_RESET)) { HMDPosX = 0; HMDPosY = 0; }

	HMD->X = HMDPosX; 
	HMD->Y = HMDPosY; 
	HMD->Z = HMDPosZ - CrouchOffsetZ;
	HMD->Yaw = (cameraHMD.Yaw + FreelookYawOffset) * -1; 
    HMD->Pitch = (cameraHMD.Pitch + FreelookPitchOffset) * -1; 
    HMD->Roll = 0;
}

void GetControllersData(__out TController* FirstController, __out TController* SecondController)
{
	float dt = max((int)deltaTime.count(), 1) / 1000.0f;
	
	FirstController->AxisX = 0; FirstController->AxisY = 0; FirstController->Buttons = 0; FirstController->Trigger = 0;
	SecondController->AxisX = 0; SecondController->AxisY = 0; SecondController->Buttons = 0; SecondController->Trigger = 0;
	SecondCtrlYPROffset[0] = 0; SecondCtrlYPROffset[1] = 0; SecondCtrlYPROffset[2] = 0;
	FirstController->Roll = 0;
	SecondController->Roll = 0;

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
		} else {
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
		if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_UP)) LeftRotateYOffset += currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_DOWN)) LeftRotateYOffset -= currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_HAND_FORWARD)) { LeftZOffset -= currentSensitivity * dt; LeftZOffset_Target -= currentSensitivity * dt; }
		if (IsKeyDown(KEY_ID_HAND_BACK)) { LeftZOffset += currentSensitivity * dt; LeftZOffset_Target += currentSensitivity * dt; }
		if (IsKeyDown(KEY_ID_HAND_LEFT)) LeftXOffset -= currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_HAND_RIGHT)) LeftXOffset += currentSensitivity * dt;
		if (wheelDelta != 0) LeftHandRotOffset[2] += wheelDelta * wheelSensitivity; 
	}
	if (RightHandIsRotating) {
		if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_UP)) RightYOffset += currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_MOTION_MODE_MOVE_DOWN)) RightYOffset -= currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_HAND_FORWARD)) { RightZOffset -= currentSensitivity * dt; RightZOffset_Target -= currentSensitivity * dt; }
		if (IsKeyDown(KEY_ID_HAND_BACK)) { RightZOffset += currentSensitivity * dt; RightZOffset_Target += currentSensitivity * dt; }
		if (IsKeyDown(KEY_ID_HAND_LEFT)) RightXOffset -= currentSensitivity * dt;
		if (IsKeyDown(KEY_ID_HAND_RIGHT)) RightXOffset += currentSensitivity * dt;
		if (wheelDelta != 0) RightHandRotOffset[2] += wheelDelta * wheelSensitivity;
	}

	if (IsKeyDown(KEY_ID_MOVE_CONTROLLERS_FORWARD)) HandRadDist += 0.005;
	if (IsKeyDown(KEY_ID_MOVE_CONTROLLERS_BACK)) HandRadDist -= 0.005;
	if (!IsKeyDown(KEY_ID_MOVE_CONTROLLERS_FORWARD) && !IsKeyDown(KEY_ID_MOVE_CONTROLLERS_BACK)) HandRadDist = 0;

	double yawRad = DegToRad(cameraHMD.Yaw);

	if (FPSModeActive) {
		double gunX = FPS_GunOffsetX + RightXOffset;
		double gunY = FPS_GunOffsetY + RightYOffset;
		double gunZ = FPS_GunOffsetZ - RightZOffset;

		double supX = FPS_LeftOffsetX + LeftXOffset;
		double supY = FPS_LeftOffsetY + LeftRotateYOffset;
		double supZ = FPS_LeftOffsetZ - LeftZOffset;

		// *** NEW: Apply Gesture MOVE Offsets to FPS Mode ***
		gunX += SeqOffsetR[0];
		gunY += SeqOffsetR[2];
		gunZ -= SeqOffsetR[1];

		supX += SeqOffsetL[0];
		supY += SeqOffsetL[2];
		supZ -= SeqOffsetL[1];
		// *************************************************

		if (AutoSwingActive) {
			ArmSwingTime += dt * ArmSwingSpeed;
			double swingForward = sin(ArmSwingTime) * ArmSwingDistance;
			gunZ += swingForward;
			supZ += swingForward;
		}

		if (PumpTimer > 0.0) {
			PumpTimer -= dt;
			if (PumpTimer < 0.0) PumpTimer = 0.0;
			double pumpProgress = 1.0 - (PumpTimer / 0.35);
			double pumpOffset = sin(pumpProgress * 3.141592653589793) * 0.15;
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

		double gYaw = DegToRad(FPS_GunZeroYaw + RightHandRotOffset[0] + SeqRotR[0]);
		double gPitch = DegToRad(FPS_GunZeroPitch + RightHandRotOffset[1] + SeqRotR[1]);

		double f1x = cam_fx * cos(gYaw) + cam_rx * sin(gYaw);
		double f1y = cam_fy * cos(gYaw) + cam_ry * sin(gYaw);
		double f1z = cam_fz * cos(gYaw) + cam_rz * sin(gYaw);

		double f2x = f1x * cos(gPitch) + cam_ux * sin(gPitch);
		double f2y = f1y * cos(gPitch) + cam_uy * sin(gPitch);
		double f2z = f1z * cos(gPitch) + cam_uz * sin(gPitch);

		SecondController->Yaw   = -atan2(-f2x, -f2z) * 57.29577951308232;
		SecondController->Pitch = -asin(f2y) * 57.29577951308232;
		SecondController->Roll  = FPS_GunZeroRoll + RightHandRotOffset[2] + SeqRotR[2];

		double lYaw = DegToRad(FPS_LeftZeroYaw + LeftHandRotOffset[0] + SeqRotL[0]);
		double lPitch = DegToRad(FPS_LeftZeroPitch + LeftHandRotOffset[1] + SeqRotL[1]);

		double lf1x = cam_fx * cos(lYaw) + cam_rx * sin(lYaw);
		double lf1y = cam_fy * cos(lYaw) + cam_ry * sin(lYaw);
		double lf1z = cam_fz * cos(lYaw) + cam_rz * sin(lYaw);

		double lf2x = lf1x * cos(lPitch) + cam_ux * sin(lPitch);
		double lf2y = lf1y * cos(lPitch) + cam_uy * sin(lPitch);
		double lf2z = lf1z * cos(lPitch) + cam_uz * sin(lPitch);

		FirstController->Yaw    = -atan2(-lf2x, -lf2z) * 57.29577951308232;
		FirstController->Pitch  = -asin(lf2y) * 57.29577951308232;
		FirstController->Roll   = FPS_LeftZeroRoll + LeftHandRotOffset[2] + SeqRotL[2];

	} else {
		FirstCtrlPos[0] = (0.4 - LeftRadDist + HandRadDist) * sin(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, -17), LeftAngleDist)));
		FirstCtrlPos[1] = (0.4 - LeftRadDist + HandRadDist) * cos(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, -17), LeftAngleDist)));
		FirstCtrlPos[2] = cameraHMD.Pitch * ZPosSensetive - CrouchOffsetZ + HMDPosZ;

		double moveX_L = LeftXOffset * cos(yawRad) + LeftZOffset * sin(yawRad);
		double moveZ_L = -LeftXOffset * sin(yawRad) + LeftZOffset * cos(yawRad);

		FirstController->X = FirstCtrlPos[0] + moveX_L + HandCenterOffsetX + HMDPosX;
		FirstController->Y = FirstCtrlPos[1] + moveZ_L + HMDPosY;
		FirstController->Z = FirstCtrlPos[2] + LeftRotateYOffset - 0.15;

		SecondCtrlPos[0] = (0.4 - RightRadDist + HandRadDist) * sin(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, 17), RightAngleDist)));
		SecondCtrlPos[1] = (0.4 - RightRadDist + HandRadDist) * cos(DegToRad(OffsetYPR(OffsetYPR(cameraHMD.Yaw + 180, 17), RightAngleDist)));
		SecondCtrlPos[2] = cameraHMD.Pitch * ZPosSensetive - CrouchOffsetZ + HMDPosZ;

		double moveX_R = RightXOffset * cos(yawRad) + RightZOffset * sin(yawRad);
		double moveZ_R = -RightXOffset * sin(yawRad) + RightZOffset * cos(yawRad);

		SecondController->X = SecondCtrlPos[0] + moveX_R + HandCenterOffsetX + HMDPosX;
		SecondController->Y = SecondCtrlPos[1] + moveZ_R + HMDPosY;
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
	if (CheckButton(KEY_ID_LEFT_TRIGGER, Btn_LeftTrigger)) FirstController->Trigger = 1;
	if (CheckButton(KEY_ID_LEFT_SYSTEM, Btn_LeftSystem)) FirstController->Buttons |= SYS_BTN;

	bool leftPadActive = false;
	if (CheckButton(KEY_ID_UP, Btn_Up)) { FirstController->AxisY = 1; leftPadActive = true; }
	if (CheckButton(KEY_ID_DOWN, Btn_Down)) { FirstController->AxisY = -1; leftPadActive = true; }
	if (CheckButton(KEY_ID_LEFT, Btn_Left)) { FirstController->AxisX = -1; leftPadActive = true; }
	if (CheckButton(KEY_ID_RIGHT, Btn_Right)) { FirstController->AxisX = 1; leftPadActive = true; }
	if (CheckButton(KEY_ID_LEFT_TRACKPAD_CLICK, Btn_LeftTrackpadClick)) leftPadActive = true;
	if (leftPadActive) FirstController->Buttons |= THUMB_BTN; 

	bool rightPadActive = false;
	if (CheckButton(KEY_ID_TURN_LEFT, Btn_TurnLeft)) { SecondController->AxisX = -1; rightPadActive = true; }
	if (CheckButton(KEY_ID_TURN_RIGHT, Btn_TurnRight)) { SecondController->AxisX = 1; rightPadActive = true; }
	if (CheckButton(KEY_ID_TELEPORT, Btn_Teleport)) { SecondController->AxisY = -1; rightPadActive = true; }
	if (CheckButton(KEY_ID_CHANGE_WEAPON, Btn_ChangeWeapon)) { SecondController->AxisY = 1; rightPadActive = true; }
	if (CheckButton(KEY_ID_RIGHT_TRACKPAD_CLICK, Btn_RightTrackpadClick)) rightPadActive = true;
	if (rightPadActive) SecondController->Buttons |= THUMB_BTN;

	if (CheckButton(KEY_ID_RIGHT_MENU, Btn_RightMenu)) SecondController->Buttons |= MENU_BTN;
	if (CheckButton(KEY_ID_RIGHT_GRIP, Btn_RightGrip)) SecondController->Buttons |= GRIP_BTN;
	if (CheckButton(KEY_ID_RIGHT_SYSTEM, Btn_RightSystem)) SecondController->Buttons |= SYS_BTN;
	if (CheckButton(KEY_ID_RIGHT_TRIGGER, Btn_RightTrigger)) SecondController->Trigger = 1;

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

	if (CurrentStepIndex == -1) {
		if (HeadsetWindowFocused) {
			std::string gestBasePath = (std::string)g_SteamPathStr + "/steamapps/common/SteamVR/drivers/FlatVR/Resources/Custom_Gestures/";
			
			if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_5)) { LoadGestureSequence(gestBasePath + "5.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_6)) { LoadGestureSequence(gestBasePath + "6.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_7)) { LoadGestureSequence(gestBasePath + "7.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_8)) { LoadGestureSequence(gestBasePath + "8.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_9)) { LoadGestureSequence(gestBasePath + "9.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			else if (IsKeyDown(KEY_ID_GESTURE_CUSTOM_0)) { LoadGestureSequence(gestBasePath + "0.gesture"); if (!ActiveSequence.empty()) CurrentStepIndex = 0; }
			
			if (CurrentStepIndex == 0) {
				StepTimer = 0.0;
				StartSeqOffsetL[0] = SeqOffsetL[0]; StartSeqOffsetL[1] = SeqOffsetL[1]; StartSeqOffsetL[2] = SeqOffsetL[2];
				StartSeqOffsetR[0] = SeqOffsetR[0]; StartSeqOffsetR[1] = SeqOffsetR[1]; StartSeqOffsetR[2] = SeqOffsetR[2];
				StartSeqRotL[0] = SeqRotL[0]; StartSeqRotL[1] = SeqRotL[1]; StartSeqRotL[2] = SeqRotL[2];
				StartSeqRotR[0] = SeqRotR[0]; StartSeqRotR[1] = SeqRotR[1]; StartSeqRotR[2] = SeqRotR[2];

				SeqGripL = SeqGripR = SeqTrigL = SeqTrigR = -1;
				SeqMenuL = SeqMenuR = SeqSysL = SeqSysR = -1;
				SeqTrackpadCL = SeqTrackpadCR = SeqTrackpadUL = SeqTrackpadUR = -1;
				SeqTrackpadDL = SeqTrackpadDR = SeqTrackpadLL = SeqTrackpadLR = -1;
				SeqTrackpadRL = SeqTrackpadRR = SeqBtnAL = SeqBtnAR = SeqBtnBL = SeqBtnBR = -1;
			}
		}
	}

	if (CurrentStepIndex >= 0 && CurrentStepIndex < ActiveSequence.size()) {
		GestureStep& step = ActiveSequence[CurrentStepIndex];
		
		if (StepTimer == 0.0) {
			StepInitialOffsetL[0] = SeqOffsetL[0]; StepInitialOffsetL[1] = SeqOffsetL[1]; StepInitialOffsetL[2] = SeqOffsetL[2];
			StepInitialOffsetR[0] = SeqOffsetR[0]; StepInitialOffsetR[1] = SeqOffsetR[1]; StepInitialOffsetR[2] = SeqOffsetR[2];
			StepInitialRotL[0] = SeqRotL[0]; StepInitialRotL[1] = SeqRotL[1]; StepInitialRotL[2] = SeqRotL[2];
			StepInitialRotR[0] = SeqRotR[0]; StepInitialRotR[1] = SeqRotR[1]; StepInitialRotR[2] = SeqRotR[2];
		}

		StepTimer += dt; 

		double progress = StepTimer / step.duration;
		if (progress > 1.0) progress = 1.0;

		if (step.command == "MOVE") {
			if (step.target == "L" || step.target == "B") {
				if (step.axis == "X") SeqOffsetL[0] = StepInitialOffsetL[0] + (step.value * progress);
				if (step.axis == "Y") SeqOffsetL[1] = StepInitialOffsetL[1] + (step.value * progress);
				if (step.axis == "Z") SeqOffsetL[2] = StepInitialOffsetL[2] + (step.value * progress);
			}
			if (step.target == "R" || step.target == "B") {
				if (step.axis == "X") SeqOffsetR[0] = StepInitialOffsetR[0] + (step.value * progress);
				if (step.axis == "Y") SeqOffsetR[1] = StepInitialOffsetR[1] + (step.value * progress);
				if (step.axis == "Z") SeqOffsetR[2] = StepInitialOffsetR[2] + (step.value * progress);
			}
		}
		else if (step.command == "ROTATE") {
			if (step.target == "L" || step.target == "B") {
				if (step.axis == "YAW") SeqRotL[0] = StepInitialRotL[0] + (step.value * progress);
				if (step.axis == "PITCH") SeqRotL[1] = StepInitialRotL[1] + (step.value * progress);
				if (step.axis == "ROLL") SeqRotL[2] = StepInitialRotL[2] + (step.value * progress);
			}
			if (step.target == "R" || step.target == "B") {
				if (step.axis == "YAW") SeqRotR[0] = StepInitialRotR[0] + (step.value * progress);
				if (step.axis == "PITCH") SeqRotR[1] = StepInitialRotR[1] + (step.value * progress);
				if (step.axis == "ROLL") SeqRotR[2] = StepInitialRotR[2] + (step.value * progress);
			}
		}
		else if (step.command == "RETURN") {
			SeqOffsetL[0] = StepInitialOffsetL[0] + (StartSeqOffsetL[0] - StepInitialOffsetL[0]) * progress;
			SeqOffsetL[1] = StepInitialOffsetL[1] + (StartSeqOffsetL[1] - StepInitialOffsetL[1]) * progress;
			SeqOffsetL[2] = StepInitialOffsetL[2] + (StartSeqOffsetL[2] - StepInitialOffsetL[2]) * progress;
			SeqRotL[0] = StepInitialRotL[0] + (StartSeqRotL[0] - StepInitialRotL[0]) * progress;
			SeqRotL[1] = StepInitialRotL[1] + (StartSeqRotL[1] - StepInitialRotL[1]) * progress;
			SeqRotL[2] = StepInitialRotL[2] + (StartSeqRotL[2] - StepInitialRotL[2]) * progress;

			SeqOffsetR[0] = StepInitialOffsetR[0] + (StartSeqOffsetR[0] - StepInitialOffsetR[0]) * progress;
			SeqOffsetR[1] = StepInitialOffsetR[1] + (StartSeqOffsetR[1] - StepInitialOffsetR[1]) * progress;
			SeqOffsetR[2] = StepInitialOffsetR[2] + (StartSeqOffsetR[2] - StepInitialOffsetR[2]) * progress;
			SeqRotR[0] = StepInitialRotR[0] + (StartSeqRotR[0] - StepInitialRotR[0]) * progress;
			SeqRotR[1] = StepInitialRotR[1] + (StartSeqRotR[1] - StepInitialRotR[1]) * progress;
			SeqRotR[2] = StepInitialRotR[2] + (StartSeqRotR[2] - StepInitialRotR[2]) * progress;
		}
		else if (step.command == "BUTTON") {
			int btnState = step.value > 0.0 ? 1 : 0;
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

		if (StepTimer >= step.duration) {
			StepTimer = 0.0;
			CurrentStepIndex++;
			
			if (CurrentStepIndex >= ActiveSequence.size()) {
				CurrentStepIndex = -1; 
				
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

	if (SeqGripL == 1) FirstController->Buttons |= GRIP_BTN;
	else if (SeqGripL == 0) FirstController->Buttons &= ~GRIP_BTN;
	if (SeqTrigL == 1) FirstController->Trigger = 1.0f;
	else if (SeqTrigL == 0) FirstController->Trigger = 0.0f;
	if (SeqMenuL == 1) FirstController->Buttons |= MENU_BTN;
	else if (SeqMenuL == 0) FirstController->Buttons &= ~MENU_BTN;
	if (SeqSysL == 1) FirstController->Buttons |= SYS_BTN;
	else if (SeqSysL == 0) FirstController->Buttons &= ~SYS_BTN;
	if (SeqBtnAL == 1) FirstController->ButtonA = true;
	else if (SeqBtnAL == 0) FirstController->ButtonA = false;
	if (SeqBtnBL == 1) FirstController->ButtonB = true;
	else if (SeqBtnBL == 0) FirstController->ButtonB = false;

	if (SeqTrackpadCL == 1) FirstController->Buttons |= THUMB_BTN;
	else if (SeqTrackpadCL == 0 && SeqTrackpadUL != 1 && SeqTrackpadDL != 1 && SeqTrackpadLL != 1 && SeqTrackpadRL != 1) FirstController->Buttons &= ~THUMB_BTN;
	if (SeqTrackpadUL == 1) { FirstController->AxisY = 1; FirstController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadUL == 0 && SeqTrackpadDL != 1) FirstController->AxisY = 0;
	if (SeqTrackpadDL == 1) { FirstController->AxisY = -1; FirstController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadDL == 0 && SeqTrackpadUL != 1) FirstController->AxisY = 0;
	if (SeqTrackpadRL == 1) { FirstController->AxisX = 1; FirstController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadRL == 0 && SeqTrackpadLL != 1) FirstController->AxisX = 0;
	if (SeqTrackpadLL == 1) { FirstController->AxisX = -1; FirstController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadLL == 0 && SeqTrackpadRL != 1) FirstController->AxisX = 0;

	if (SeqGripR == 1) SecondController->Buttons |= GRIP_BTN;
	else if (SeqGripR == 0) SecondController->Buttons &= ~GRIP_BTN;
	if (SeqTrigR == 1) SecondController->Trigger = 1.0f;
	else if (SeqTrigR == 0) SecondController->Trigger = 0.0f;
	if (SeqMenuR == 1) SecondController->Buttons |= MENU_BTN;
	else if (SeqMenuR == 0) SecondController->Buttons &= ~MENU_BTN;
	if (SeqSysR == 1) SecondController->Buttons |= SYS_BTN;
	else if (SeqSysR == 0) SecondController->Buttons &= ~SYS_BTN;
	if (SeqBtnAR == 1) SecondController->ButtonA = true;
	else if (SeqBtnAR == 0) SecondController->ButtonA = false;
	if (SeqBtnBR == 1) SecondController->ButtonB = true;
	else if (SeqBtnBR == 0) SecondController->ButtonB = false;

	if (SeqTrackpadCR == 1) SecondController->Buttons |= THUMB_BTN;
	else if (SeqTrackpadCR == 0 && SeqTrackpadUR != 1 && SeqTrackpadDR != 1 && SeqTrackpadLR != 1 && SeqTrackpadRR != 1) SecondController->Buttons &= ~THUMB_BTN;
	if (SeqTrackpadUR == 1) { SecondController->AxisY = 1; SecondController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadUR == 0 && SeqTrackpadDR != 1) SecondController->AxisY = 0;
	if (SeqTrackpadDR == 1) { SecondController->AxisY = -1; SecondController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadDR == 0 && SeqTrackpadUR != 1) SecondController->AxisY = 0;
	if (SeqTrackpadRR == 1) { SecondController->AxisX = 1; SecondController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadRR == 0 && SeqTrackpadLR != 1) SecondController->AxisX = 0;
	if (SeqTrackpadLR == 1) { SecondController->AxisX = -1; SecondController->Buttons |= THUMB_BTN; }
	else if (SeqTrackpadLR == 0 && SeqTrackpadRR != 1) SecondController->AxisX = 0;

	if (LeftHandLocked) {
		FirstController->X = LockedLeftPos[0];
		FirstController->Y = LockedLeftPos[1];
		FirstController->Z = LockedLeftPos[2];
		if (!LeftHandIsRotating) {
			FirstController->Yaw = LockedLeftYaw;
			FirstController->Pitch = LockedLeftPitch;
			FirstController->Roll = LockedLeftRoll;
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
		}
	}
}

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

    AutoSwingHoldMode = IniFile.ReadBoolean("FPS_Mode", "AutoSwingHoldMode", IniFile.ReadBoolean("Main", "AutoSwingHoldMode", true));

    g_DriverKeys.clear(); 
    std::string swallowStr = IniFile.ReadString("SwallowKeys", "Keys", "");
    std::stringstream ss(swallowStr);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty()) {
            int code = KeyNameToKeyCode(token);
            if (code != 0) g_DriverKeys.push_back(code);
        }
    }

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
    KEY_ID_FINE_TUNE = KeyNameToKeyCode(IniFile.ReadString("Keys", "FINE_TUNE", "H"));
    KEY_ID_FPS_MODE_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "FPS_MODE_TOGGLE", "J"));
    KEY_ID_AUTO_SWING_TOGGLE = KeyNameToKeyCode(IniFile.ReadString("Keys", "AUTO_SWING_TOGGLE", "SHIFT"));
    KEY_ID_PUMP_ACTION = KeyNameToKeyCode(IniFile.ReadString("Keys", "PUMP_ACTION", "MOUSE-SIDE1-BTN"));
    
    KEY_ID_LEFT_HAND_LOCK = KeyNameToKeyCode(IniFile.ReadString("Keys", "LEFT_HAND_LOCK", "L"));
    KEY_ID_RIGHT_HAND_LOCK = KeyNameToKeyCode(IniFile.ReadString("Keys", "RIGHT_HAND_LOCK", "SEMICOLON"));

    KEY_ID_HMD_FORWARD = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_FORWARD", "NUMPAD8"));
    KEY_ID_HMD_BACK = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_BACK", "NUMPAD5"));
    KEY_ID_HMD_LEFT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_LEFT", "NUMPAD4"));
    KEY_ID_HMD_RIGHT = KeyNameToKeyCode(IniFile.ReadString("Keys", "HMD_RIGHT", "NUMPAD6"));
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
}

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
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_ContainsProximitySensor_Bool, false);
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
			pose.qRotation = EulerAngleToQuaternion(DegToRad(-MyHMD.Yaw), DegToRad(-MyHMD.Pitch), DegToRad(MyHMD.Roll)); 
			pose.vecPosition[0] = MyHMD.X; 
			pose.vecPosition[1] = MyHMD.Z; 
			pose.vecPosition[2] = MyHMD.Y; 
			
			double dt = max((int)deltaTime.count(), 1) / 1000.0;
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
		pose.shouldApplyHeadModel = !FPSModeActive;
		pose.poseTimeOffset = 0; pose.willDriftInYaw = false; return pose;
	}
	void RunFrame() { if (HMDIndex_t != vr::k_unTrackedDeviceIndexInvalid) vr::VRServerDriverHost()->TrackedDevicePoseUpdated(HMDIndex_t, GetPose(), sizeof(DriverPose_t)); }
	std::string GetSerialNumber() const { return m_sSerialNumber; }
private:
	vr::TrackedDeviceIndex_t HMDIndex_t; vr::PropertyContainerHandle_t m_ulPropertyContainer;
	std::string m_sSerialNumber, m_sModelNumber;
	int32_t m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight, m_nRenderWidth, m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons, m_flDisplayFrequency, m_flIPD, m_fZoomWidth, m_fZoomHeight, m_fFOV;
	bool m_bStereoMode, m_bDebugMode;
};

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
		
		double dt = max((int)deltaTime.count(), 1) / 1000.0;

		if (ControllerIndex == 1) {
			pose.vecPosition[0] = MyCtrl.X; pose.vecPosition[1] = MyCtrl.Z; pose.vecPosition[2] = MyCtrl.Y;
			
			double rx = MyCtrl.X - HMDPosX;
			double ry = MyCtrl.Z - (HMDPosZ - CrouchOffsetZ);
			double rz = MyCtrl.Y - HMDPosY;

			double fakeVX = HmdOmegaY * rz;
			double fakeVY = -HmdOmegaX * rz;
			double fakeVZ = HmdOmegaX * ry - HmdOmegaY * rx;

			double trueVX = (pose.vecPosition[0] - FirstCtrlLastPos[0]) / dt;
			double trueVY = (pose.vecPosition[1] - FirstCtrlLastPos[1]) / dt;
			double trueVZ = (pose.vecPosition[2] - FirstCtrlLastPos[2]) / dt;

			if (std::abs(trueVX - fakeVX) > 0.5 || std::abs(trueVY - fakeVY) > 0.5 || std::abs(trueVZ - fakeVZ) > 0.5) {
				pose.vecVelocity[0] = trueVX; pose.vecVelocity[1] = trueVY; pose.vecVelocity[2] = trueVZ;
			} else {
				pose.vecVelocity[0] = fakeVX; pose.vecVelocity[1] = fakeVY; pose.vecVelocity[2] = fakeVZ;
			}

			FirstCtrlLastPos[0] = pose.vecPosition[0]; FirstCtrlLastPos[1] = pose.vecPosition[1]; FirstCtrlLastPos[2] = pose.vecPosition[2];
			
			pose.vecAngularVelocity[0] = HmdOmegaX; pose.vecAngularVelocity[1] = HmdOmegaY; pose.vecAngularVelocity[2] = 0;
			
			pose.qRotation = EulerAngleToQuaternion(DegToRad(-MyCtrl.Yaw), DegToRad(-MyCtrl.Pitch), DegToRad(MyCtrl.Roll));
		}
		else {
			pose.vecPosition[0] = MyCtrl2.X; pose.vecPosition[1] = MyCtrl2.Z; pose.vecPosition[2] = MyCtrl2.Y;
			
			double rx = MyCtrl2.X - HMDPosX;
			double ry = MyCtrl2.Z - (HMDPosZ - CrouchOffsetZ);
			double rz = MyCtrl2.Y - HMDPosY;

			double fakeVX = HmdOmegaY * rz;
			double fakeVY = -HmdOmegaX * rz;
			double fakeVZ = HmdOmegaX * ry - HmdOmegaY * rx;

			double trueVX = (pose.vecPosition[0] - SecondCtrlLastPos[0]) / dt;
			double trueVY = (pose.vecPosition[1] - SecondCtrlLastPos[1]) / dt;
			double trueVZ = (pose.vecPosition[2] - SecondCtrlLastPos[2]) / dt;

			if (std::abs(trueVX - fakeVX) > 0.5 || std::abs(trueVY - fakeVY) > 0.5 || std::abs(trueVZ - fakeVZ) > 0.5) {
				pose.vecVelocity[0] = trueVX; pose.vecVelocity[1] = trueVY; pose.vecVelocity[2] = trueVZ;
			} else {
				pose.vecVelocity[0] = fakeVX; pose.vecVelocity[1] = fakeVY; pose.vecVelocity[2] = fakeVZ;
			}

			SecondCtrlLastPos[0] = pose.vecPosition[0]; SecondCtrlLastPos[1] = pose.vecPosition[1]; SecondCtrlLastPos[2] = pose.vecPosition[2];
			
			pose.vecAngularVelocity[0] = HmdOmegaX; pose.vecAngularVelocity[1] = HmdOmegaY; pose.vecAngularVelocity[2] = 0;
			
			pose.qRotation = EulerAngleToQuaternion(DegToRad(-MyCtrl2.Yaw), DegToRad(-MyCtrl2.Pitch), DegToRad(MyCtrl2.Roll));
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
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], (MyCtrl.Buttons & THUMB_BTN) != 0, 0);
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
				vr::VRDriverInput()->UpdateBooleanComponent(HButtons[3], (MyCtrl2.Buttons & THUMB_BTN) != 0, 0);
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

EVRInitError CServerDriver_Sample::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	CRegKey key; TCHAR steamPath[MAX_PATH]; LONG status = key.Open(HKEY_CURRENT_USER, _T("Software\\Valve\\Steam"));
	if (status == ERROR_SUCCESS) { ULONG regSize = sizeof(steamPath); status = key.QueryStringValue(_T("SteamPath"), steamPath, &regSize); }
	key.Close();

	g_SteamPathStr = std::string(steamPath);

	TCHAR configPath[MAX_PATH] = { 0 };
	_tcscat_s(configPath, sizeof(configPath), steamPath);
	_tcscat_s(configPath, sizeof(configPath), _T("/steamapps/common/SteamVR/drivers/FlatVR/FlatVR.ini"));

	g_IniPathStr = std::string(configPath);
	
	ctrlsConnected = true;
	
	LoadConfig();

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

void CServerDriver_Sample::RunFrame()
{
	static milliseconds lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	deltaTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()) - lastMillis;
	lastMillis = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	
	static FILETIME lastIniMod = GetFileModTime(g_IniPathStr);
	FILETIME currentIniMod = GetFileModTime(g_IniPathStr);
	
	if (CompareFileTime(&lastIniMod, &currentIniMod) != 0) {
		lastIniMod = currentIniMod;
		LoadConfig(); 
	}

	SteamVRHeadsetWindow = FindWindow(NULL, "Headset Window");

	if (IsKeyDown(VK_F1) || IsKeyDown(VK_F7)) SetForegroundWindow(SteamVRHeadsetWindow);

	HeadsetWindowFocused = SteamVRHeadsetWindow != 0 && SteamVRHeadsetWindow == GetForegroundWindow() && IsWindowVisible(SteamVRHeadsetWindow);

	if (HeadsetWindowFocused) {
		bool lr = IsKeyDown(KEY_ID_LEFT_HAND_ROTATE);
		bool rr = IsKeyDown(KEY_ID_RIGHT_HAND_ROTATE);
		
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
			}
		}
		PrevLeftLockKey = lockL;

		bool lockR = IsKeyDown(KEY_ID_RIGHT_HAND_LOCK);
		if (lockR && !PrevRightLockKey) {
			RightHandLocked = !RightHandLocked;
			if (RightHandLocked) {
				LockedRightPos[0] = MyCtrl2.X; LockedRightPos[1] = MyCtrl2.Y; LockedRightPos[2] = MyCtrl2.Z;
				LockedRightYaw = MyCtrl2.Yaw; LockedRightPitch = MyCtrl2.Pitch; LockedRightRoll = MyCtrl2.Roll;
			}
		}
		PrevRightLockKey = lockR;

		bool pumpKey = IsKeyDown(KEY_ID_PUMP_ACTION);
		if (pumpKey && !PrevPumpKey) {
			if (PumpTimer <= 0.0) {
				PumpTimer = 0.35;
			}
		}
		PrevPumpKey = pumpKey;

		float rdt = max((int)deltaTime.count(), 1) / 1000.0f;
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
	}

	if (m_pNullHmdLatest) m_pNullHmdLatest->RunFrame();
	if (ctrlsConnected) {
		GetControllersData(&MyCtrl, &MyCtrl2);
		if (m_pController) m_pController->RunFrame();
		if (m_pController2) m_pController2->RunFrame();
		vr::VREvent_t vrEvent;
		while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent))) {
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
