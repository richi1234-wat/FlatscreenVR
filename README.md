<meta name="google-site-verification" content="pUz-tROVwL-bKn2jZWmQxTgerCPVS_x_Z6Q9l6P7tvQ" />
# FlatscreenVR Universal Driver (Vive + Oculus)

In a nutshell, this is a heavily modified fork of [R57Zone's No-VR driver](https://github.com/r57zone/Half-Life-Alyx-novr), more leaned on making you able to play **H3VR without a VR Headset!**

The goal was to make it as similar to the **Flatplayer** mod for Bonelab, while keeping the original author's (R57Zone) cool utility features and adding an experiment for Oculus controller support. We went from a basic mouse mapper to a full-fledged macro engine with a custom gesture animation system and an FPS mode.

**A note on AI:** I'll be blunt—I used DeepSeek to code this, while I tuned the numbers until they felt right. I'm sure it's got bugs, and I'll fix them up eventually. 

---

## ⌨️ Controls

*(You can change all of these in `FlatVR.ini`)*

### Left Controller
*   **Trigger:** Left Mouse Button
*   **Menu:** Tab
*   **Grip:** Q *(Set to TOGGLE by default)*
*   **System:** 3
*   **Center Trackpad Click:** F
*   **Trackpad:** Arrow Keys

### Right Controller
*   **Trigger:** Right Mouse Button
*   **Menu:** R
*   **Grip:** E
*   **System:** 4
*   **Center Trackpad Click:** V
*   **Trackpad:** Left = B, Right = N, Up = Middle Mouse Button, Down = Spacebar

### Oculus-Specific Bindings
*   **Left Hand:** X = T, Y = Tab
*   **Right Hand:** A = G, B = R

### Hand Movement & Rotation (6DOF)
*   **Left Hand Edit Mode:** X
*   **Right Hand Edit Mode:** C
*   **Move Hand Horizontally/Vertically:** W, A, Z, D *(Note: The screenshot button "S" is in the way by default, you can change this in `FlatVR.ini`)*
*   **Move Hand Forward/Backward:** W / Z
*   **Move Hand Left/Right:** A / D
*   **Raise/Lower Hand:** 1 (Up), 2 (Down)
*   **Rotate Hand (Twist):** Mouse Wheel
*   **Fine Tune / Slow Motion:** H
*   **Move Controllers Forward/Back:** INSERT / HOME

(this is a fragment from the HL:A No-VR driver we mangled. Still fun to use if you want unlimited reach)

### FPS Mode & Camera (New in V2.2)
*   **Toggle FPS Mode:** J *(Locks hands to the center of the screen for crosshair-aligned aiming)*
*   **Freelook:** Hold `ALT` *(Look around with your mouse while your hands stay locked)*
*   **Auto Swing (Armswinger):** Hold `SHIFT` *(Scroll wheel adjusts swing speed)*
*   **Crouch:** `CTRL` *(Set to TOGGLE by default, glides camera smoothly)*
*   **HMD Movement (Leaning):** NUMPAD 8 (Forward), NUMPAD 5 (Back), NUMPAD 4 (Left), NUMPAD 6 (Right)
*   **HMD Move Reset:** U
*   **Player Height:** PAGE-UP (Higher), PAGE-DOWN (Lower), NUMPAD-MINUS (Reset)
*   **Shotgun Pump:** MOUSE-SIDE1-BTN (Mouse 4)

### Gestures & Hand Locks
*   **Custom Gestures:** 5, 6, 7, 8, 9, 0 *(Triggers files in `Resources/Custom_Gestures/`)*
*   **Left Hand Lock:** L *(Freezes hand in world space)*
*   **Right Hand Lock:** `:` (Colon/Semicolon key)

---

## 🚧 Roadmap & Call for Ideas!

I have a lot more updates planned for FlatscreenVR (including a literal arm swinger locomotion and toggleable buttons), but honestly, I'm somewhat figuring out how C++ works by reading the code, since I am NOT learning all of it. 

I know that I'm practically talking to an amount of people that's countable on my hands, but if you have any cool ideas you think would make this driver better (or if you find a bug), **please open an Issue** or let me know! I'd love to hear what the early adopters want to see next. 

---

## 📥 Installation

1. Download the latest `.zip` from the **Releases** tab.
2. Install SteamVR from Steam if you haven't already.
3. Extract the `FlatVR` folder into your SteamVR directory: `C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\FlatVR` *(Or wherever you installed SteamVR).*
4. Boot SteamVR. A Headset Mirror should pop up, and you can finally play VR without spending a penny on a physical headset!
5. *(Optional)* Check inside `Resources\Custom_Gestures` for a `README.txt` to learn how to script your own hand animations using the Custom Gesture System.

*I know this is bare bones, but it's my first time on GitHub and I still have a feeling it is. Enjoy my modded driver!*
