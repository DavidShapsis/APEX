# APEX Mission Control for iOS

Native SwiftUI client for the APEX quadruped's web dashboard (`Code/Pi5/stream_server.py`).
Built to compile on **Xcode 15.4** (iOS 17.5 SDK, Swift 5.10). Classic project format,
deployment target iOS 17.0, no iOS 18+ APIs. The compiled app runs on newer iOS, including 27.2.

## Build and install (no Run button, no developer disk image)

1. Open `APEX.xcodeproj`.
2. Settings > Accounts: add your Apple ID.
3. Target APEX > Signing & Capabilities: Automatically manage signing, Personal Team.
   The bundle ID is `com.davshapsis1.apex`. Change it if Xcode says it is taken.
4. Select the connected iPhone as the destination and press Command+B (build only).
5. Product > Show Build Folder in Finder > Products > Debug-iphoneos. Copy `APEX.app` to `~/Documents`.
6. Package and install (phone unlocked, Developer Mode on, Trust accepted):

       source ~/pmd3env/bin/activate
       mkdir -p /tmp/ipa_build/Payload && rm -rf /tmp/ipa_build/Payload/* && cp -R ~/Documents/APEX.app /tmp/ipa_build/Payload/
       (cd /tmp/ipa_build && rm -f ~/Documents/APEX.ipa && zip -r -y ~/Documents/APEX.ipa Payload)
       pymobiledevice3 apps install ~/Documents/APEX.ipa

7. First launch: Settings > General > VPN & Device Management > trust your Apple ID, then open APEX.
   Allow **Local Network** when asked, or the app cannot reach the robot.
8. Free signing expires after 7 days. Rebuild and reinstall.

## Connect button

While the robot is unreachable, a red **Connect** button shows in the top right and in the
"Robot not reachable" banner. It opens iOS Wi-Fi settings through the private `App-Prefs` URL
(`SettingsOpener.swift`), so you can join the robot's network. Where it lands depends on the iOS
version. Join APEXWifi once and iOS auto-joins it whenever it is in range. The app cannot join
a network by itself: that needs the Hotspot Configuration capability, which a free Personal
Team does not offer.

## Layout

| Tab      | Contents |
|----------|----------|
| Drive    | Status pills, banners, live MJPEG feed (corner button for full screen with controls), Snapshot and Record buttons that save to Photos, steering, Home x4 / Stand / Go / Stop |
| Navigate | Robot GPS, route map (opens on the robot; Fit centers on it; Track follows it and rotates the map to its compass heading; tap to add waypoints), NAV MODE Start / Pause / Stop, route editor |
| Systems  | Sensor Readout (all of `/api/state`: INA226 Pi 5 V rail current/voltage/power, GPS, subsystems, robot, navigation, legs), obstacle avoidance, subsystem health, motor currents, leg deactivation |
| Settings | Robot address (default `apex.local`), connection details, icon toggle (A.P.E.X. logo or robot render), open web dashboard in your browser |

## Files

| File | Role |
|------|------|
| `ApexClient.swift` | HTTP calls to every Flask route |
| `Models.swift` | `ApexStatus` from `/status`, waypoint drafts, avoidance states |
| `StateModels.swift` | `ApexState` from `/api/state` (all fields optional) |
| `RobotStore.swift` | 1 s status polling, commands, steering coalescing, route editor |
| `MJPEGStream.swift` | Pulls `/video_feed` and cuts JPEG frames out of the stream |
| `LocationProvider.swift` | One-shot phone location for the "Me" waypoint button |
| `Theme.swift` | APEX red/black theme, cards, buttons, haptics |
| `Views/` | The four tabs |

## Notes

- `Info.plist` sets `NSAllowsLocalNetworking` so plain HTTP to `apex.local` and local IPs works.
- Polling and video stop while the app is in the background.
- Motor current cells turn amber at 5 A and red at 8 A (display only, `SystemsView.swift`).
- Map imagery needs internet. On the robot hotspot the map background may be blank, but the
  robot, waypoints and route still draw.
- This was written without a compiler. If Xcode 15.4 reports errors on first build, send them over.
