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

## Layout

| Tab      | Contents |
|----------|----------|
| Drive    | Status pills, banners, live MJPEG feed (corner button for full screen with controls), steering, Home x4 / Stand / Go / Stop |
| Navigate | Robot GPS, route map (tap to add waypoints), NAV MODE Start / Pause / Stop, route editor |
| Systems  | Obstacle avoidance, subsystem health, motor currents, leg deactivation |
| Settings | Robot address (default `apex.local`), connection details, open web dashboard in Safari |

## Files

| File | Role |
|------|------|
| `ApexClient.swift` | HTTP calls to every Flask route |
| `Models.swift` | `ApexStatus` from `/status`, waypoint drafts, avoidance states |
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
