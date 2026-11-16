#define MyAppName "IMU Joint Viewer"
#ifndef MyAppVersion
  #error MyAppVersion must be supplied by packaging/build_windows.ps1 -Installer
#endif
#define MyAppPublisher "OpenSourceTools"
#define MyAppExeName "IMUJointViewer.exe"

[Setup]
AppId={{B4A7B0B1-5C1F-4B4C-9A22-3D6A2E5A9D11}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
OutputDir=..\dist-installer
OutputBaseFilename=IMUJointViewer-Setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
UninstallDisplayIcon={app}\{#MyAppExeName}
LicenseFile=..\THIRD_PARTY_NOTICES.md
DisableProgramGroupPage=yes

[Files]
Source: "..\dist\IMUJointViewer\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "..\imu_desktop\*"; DestDir: "{app}\Source\imu_desktop"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "*.pyc"
Source: "..\run_desktop.py"; DestDir: "{app}\Source"; Flags: ignoreversion
Source: "..\pyproject.toml"; DestDir: "{app}\Source"; Flags: ignoreversion
Source: "..\requirements-desktop.txt"; DestDir: "{app}\Source"; Flags: ignoreversion
Source: "..\README.md"; DestDir: "{app}\Source"; Flags: ignoreversion
Source: "..\firmware\*"; DestDir: "{app}\Firmware"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "..\tools\*"; DestDir: "{app}\Source\tools"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "*.pyc"
Source: "..\THIRD_PARTY_NOTICES.md"; DestDir: "{app}\Source"; Flags: ignoreversion
Source: "..\tests\*"; DestDir: "{app}\Source\tests"; Flags: ignoreversion recursesubdirs createallsubdirs; Excludes: "*.pyc"
Source: "..\packaging\imu_joint_viewer.spec"; DestDir: "{app}\Source\packaging"; Flags: ignoreversion

[Run]
Filename: "{app}\{#MyAppExeName}"; Parameters: "--bench"; Description: "Mở màn hình kiểm tra IMU với encoder"; Flags: nowait postinstall skipifsilent

[Icons]
Name: "{group}\Kiểm tra IMU với encoder"; Filename: "{app}\{#MyAppExeName}"; Parameters: "--bench"
Name: "{group}\Đo góc khớp IMU"; Filename: "{app}\{#MyAppExeName}"
