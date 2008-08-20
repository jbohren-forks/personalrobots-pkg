# Microsoft Developer Studio Project File - Name="propgridsample" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=propgridsample - Win32 Static ANSI Release Multilib
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "propgrid_propgridsample.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "propgrid_propgridsample.mak" CFG="propgridsample - Win32 Static ANSI Release Multilib"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "propgridsample - Win32 DLL Unicode Debug Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL Unicode Debug Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL Unicode Release Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL Unicode Release Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL ANSI Debug Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL ANSI Debug Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL ANSI Release Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 DLL ANSI Release Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static Unicode Debug Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static Unicode Debug Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static Unicode Release Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static Unicode Release Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static ANSI Debug Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static ANSI Debug Multilib" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static ANSI Release Monolithic" (based on "Win32 (x86) Application")
!MESSAGE "propgridsample - Win32 Static ANSI Release Multilib" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "propgridsample - Win32 DLL Unicode Debug Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswud_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswud_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28ud_propgrid.lib wxmsw28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28ud_propgrid.lib wxmsw28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL Unicode Debug Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswud_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswud_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28ud_propgrid.lib wxbase28ud_xml.lib wxmsw28ud_xrc.lib wxmsw28ud_adv.lib wxmsw28ud_core.lib wxbase28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28ud_propgrid.lib wxbase28ud_xml.lib wxmsw28ud_xrc.lib wxmsw28ud_adv.lib wxmsw28ud_core.lib wxbase28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL Unicode Release Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswu_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswu_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28u_propgrid.lib wxmsw28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28u_propgrid.lib wxmsw28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL Unicode Release Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswu_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswu_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28u_propgrid.lib wxbase28u_xml.lib wxmsw28u_xrc.lib wxmsw28u_adv.lib wxmsw28u_core.lib wxbase28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28u_propgrid.lib wxbase28u_xml.lib wxmsw28u_xrc.lib wxmsw28u_adv.lib wxmsw28u_core.lib wxbase28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL ANSI Debug Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswd_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswd_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28d_propgrid.lib wxmsw28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28d_propgrid.lib wxmsw28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL ANSI Debug Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswd_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswd_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_dll\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28d_propgrid.lib wxbase28d_xml.lib wxmsw28d_xrc.lib wxmsw28d_adv.lib wxmsw28d_core.lib wxbase28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28d_propgrid.lib wxbase28d_xml.lib wxmsw28d_xrc.lib wxmsw28d_adv.lib wxmsw28d_core.lib wxbase28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL ANSI Release Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmsw_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmsw_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28_propgrid.lib wxmsw28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28_propgrid.lib wxmsw28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 DLL ANSI Release Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmsw_dll\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmsw_dll\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_dll\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "WXUSINGDLL" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_dll\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "WXUSINGDLL" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_dll\wxcode_msw28_propgrid.lib wxbase28_xml.lib wxmsw28_xrc.lib wxmsw28_adv.lib wxmsw28_core.lib wxbase28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"
# ADD LINK32 ..\lib\vc_dll\wxcode_msw28_propgrid.lib wxbase28_xml.lib wxmsw28_xrc.lib wxmsw28_adv.lib wxmsw28_core.lib wxbase28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_dll" /libpath:"..\lib\vc_dll"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static Unicode Debug Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswud\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswud\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28ud_propgrid.lib wxmsw28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28ud_propgrid.lib wxmsw28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static Unicode Debug Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswud\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswud\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswud" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswud" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28ud_propgrid.lib wxbase28ud_xml.lib wxmsw28ud_xrc.lib wxmsw28ud_adv.lib wxmsw28ud_core.lib wxbase28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28ud_propgrid.lib wxbase28ud_xml.lib wxmsw28ud_xrc.lib wxmsw28ud_adv.lib wxmsw28ud_core.lib wxbase28ud.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexud.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static Unicode Release Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswu\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswu\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28u_propgrid.lib wxmsw28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28u_propgrid.lib wxmsw28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static Unicode Release Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswu\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswu\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\mswu" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "_UNICODE" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswu" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "_UNICODE" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28u_propgrid.lib wxbase28u_xml.lib wxmsw28u_xrc.lib wxmsw28u_adv.lib wxmsw28u_core.lib wxbase28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28u_propgrid.lib wxbase28u_xml.lib wxmsw28u_xrc.lib wxmsw28u_adv.lib wxmsw28u_core.lib wxbase28u.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregexu.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static ANSI Debug Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswd\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswd\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28d_propgrid.lib wxmsw28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28d_propgrid.lib wxmsw28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static ANSI Debug Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmswd\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmswd\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD CPP /nologo /MDd /W4 /Gm /GR /Zi /Od /I "$(WXWIN)\lib\vc_lib\mswd" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /GZ /c
# ADD BASE MTL /nologo /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "__WXDEBUG__" /D "__WXMSW__" /D "_WINDOWS" /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\mswd" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXDEBUG__" /d "__WXMSW__" /d "_WINDOWS" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28d_propgrid.lib wxbase28d_xml.lib wxmsw28d_xrc.lib wxmsw28d_adv.lib wxmsw28d_core.lib wxbase28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28d_propgrid.lib wxbase28d_xml.lib wxmsw28d_xrc.lib wxmsw28d_adv.lib wxmsw28d_core.lib wxbase28d.lib wxtiffd.lib wxjpegd.lib wxpngd.lib wxzlibd.lib wxregexd.lib wxexpatd.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /debug /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static ANSI Release Monolithic"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmsw\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmsw\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28_propgrid.lib wxmsw28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28_propgrid.lib wxmsw28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ELSEIF  "$(CFG)" == "propgridsample - Win32 Static ANSI Release Multilib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\samples"
# PROP BASE Intermediate_Dir "vcmsw\propgridsample"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\samples"
# PROP Intermediate_Dir "vcmsw\propgridsample"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD CPP /nologo /MD /W4 /GR /O2 /I "$(WXWIN)\lib\vc_lib\msw" /I "$(WXWIN)\include" /I "..\include" /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /Fd"..\samples\propgridsample.pdb" /FD /EHsc /c
# ADD BASE MTL /nologo /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD MTL /nologo /D "WIN32" /D "__WXMSW__" /D "_WINDOWS" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXMSW__" /d "_WINDOWS"
# ADD RSC /l 0x409 /i "$(WXWIN)\lib\vc_lib\msw" /i "$(WXWIN)\include" /i "..\include" /i "..\samples" /d "__WXMSW__" /d "_WINDOWS"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ..\lib\vc_lib\wxcode_msw28_propgrid.lib wxbase28_xml.lib wxmsw28_xrc.lib wxmsw28_adv.lib wxmsw28_core.lib wxbase28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"
# ADD LINK32 ..\lib\vc_lib\wxcode_msw28_propgrid.lib wxbase28_xml.lib wxmsw28_xrc.lib wxmsw28_adv.lib wxmsw28_core.lib wxbase28.lib wxtiff.lib wxjpeg.lib wxpng.lib wxzlib.lib wxregex.lib wxexpat.lib kernel32.lib user32.lib gdi32.lib comdlg32.lib winspool.lib winmm.lib shell32.lib comctl32.lib ole32.lib oleaut32.lib uuid.lib rpcrt4.lib advapi32.lib wsock32.lib odbc32.lib oleacc.lib /nologo /subsystem:windows /machine:I386 /out:"..\samples\propgridsample.exe" /libpath:"$(WXWIN)\lib\vc_lib" /libpath:"..\lib\vc_lib"

!ENDIF 

# Begin Target

# Name "propgridsample - Win32 DLL Unicode Debug Monolithic"
# Name "propgridsample - Win32 DLL Unicode Debug Multilib"
# Name "propgridsample - Win32 DLL Unicode Release Monolithic"
# Name "propgridsample - Win32 DLL Unicode Release Multilib"
# Name "propgridsample - Win32 DLL ANSI Debug Monolithic"
# Name "propgridsample - Win32 DLL ANSI Debug Multilib"
# Name "propgridsample - Win32 DLL ANSI Release Monolithic"
# Name "propgridsample - Win32 DLL ANSI Release Multilib"
# Name "propgridsample - Win32 Static Unicode Debug Monolithic"
# Name "propgridsample - Win32 Static Unicode Debug Multilib"
# Name "propgridsample - Win32 Static Unicode Release Monolithic"
# Name "propgridsample - Win32 Static Unicode Release Multilib"
# Name "propgridsample - Win32 Static ANSI Debug Monolithic"
# Name "propgridsample - Win32 Static ANSI Debug Multilib"
# Name "propgridsample - Win32 Static ANSI Release Monolithic"
# Name "propgridsample - Win32 Static ANSI Release Multilib"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\samples\propgridsample.cpp
# End Source File
# Begin Source File

SOURCE=..\samples\sample.rc
# End Source File
# Begin Source File

SOURCE=..\samples\sampleprops.cpp
# End Source File
# Begin Source File

SOURCE=..\samples\tests.cpp
# End Source File
# Begin Source File

SOURCE=..\src\xh_propgrid.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\samples\propgridsample.h
# End Source File
# Begin Source File

SOURCE=..\samples\sampleprops.h
# End Source File
# End Group
# Begin Source File

SOURCE=..\samples\wx\msw\blank.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\bullseye.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\cdrom.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\computer.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\cross.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\drive.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\file1.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\floppy.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\folder1.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\folder2.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\hand.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\magnif1.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\pbrush.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\pencil.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\pntleft.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\pntright.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\removble.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\rightarr.cur
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\roller.cur
# End Source File
# Begin Source File

SOURCE=..\samples\sample.ico
# End Source File
# Begin Source File

SOURCE=..\samples\wx\msw\std.ico
# End Source File
# End Target
# End Project
