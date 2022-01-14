@echo off
setlocal enabledelayedexpansion
set folder=%1
echo !folder!
if not exist !folder! (
    echo Input folder doesn't exist! Exiting ...
    exit 0
)

if not exist !folder!\gpx mkdir !folder!\gpx
for %%A in (!folder!\raw\*.360) do (
  set fname=!folder!\gpx\%%~nA
  gopro2gpx %%A !fname!
)

@REM set fov=70 80 90 100 110 120
@REM set angle=30 45 60
@REM for %%a in (%fov%) do (
@REM     for %%b in (%angle%) do (
@REM         set outfolder=!folder!_processed_fov_%%a_angle_%%b
@REM         if not exist !outfolder! mkdir !outfolder!
@REM         for %%A in (!folder!\export\*.mp4) do (
@REM           set gpxfname=!folder!\gpx\%%~nA.gpx
@REM           "D:\Projects\GOSAIFE\Repositories\gopro2gpx\venv-win\Scripts\python.exe" "D:\Projects\GOSAIFE\Repositories\gopro2gpx\misc\extract_frames_by_gps_distance.py" --gpx !gpxfname! --vid %%A --output !outfolder! --fov %%a --f_angle %%b
@REM         )
@REM     )
@REM )

if not exist !folder!_processed mkdir !folder!_processed
for %%A in (!folder!\export\*.mp4) do (
  set gpxfname=!folder!\gpx\%%~nA.gpx
  "D:\Projects\GOSAIFE\Repositories\gopro2gpx\venv-win\Scripts\python.exe" "D:\Projects\GOSAIFE\Repositories\gopro2gpx\misc\extract_frames_by_gps_distance.py" --gpx !gpxfname! --vid %%A --output !folder!_processed --fov 90 --f_angle 60
)
