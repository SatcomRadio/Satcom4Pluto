@echo off
setlocal
title satcom4pluto receiver - keep this window open (minimise it)

rem ======================= SETTINGS (edit here) =======================
rem After a change: close this window and start satcom4pluto.bat again.

rem Address of the receiver (Pluto). A stock ADALM-Pluto uses 192.168.2.1.
set "PLUTO=192.168.5.5"

rem Radio: centre 255 MHz, 36.864 MS/s (240-270 MHz), gain 30 dB, analog bandwidth 40 MHz
set "LO_HZ=255000000"
set "RATE=36864000"
set "GAIN_DB=30"
set "RF_BW=40000000"

rem Detector (sent to the receiver at every start):
rem   -f 240:270            band in MHz whose downlinks are sent (repeat -f for several ranges)
rem   --max-snr-db 40       jammer filter: block anything peaking more than 40 dB above the noise
rem   --kernel sin3         FPGA detection smoothing (sin1 / sin3 / sin5)
rem   --pass 248.9:249.9    optional: always send this range as real IQ, e.g. a block of 5 kHz
rem                         channels the detector cannot separate; repeat (max 16)
rem   --max-bw-khz 150      optional: widest single detection in kHz (default 150)
rem   --spur 250.0          optional: ignore a known birdie at this frequency (MHz); repeat (max 32)
rem   All options are explained in README.md, section "Detector options".
set "DETECTOR=-f 240:270 --max-snr-db 40 --kernel sin3 --pass 248.9:249.27"

rem Port for SDRangel (Remote TCP Input, protocol SDRangel)
set "PORT=1234"
rem =====================================================================

cd /d "%~dp0"
set "SSHOPT=-o StrictHostKeyChecking=no -o UserKnownHostsFile=NUL -o LogLevel=ERROR"

:wait
ping -n 1 -w 1000 %PLUTO% | find "TTL=" >nul
if errorlevel 1 (
  echo Waiting for the receiver at %PLUTO% ... is it switched on and plugged in?
  timeout /t 3 /nobreak >nul
  goto wait
)

rem Is satcom4pluto running on the Pluto (detector on TCP port 30470)? If not -- the Pluto was
rem just powered up -- load the FPGA image and the detector into its RAM (nothing is flashed).
powershell -NoProfile -Command "try { $c = New-Object Net.Sockets.TcpClient; $c.Connect('%PLUTO%', 30470); $c.Close(); exit 0 } catch { exit 1 }" >nul 2>&1
if not errorlevel 1 goto run
echo.
ssh-keygen -R 192.168.5.5
echo Loading satcom4pluto into the receiver. When asked for the password, type: analog
scp -O %SSHOPT% sgch_pl.bin pluto_tryload.sh sgchd root@%PLUTO%:/tmp/
ssh %SSHOPT% root@%PLUTO% "chmod +x /tmp/sgchd; sh /tmp/pluto_tryload.sh /tmp/sgch_pl.bin && (nohup /tmp/sgchd > /tmp/sgchd.log 2>&1 < /dev/null &); sleep 2"
timeout /t 2 /nobreak >nul

:run
echo Receiver ready - starting.
sgch_rx.exe -u ip:%PLUTO% -f %LO_HZ% -s %RATE% -g %GAIN_DB% -B %RF_BW% -x "%DETECTOR%" --server %PORT%
echo.
echo Connection to the receiver lost - starting again in 5 seconds ...
timeout /t 5 /nobreak >nul
goto wait
