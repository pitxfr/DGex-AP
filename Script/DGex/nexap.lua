-- Set up autopilot structures
nexap = {}
nexap.altid = nil
nexap.spdid = nil
nexap.bnkid = nil
nexap.hdgid = nil
nexap.tgtalt = nil
nexap.tgtspd = nil
nexap.tgtbnk = nil
nexap.tgthdg = nil
nexap.maxasc = 30
nexap.maxdsc = -20

nexap.pitchid = nil
nexap.tgtpitch = nil
nexap.rstpitch = nil

nexap.aoaid = nil
nexap.tgtaoa = nil
nexap.rstaoa = nil

nexap.bankid = nil
nexap.tgtbank = nil
nexap.rstbank = nil

nexap.slipid = nil
nexap.tgtslip = nil
nexap.rstslip = nil

nexap.vspdid = nil
nexap.tgtvspd = nil
nexap.rstvspd = nil

nexap.lspdid = nil
nexap.tgtlspd = nil
nexap.rstlspd = nil

nexap.recid = nil
nexap.prevyaw=nil
nexap.rectime=0
nexap.bailid=nil
nexap.ilsid=nil

nexap.launchid = nil
nexap.tgtlaunch = nil
nexap.rstlaunch = nil

nexap.datafile = nil

function setvessel (_v)
    v = _v
    class = _v:get_classname()
    if (class ~= 'DeltaGliderEX') then
        term.out('Warning: Autopilot is designed for use with DeltaGliderEX.')
    end
end

function altitude ()
    alt = v:get_altitude()
    return alt
end

function airspeed ()
    return v:get_airspeed()
end

function slope ()
    as = v:get_horizonairspeedvector()
    xz = math.sqrt (as.x^2 + as.z^2)
    sl = math.atan2 (as.y, xz)
    return sl
end

function getPitch()
  return v:get_pitch()*DEG
end

function getAoa()
  return v:get_aoa()
end

function getBank()
  return v:get_bank()
end

function getYaw()
  return v:get_yaw()*DEG
end

function getSlip()
  return v:get_slipangle()*DEG
end

function getVspd()
  as = v:get_horizonairspeedvector()
  return as.y
end

function getLspd()
  as = v:get_horizonairspeedvector()
  return math.sqrt (as.x^2 + as.z^2)
end

function getTurnRate()
  yaw=getYaw()
  local dt=oapi.get_simstep()
  if nexap.prevyaw==nil then
    turnRate=0
  else
    turnRate=(nexap.prevyaw-yaw)/dt
  end
  nexap.prevyaw=yaw
  return turnRate
end

function getRecTime()
  local dt=oapi.get_simstep()
  nexap.rectime=nexap.rectime+dt
  return nexap.rectime
end

function setElevator(val)
  elev = v:get_adclevel(AIRCTRL.ELEVATOR)+val
  if elev > 1 then elev = 1 elseif elev < -1 then elev = -1 end
  v:set_adclevel(AIRCTRL.ELEVATOR,elev)
  return elev
end

function setAileron(val)
  ail = v:get_adclevel(AIRCTRL.AILERON)-val
  if ail > 1 then ail = 1 elseif ail < -1 then ail = -1 end
  v:set_adclevel(AIRCTRL.AILERON,ail)
  return ail
end

function setRudder(val)
  rud = v:get_adclevel(AIRCTRL.RUDDER)+val
  if rud > 1 then rud = 1 elseif rud < -1 then rud = -1 end
  v:set_adclevel(AIRCTRL.RUDDER,rud)
  return rud
end

function setMainThruster(val)
  thrott = v:get_thrustergrouplevel (THGROUP.MAIN)+val
  if thrott > 1 then thrott = 1 elseif thrott < 0 then thrott = 0 end
  v:set_thrustergrouplevel (THGROUP.MAIN, thrott)
  return thrott
end

function setElevatorLspd(val)
  pitch=getPitch()
  if(math.abs(pitch)<5) or (pitch/val>0) then
    elev = -1.*val
  else
    elev = 0
  end
  if elev > 1 then elev = 1 elseif elev < -1 then elev = -1 end
  v:set_adclevel(AIRCTRL.ELEVATOR,elev)
  return elev
end

--[[ --------------------------------------------------------
pitch autopilot (elevator control)
argument 1: target pitch (deg)
--]]
function pitch_ap(pitch)
  if pitch == nil then
    pitch = 5.
  end
  nexap.tgtpitch = pitch
  --pid_ap(getPitch, 'tgtpitch', setElevator, 0.3, 0.01, 0., 5., 'rstpitch' )
  pid_ap(getPitch, 'tgtpitch', setElevator, 0.1, 0., 0.25, 30., 'rstpitch' )
end

--[[ --------------------------------------------------------
Bank autopilot (aileron control)
argument 1: target bank (deg)
--]]
function bank_ap(bank)
  if bank == nil then
    bank = 0.
  end
  nexap.tgtbank = bank
  --pid_ap(getBank, 'tgtbank', setAileron, .1, 0., 0., 10., 'rstbank' )
  pid_ap(getBank, 'tgtbank', setAileron, 0.1, 0., 0.25, 30., 'rstbank' )
end

--[[ --------------------------------------------------------
Angle Of Attack autopilot (elevator control)
argument 1: target aoa (deg)
--]]
function aoa_ap(aoa)
  if aoa == nil then
    aoa = 1*RAD
  end
  nexap.tgtaoa = aoa
  --pid_ap(getAoa, 'tgtaoa', setElevator, 2., 0.8, 0., 2., 'rstaoa' )
  pid_ap(getAoa, 'tgtaoa', setElevator, 0.4, 0., 0.25, 5., 'rstaoa' )

end

--[[ --------------------------------------------------------
Slip autopilot (rudder control)
argument 1: target slip (deg)
--]]
function slip_ap(slip)
  if slip == nil then
    slip = 0.
  end
  nexap.tgtslip = slip
  --pid_ap(getSlip, 'tgtslip', setRudder, .5, 0.05, 0., 5., 'rstslip' )
  pid_ap(getSlip, 'tgtslip', setRudder, .1, 0., 0.25, 30., 'rstslip' )
end

--[[ --------------------------------------------------------
vspd autopilot (throtte control)
argument 1: target vspd (deg)
--]]
function vspd_ap(vspd)
  if vspd == nil then
    vspd= 0
  end
  nexap.tgtvspd = vspd
  --pid_ap(getVspd, 'tgtvspd', setMainThruster, .3, 0.01, 0., 100., 'rstvspd' )
  pid_ap(getVspd, 'tgtvspd', setMainThruster, .1, 0., .25, 100., 'rstvspd' )
end

--[[ --------------------------------------------------------
lspd autopilot (elevator control)
argument 1: target lspd (deg)
--]]
function lspd_ap(lspd)
  if lspd == nil then
    lspd= 0
  end
  nexap.tgtlspd = lspd
  pid_ap(getLspd, 'tgtlspd', setElevator, -0.1, 0., -1., 100., 'rstlspd' )
end

--[[ --------------------------------------------------------
heading autopilot (aileron+elevator control)
argument 1: target hdg (deg)
--]]
function heading_ap(hdg)
  if hdg == nil then
    hdg= 90
  end
  nexap.tgthdg = hdg
  pid_ap(getYaw, 'tgthdg', setAileron, -0.001, 0., -0.02, 100., 'rsthdg' )
end

--[[ --------------------------------------------------------
altitude autopilot (elevator control)
argument 1: target altitude (m)
note: does not terminate (run as background job and kill
      when done)
--]]

function alt_ap (alt)
    if alt == nil then
        alt = 20e3
    end
    nexap.tgtalt = alt
    local dslope0 = 0
    local dslope_rate
    while true do
        if nexap.tgtalt ~= nil then
            local dt = oapi.get_simstep()
            alt = altitude()
            dalt = nexap.tgtalt-alt
            tgt_slope = RAD*1e-2 * dalt
	        if tgt_slope > nexap.maxasc*RAD then
		        tgt_slope = nexap.maxasc*RAD
	        elseif tgt_slope < nexap.maxdsc*RAD then
		        tgt_slope = nexap.maxdsc*RAD
	        end
            dslope = tgt_slope-slope()
            if dslope0 == 0 then
                dslope_rate = 0
            else
                dslope_rate = (dslope-dslope0)/dt
            end
            dslope0 = dslope
            delev = (dslope*0.1 + dslope_rate)*(dt*10.0)
            elev = v:get_adclevel(AIRCTRL.ELEVATOR)
            elev = elev+delev;
            if elev > 1 then elev = 1 elseif elev < -1 then elev = -1 end
            v:set_adclevel(AIRCTRL.ELEVATOR,elev)
        end
        proc.skip()
    end
end

--[[ --------------------------------------------------------
airspeed autopilot (throttle control)
argument 1: target speed (m/s)
note: does not terminate (run as background job and kill
      when done)
--]]

function spd_ap (spd)
    if spd ~= nil then nexap.tgtspd = spd end
    local spd0 = airspeed()
    local acc, dpsd
    local alpha = 1
    local beta = 1
    while true do
        if nexap.tgtspd ~= nil then
            local dt = oapi.get_simstep()
            spd = airspeed()
            acc = (spd-spd0)/dt
            spd0 = spd
            dspd = nexap.tgtspd-spd
            dthrott = (dspd*alpha - acc*beta)*dt
            thrott = v:get_thrustergrouplevel (THGROUP.MAIN)
            v:set_thrustergrouplevel (THGROUP.MAIN, thrott+dthrott)
        end
	  proc.skip()
    end
end

--[[ --------------------------------------------------------
bank autopilot (aileron control)
argument 1: bank angle [deg]
note: does not terminate (run as background job and kill
      when done)
--]]

--function bank_ap (bnk)
--    nexap.tgtbnk = bnk
--    local bnk0 = v:get_bank()
--    while true do
--        local dt = oapi.get_simstep()
--        bnk = v:get_bank()
--        dbnk = bnk-bnk0
--        if dbnk < -PI then dbnk = dbnk+2*PI elseif dbnk > PI then dbnk = dbnk-2*PI end -- phase unwrap
--        rate = dbnk/dt
--        bnk0 = bnk
--        dbnk = nexap.tgtbnk*RAD - bnk
--        if dbnk < -PI then dbnk = dbnk+2*PI elseif dbnk > PI then dbnk = dbnk-2*PI end -- phase unwrap
--        dail = (-dbnk*0.1 + rate*0.3)*(dt*5.0) -- the damping term should really depend on atmospheric density
--        ail = v:get_adclevel(AIRCTRL.AILERON)
--        ail = ail+dail
--        if ail > 1 then ail = 1 elseif ail < -1 then ail = -1 end
--        v:set_adclevel(AIRCTRL.AILERON,ail)
--        proc.skip()
--    end
--end

--[[ --------------------------------------------------------
heading autopilot (aileron+pitch control)
argument 1: heading angle [deg]
note: does not terminate (run as background job and kill
      when done)
--]]

--function heading_ap (hdg)
--    nexap.tgthdg = hdg
--    local hdg0 = v:get_yaw()
--    local tgtbank = v:get_bank()
--    while true do
--        local dt = oapi.get_simstep()
--        hdg = v:get_yaw()
--        dhdg = hdg-hdg0
--        if dhdg < -PI then dhdg = dhdg+2*PI elseif dhdg > PI then dhdg = dhdg-2*PI end -- phase unwrap
--        rate = dhdg/dt
--        hdg0 = hdg
--        dhdg = nexap.tgthdg*RAD - hdg
--        if dhdg < -PI then dhdg = dhdg+2*PI elseif dhdg > PI then dhdg = dhdg-2*PI end -- phase unwrap
--        dbank = (-dhdg*400 + rate*4000)*dt
--        tgtbank = v:get_bank() + dbank
--	  if tgtbank > 0.3*PI then tgtbank = 0.3*PI elseif tgtbank < -0.3*PI then tgtbank = -0.3*PI end
--        nexap.tgtbnk = tgtbank*DEG
--        proc.skip()
--    end
--end

--[[ --------------------------------------------------------
PID autopilot
argument 1: error function
argument 2: command function
argument 3: kp (proportinnal) factor 
argument 4: ki (integrated) factor
argument 5: kd (derived) factor 
argument 6: integrated range (sliding window)
argument 7: reset keyword
--]]
function pid_ap(measure,target,command,kp,ki,kd,satlvl,reset)
  term.out('AP: Entering in PID program. Current target:'..nexap[target]..'['..target..']')
  local err0=0
  local errSum=0
  local errRate=0
  while true do
    if (nexap[reset]) then
      err0=0
      errSum=0
      errRate=0
      nexap[reset]=false
    end
    local dt = oapi.get_simstep()
    err = nexap[target]-measure()
    if math.abs(err) > satlvl then  -- Saturation treatment
      pidCor = err
      nexap[reset]=true
    else
      pcr = kp*err
      errSum = errSum+err*dt
      icr = ki*errSum
      errRate=(err-err0)/dt
      dcr = kd*errRate
      pidCor = pcr+icr+dcr
    end
    if pidCor > 1. then pidCor = 1. elseif pidCor < -1. then pidCor = -1. end -- Saturation
    val=command(pidCor)
    --if nexap.datafile ~= nil then nexap.datafile:write(target..';'..measure()..';'..err..';'..errSum..';'..errRate..';'..pidCor..'\n') end
    err0=err
    proc.skip()
  end
end

--[[ --------------------------------------------------------
Launch autopilot
argument 1: heading
--]]
function launch_ap(hdg)
  note = oapi.create_annotation()
  note:set_pos (0.25,0.1,0.8,0.95)
  note:set_colour ({r=0.9,g=0.5,b=0.2})
  note:set_text("Launch test")
  if hdg == nil then
    hdg = 90
  end
  nexap.bank(0)
  nexap.spd(200)
  nexap.pitch(10)
  while altitude() < 30 do proc.skip() end
  note:set_text("30 meters, Gear UP")
  res=v:send_bufferedkey(ktable.G)
  while altitude() < 500 do proc.skip() end
  note:set_text("500 meters, pitch 5 deg")
  nexap.pitch(5)
  --v:set_thrustergrouplevel (THGROUP.MAIN, 1)
  while altitude() < 5e3 do proc.skip() end
  note:set_text("5 000 meters, end of test")
  nexap.bank()
  nexap.spd()
  nexap.pitch()
end

--[[ --------------------------------------------------------
Record flight values
--]]
function rec_ap()
  while true do
    --local dt=oapi.get_simstep()
    --if nexap.datafile ~= nil then nexap.datafile:write(getRecTime()..'\n') end
    if nexap.datafile ~= nil then nexap.datafile:write(getRecTime()..';'..altitude()..';'..airspeed()..';'..getVspd()..';'..getLspd()..';'..getPitch()..';'..getBank()..';'..getYaw()..';'..getTurnRate()..'\n') end
    proc.skip()
  end
end

function bail_ap(dtime)
  if dtime>0 then bnk=1 else bnk=-1 end
  nexap.bank(bnk)
  local count=0
  while count<math.abs(dtime) do
    local dt=oapi.get_simstep()
    count=count+dt
    proc.skip()
  end
  nexap.bank(0)
  count=0
  while count<math.abs(dtime) do
    local dt=oapi.get_simstep()
    count=count+dt
    proc.skip()
  end
  nexap.bank(-1.*bnk)
  count=0
  while count<math.abs(dtime) do
    local dt=oapi.get_simstep()
    count=count+dt
    proc.skip()
  end
  nexap.bank(0)
end

function lin_ap(ang)
  yaw=getYaw()
  yawRetSt=0.3
  yawRetEd=0.1
  yawAntSt=yaw+ang-yawRetSt
  yawAntEd=yaw+ang-yawRetEd
  if ang>0 then cbnk=1 else cbnk=-1 end
  nexap.bank(cbnk*5)
  while yaw<yawAntSt do
    local crtyaw=getYaw()
    yaw=crtyaw 
    proc.skip()
  end
  nexap.bank(-1.*cbnk*(5.+ang))
  while yaw<yawAntEd do
    local crtyaw=getYaw()
    yaw=crtyaw 
    proc.skip()
  end
  nexap.bank(0)
end


function vor_rad()
  note = oapi.create_annotation()
  note:set_pos (0.25,0.1,0.8,0.95)
  note:set_colour ({r=0.9,g=0.5,b=0.2})
  freq=0.25
  dtime=0
  while true do
    local dt=oapi.get_simstep()
    dtime=dtime+dt
    if dtime > freq then
      hnav=v:get_navsource(0)
      pos=oapi.get_navpos(hnav)
      epos=oapi.global_to_equ(v:get_handle(),pos)
      yaw=getYaw()
      radial=360-(180.-yaw+epos.lng*DEG)
      note:set_text("radial:"..radial)
    end
    proc.skip()
  end
end

-- -----------------------------------------------------------
-- User level entry functions
-- -----------------------------------------------------------

function nexap.testbank()
  note = oapi.create_annotation()
  note:set_pos (0.25,0.1,0.8,0.95)
  note:set_colour ({r=0.9,g=0.5,b=0.2})
  note:set_text("Launch bank test")
  nexap.open()
  nexap.pitch(10)
  val=setMainThruster(1.)
  while altitude() < 30 do proc.skip() end
  note:set_text("30 meters, Gear UP")
  res=v:send_bufferedkey(ktable.G)
  while altitude() < 500 do proc.skip() end
  note:set_text("500 meters, bank 5 deg")
  nexap.pitch(5)
  nexap.bank(5.)
  while altitude() < 700  do proc.skip() end
  note:set_text("end of test")
  nexap.bank()
  nexap.pitch()
  nexap.close()
end

function nexap.fly()
  nexap.alt(2000)
  nexap.spd(150)
  nexap.bank(0)
  nexap.slip(0)
end

function nexap.open()
  nexap.datafile = io.open("nexap.data","w")
  nexap.datafile:write("# id; measure; err; sum; rate; pid \n")
end

function nexap.close()
  nexap.datafile:close()
  nexap.datafile=nil
end

function nexap.rec()
  if nexap.datafile == nil then
    nexap.datafile = io.open("nexap_rec.data","w")
    nexap.datafile:write("# time; altitude; speed; vertical speed; ground speed; pitch; bank; yaw; turn rate \n")
    nexap.recid = proc.bg(rec_ap)
    term.out('AP: Record of flight parameters')
  else
    proc.kill(nexap.recid)
    nexap.close()
    nexap.rectime=0
    nexap.yawrate=nil
    term.out('AP: End of record')
  end
end

function nexap.tt (val)
  cmd=setMainThruster(val)
end

function nexap.w ()
  val = v:get_adclevel(AIRCTRL.ELEVATOR)
  term.out('AIRCTRL.ELEVATOR: '..val)
end

function nexap.rst()
  nexap.pitch()
  nexap.bank()
  nexap.aoa()
  nexap.slip()
  nexap.vspd()
  nexap.lspd()
  nexap.hdg()
  nexap.spd()
  nexap.alt()
end

function nexap.inter()
  nexap.alt(900)
  nexap.spd(100)
  nexap.bank(0)
  nexap.slip(0)
end

function nexap.stop()
  nexap.rst()
  nexap.spd(0)
end

function nexap.final()
  pitch=getPitch()
  nexap.alt()
  nexap.pitch(pitch)
  nexap.spd()
  nexap.vspd(-5.24)
end

-- Invoke baillonnette autopilot as background job

function nexap.ils(val)
  --hnav=v:get_navsource(1)
  --pos=oapi.get_navpos(hnav)
  --epos=oapi.global_to_equ(v:get_handle(),pos)
  --term.out(epos.lng*DEG..' , '..epos.lat*DEG..' , '..epos.rad..'\n')
  if val == nil then
        if nexap.ilsid ~= nil then
            term.out('AP: ils display disabled')
            proc.kill(nexap.ilsid)
            nexap.ilsid = nil
        end
    else
        term.out('AP: ils display')
        if nexap.ilsid == nil then
            nexap.ilsid = proc.bg(vor_rad) -- launch pitch autopilot
        else
            proc.kill(nexap.ilsid)
            nexap.ilsid = nil
            nexap.ilsid = proc.bg(vor_rad)
        end
    end
end

function nexap.bail (dtime)
    if dtime == nil then
        if nexap.bailid ~= nil then
            term.out('AP: baillonnette autopilot disabled')
            proc.kill(nexap.bailid)
            nexap.bailid = nil
            nexap.bank(0)
        end
    else
        term.out('AP: baillonnette time '..dtime..' s')
        if nexap.bailid == nil then
            nexap.bailid = proc.bg(bail_ap,dtime) -- launch pitch autopilot
        else
            proc.kill(nexap.bailid)
            nexap.bailid = nil
            nexap.bailid = proc.bg(bail_ap,dtime)
        end
    end
end

-- Invoke aline autopilot as background job

function nexap.lin (ang)
    if ang == nil then
        if nexap.linid ~= nil then
            term.out('AP: angle correction autopilot disabled')
            proc.kill(nexap.linid)
            nexap.linid = nil
            nexap.bank(0)
        end
    else
        term.out('AP: Ang correction '..ang..' deg')
        if nexap.linid == nil then
            nexap.linid = proc.bg(lin_ap,ang) -- launch pitch autopilot
        else
            proc.kill(nexap.linid)
            nexap.linid = nil
            nexap.linid = proc.bg(lin_ap,ang)
        end
    end
end

-- Invoke pitch autopilot as background job

function nexap.pitch (pitch)
    if pitch == nil then
        if nexap.pitchid ~= nil then
            term.out('AP: pitch autopilot disabled')
            proc.kill(nexap.pitchid)
            nexap.pitchid = nil
            --v:set_adclevel(AIRCTRL.ELEVATOR,0)
        end
    else
        term.out('AP: pitch target '..pitch..' deg')
        if nexap.pitchid == nil then
            nexap.pitchid = proc.bg(pitch_ap,pitch) -- launch pitch autopilot
        else
            nexap.tgtpitch = pitch -- autopilot already running: just reset pitch
            nexap.rstpitch = true
        end
    end
end

-- Invoke AOA autopilot as background job

function nexap.aoa (aoa)
    if aoa == nil then
        if nexap.aoaid ~= nil then
            term.out('AP: AOA autopilot disabled')
            proc.kill(nexap.aoaid)
            nexap.aoaid = nil
            --v:set_adclevel(AIRCTRL.ELEVATOR,0)
        end
    else
        term.out('AP: AOA target '..aoa..' deg')
        if nexap.aoaid == nil then
            nexap.aoaid = proc.bg(aoa_ap,aoa*RAD) -- launch aoa autopilot
        else
            nexap.tgtaoa = aoa*RAD -- autopilot already running: just reset aoa
            nexap.rstaoa = true
        end
    end
end

-- Invoke altitude autopilot as background job

function nexap.alt (alt)
    if alt == nil then
        if nexap.altid ~= nil then
            term.out('AP: altitude autopilot disabled')
            proc.kill(nexap.altid)
            nexap.altid = nil
            --v:set_adclevel(AIRCTRL.ELEVATOR,0)
        end
    else
        term.out('AP: altitude target '..alt..' m')
        if nexap.altid == nil then
            nexap.altid = proc.bg(alt_ap,alt) -- launch altitude autopilot
        else
            nexap.tgtalt = alt -- autopilot already running: just reset altitude
        end
    end
end

-- Invoke speed autopilot as background job

function nexap.spd (spd)
    if spd == nil then
        if nexap.spdid ~= nil then
            term.out('AP: airspeed autopilot disabled')
            proc.kill(nexap.spdid)
            nexap.spdid = nil
        end
    else
        term.out('AP: airspeed target '..spd..' m/s')
        if nexap.spdid == nil then
            nexap.spdid = proc.bg(spd_ap,spd) -- launch airspeed autopilot
        else
            nexap.tgtspd = spd -- autopilot already running: just reset target speed
        end
    end
end

-- Invoke bank autopilot as background job

function nexap.bank (bank)
    if bank == nil then
        if nexap.bankid ~= nil then
            term.out('AP: bank autopilot disabled')
            proc.kill(nexap.bankid)
            nexap.bankid = nil
            v:set_adclevel(AIRCTRL.AILERON,0)
        end
    else
        term.out('AP: bank target '..bank..' deg')
        if nexap.bankid == nil then
            nexap.bankid = proc.bg(bank_ap,bank*RAD) -- launch bank autopilot
        else
            nexap.tgtbank = bank*RAD -- autopilot already running: just reset target angle
            nexap.rstbank = true
            nexap.rstslip = true
        end
    end
end

-- Invoke slip autopilot as background job

function nexap.slip (slip)
    if slip == nil then
        if nexap.slipid ~= nil then
            term.out('AP: slip autopilot disabled')
            proc.kill(nexap.slipid)
            nexap.slipid = nil
            v:set_adclevel(AIRCTRL.RUDDER,0)
        end
    else
        term.out('AP: slip target '..slip..' deg')
        if nexap.slipid == nil then
            nexap.slipid = proc.bg(slip_ap,slip) -- launch slip autopilot
        else
            nexap.tgtslip = slip -- autopilot already running: just reset target angle
            nexap.rstslip = true

        end
    end
end

-- Invoke vspd autopilot as background job

function nexap.vspd (vspd)
    if vspd == nil then
        if nexap.vspdid ~= nil then
            term.out('AP: vspd autopilot disabled')
            proc.kill(nexap.vspdid)
            nexap.vspdid = nil
            --v:set_thrustergrouplevel (THGROUP.MAIN, 0.8)
        end
    else
        term.out('AP: vspd target '..vspd..' deg')
        if nexap.vspdid == nil then
            nexap.vspdid = proc.bg(vspd_ap,vspd) -- launch vspd autopilot
        else
            nexap.tgtvspd = vspd -- autopilot already running: just reset target angle
            nexap.rstvspd = true

        end
    end
end

-- Invoke lspd autopilot as background job

function nexap.lspd (lspd)
    if lspd == nil then
        if nexap.lspdid ~= nil then
            term.out('AP: lspd autopilot disabled')
            proc.kill(nexap.lspdid)
            nexap.lspdid = nil
            --v:set_adclevel(AIRCTRL.AILERON,0)
        end
    else
        term.out('AP: lspd target '..lspd..' deg')
        if nexap.lspdid == nil then
            nexap.lspdid = proc.bg(lspd_ap,lspd) -- launch lspd autopilot
        else
            nexap.tgtlspd = lspd -- autopilot already running: just reset target angle
            nexap.rstlspd = true

        end
    end
end

-- Invoke heading autopilot as background job

function nexap.hdg (hdg)
    if hdg == nil then
        if nexap.hdgid ~= nil then
            term.out('AP: heading autopilot disabled')
            proc.kill(nexap.hdgid)
            --nexap.bank() -- kill bank ap
            nexap.bank(0)
            nexap.hdgid = nil
        end
    else
        term.out('AP: heading target '..hdg..' deg')
        if nexap.hdgid == nil then
            nexap.bank()
            nexap.slip(0)
            nexap.pitch(1.2)
            nexap.hdgid = proc.bg(heading_ap,hdg) -- launch heading autopilot
            --nexap.bank(0) -- invoke bank ap
        else
            nexap.tgthdg = hdg -- autopilot already running: just reset target angle
        end
    end
end

-- Invoke launch autopilot as background job

function nexap.launch (hdg)
    if hdg == nil then
        if nexap.launchid ~= nil then
            term.out('AP: Launch autopilot disabled')
            proc.kill(nexap.launchid)
            nexap.launchid = nil
            v:set_adclevel(AIRCTRL.ELEVATOR,0)
        end
    else
        term.out('AP: Launch autopilot enable')
        if nexap.launchid == nil then
            nexap.launchid = proc.bg(launch_ap,hdg) -- launch aoa autopilot
        else
            nexap.tgthdg = hdg -- autopilot already running: just reset aoa
            nexap.rsthdg = true
        end
    end
end
-- -----------------------------------------------------------
-- Initialisation code

v = {}

setvessel(vessel.get_focusinterface())

term.out('DgEx: new Atmospheric autopilot loaded.')
